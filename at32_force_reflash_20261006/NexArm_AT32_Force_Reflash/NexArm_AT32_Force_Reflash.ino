#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include "soc/rtc_cntl_reg.h"
#include "CommProtocol.h"
#include "at32_firmware.h"
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* VERSION = "nexarm-at32-force-reflash-r3";

// Official OTA command set (Global.h of factory firmware)
#define CMD_FIRMWARE_UPDATE 90
#define CMD_FW_START        91
#define CMD_FW_DATA         92
#define CMD_FW_END          93
#define CMD_FW_QUERY        94

// Integrity fingerprint of the embedded official 1.0.2 image (21848 bytes).
// Binary SHA256 = 76d1234e563a6ee6202c520cbfa4b794d8ef3c2e626baf72647a1ee5fd63bc2b
// Binary CRC32  = 0x3B2178B8  (verified offline against 出厂程序/Nex_Arm/at32_firmware.h)
static const uint32_t EXPECT_FW_CRC32 = 0x3B2178B8UL;

// Phases: abort (KEY2 / host) honoured only BEFORE erase starts.
// After ERASE begins there is no safe abort; power must be kept stable.
// RESUME_WINDOW: entered at boot when the NVS ota_pend flag is set, i.e. the
// ESP32 was unexpectedly reset mid-OTA (observed on r1: AT32 reset transient
// power-cycled the ESP32 right after CMD90). The flag is cleared only on a
// controlled finish(), so an interrupted OTA always resumes at BL_CHECK.
enum Phase : uint8_t { IDLE, PRE_QUERY, CONFIRM_WINDOW, ENTER_BL, BL_CHECK,
                       ERASE, WRITE, END_FW, VERIFY, FINISHED, RESUME_WINDOW };
static Phase phase = IDLE;
static CommProtocol_t rx, tx;
static bool ready = false, spent = false, ota_ok = false;
static uint32_t stage_ms = 0, last_report = 0;
static uint8_t pre_ver[3] = {0,0,0}, post_ver[3] = {0,0,0};
static bool pre_got = false, post_got = false;
static uint32_t pkt_i = 0, pkt_total = 0;
static const char* reason = "waiting_host";
static char host_line[40] = {}; static uint8_t host_used = 0; static bool host_overflow = false;
static bool host_abort = false;
static uint32_t saved_brownout = 0;
static bool brownout_saved = false;

// NVS persistence of the OTA-in-progress flag (survives any reset/power loss)
static void nvs_set_pend(uint8_t v) { Preferences p; if (p.begin("at32ota", false)) { p.putUChar("pend", v); p.end(); } }
static uint8_t nvs_get_pend() { Preferences p; uint8_t v = 0; if (p.begin("at32ota", true)) { v = p.getUChar("pend", 0); p.end(); } return v; }

static void log_step(const char* step, bool ok) {
    Serial.printf("{\"type\":\"ota_step\",\"step\":\"%s\",\"ok\":%s,\"ms\":%lu}\n",
                  step, ok ? "true" : "false", (unsigned long)millis());
}
static void evidence() {
    Serial.printf("{\"type\":\"reflash_state\",\"firmware\":\"%s\",\"phase\":%u,\"spent\":%s,\"ota_ok\":%s,"
                  "\"pre_ver\":\"%d.%d.%d\",\"pre_got\":%s,\"post_ver\":\"%d.%d.%d\",\"post_got\":%s,"
                  "\"pkt\":%lu,\"pkt_total\":%lu,\"reason\":\"%s\"}\n",
                  VERSION, (unsigned)phase, spent ? "true" : "false", ota_ok ? "true" : "false",
                  pre_ver[0], pre_ver[1], pre_ver[2], pre_got ? "true" : "false",
                  post_ver[0], post_ver[1], post_ver[2], post_got ? "true" : "false",
                  (unsigned long)pkt_i, (unsigned long)pkt_total, reason);
}
static void finish(const char* why) {
    reason = why; phase = FINISHED;
    nvs_set_pend(0); // controlled finish: no resume on next boot
    if (brownout_saved) { REG_WRITE(RTC_CNTL_BROWN_OUT_REG, saved_brownout); brownout_saved = false; }
    evidence();
}

// ---- CRC32 (standard zlib polynomial) over the embedded image ----
static uint32_t fw_crc32() {
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < AT32_FW_SIZE; i++) {
        crc ^= at32_firmware[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}

// ---- raw UART helpers (self-contained copy of the official OTA reply parser) ----
static bool wait_reply(HardwareSerial &uart, uint8_t expect_cmd,
                       uint8_t *out_args, uint8_t *out_args_len, uint32_t timeout_ms) {
    uint32_t start = millis();
    uint8_t state = 0, id = 0, len = 0, cmd = 0;
    uint8_t args[16]; uint8_t arg_idx = 0;
    uint16_t checksum = 0; uint8_t skip_remaining = 0;
    while (millis() - start < timeout_ms) {
        if (!uart.available()) { delay(1); continue; }
        uint8_t ch = uart.read();
        if (skip_remaining > 0) { skip_remaining--; continue; }
        switch (state) {
            case 0: state = (ch == 0xFF) ? 1 : 0; break;
            case 1: state = (ch == 0xFF) ? 2 : 0; break;
            case 2: id = ch; checksum = ch; state = 3; break;
            case 3:
                len = ch; checksum += ch; arg_idx = 0;
                if (len < 2) { state = 0; break; }
                if (len > 16) { skip_remaining = len; state = 0; break; }
                state = 4; break;
            case 4: cmd = ch; checksum += ch; state = (len > 2) ? 5 : 6; break;
            case 5:
                if (arg_idx < 16) args[arg_idx] = ch;
                arg_idx++; checksum += ch;
                if (arg_idx >= len - 2) state = 6;
                break;
            case 6:
                state = 0;
                if (ch == (uint8_t)(~checksum) && cmd == expect_cmd) {
                    if (out_args && out_args_len) {
                        *out_args_len = (arg_idx > 16) ? 16 : arg_idx;
                        memcpy(out_args, args, *out_args_len);
                    }
                    return true;
                }
                break;
        }
    }
    return false;
}
static void send_cmd(uint8_t cmd, const uint8_t* args, size_t n) {
    uint16_t len = tx.tx_packet_complete(0xFF, cmd, args, n);
    Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet), len);
    Serial1.flush();
}
static bool query_version(uint8_t ver[3]) {
    for (int retry = 0; retry < 3; retry++) {
        while (Serial1.available()) Serial1.read();
        delay(50);
        while (Serial1.available()) Serial1.read();
        send_cmd(CMD_FW_QUERY, nullptr, 0);
        uint8_t args[8], args_len = 0;
        if (wait_reply(Serial1, CMD_FW_QUERY, args, &args_len, 500) && args_len >= 3) {
            ver[0] = args[0]; ver[1] = args[1]; ver[2] = args[2];
            return true;
        }
        delay(300);
    }
    return false;
}
static bool abort_requested() { // honoured only before erase
    return host_abort || digitalRead(2) == HIGH;
}

static void start_reflash() {
    if (!ready || phase != IDLE || spent) return;
    spent = true; host_abort = false;
    // Disable ESP32 brownout detector: AT32 resets during OTA can cause supply
    // transients; an ESP32 brownout reset mid-write would leave AT32 half-flashed.
    saved_brownout = REG_READ(RTC_CNTL_BROWN_OUT_REG);
    REG_WRITE(RTC_CNTL_BROWN_OUT_REG, 0);
    brownout_saved = true;
    phase = PRE_QUERY; stage_ms = millis(); reason = "pre_query_version";
    Serial.println("[Reflash] Authorized one-shot AT32 official 1.0.2 force reflash starting");
}

void setup() {
    Serial.begin(1000000); delay(300);
    pinMode(2, INPUT_PULLDOWN);

    // Self-check 1: embedded image integrity (version macros + size + CRC32)
    if (AT32_FW_VERSION_MAJOR != 1 || AT32_FW_VERSION_MINOR != 0 || AT32_FW_VERSION_PATCH != 2 ||
        AT32_FW_SIZE != 21848 || fw_crc32() != EXPECT_FW_CRC32) {
        Serial.println("[Reflash] SELF TEST FAIL: embedded image mismatch; UART disabled");
        return;
    }
    // Self-check 2: frame builder checksum sanity (cmd94 zero-arg frame)
    {
        uint16_t len = tx.tx_packet_complete(0xFF, CMD_FW_QUERY, nullptr, 0);
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&tx.tx_packet);
        uint8_t sum = 0; for (uint16_t i = 2; i < len - 1; i++) sum += p[i];
        if (len != 6 || p[0] != 0xFF || p[1] != 0xFF || p[len-1] != (uint8_t)(~sum)) {
            Serial.println("[Reflash] SELF TEST FAIL: frame builder; UART disabled");
            return;
        }
    }
    Serial1.setRxBufferSize(2048);
    Serial1.begin(1000000, SERIAL_8N1, 17, 16);
    rx.begin();
    ready = true;
    pkt_total = (AT32_FW_SIZE + 127) / 128;
    Serial.printf("[Reflash] %s READY; embedded=1.0.2 size=%u crc32=0x%08lX packets=%lu; no automatic start\n",
                  VERSION, (unsigned)AT32_FW_SIZE, (unsigned long)EXPECT_FW_CRC32, (unsigned long)pkt_total);
    Serial.printf("[Reflash] reset_reason=%d\n", (int)esp_reset_reason());
    if (nvs_get_pend() == 1) {
        // We were reset mid-OTA. AT32 should be sitting in its bootloader
        // (its app is intact only if erase never started; either way BL_CHECK
        // below decides safely). Auto-resume after a short abort window.
        spent = true; host_abort = false;
        saved_brownout = REG_READ(RTC_CNTL_BROWN_OUT_REG);
        REG_WRITE(RTC_CNTL_BROWN_OUT_REG, 0);
        brownout_saved = true;
        phase = RESUME_WINDOW; stage_ms = millis(); reason = "resume_window_3s";
        Serial.println("[Reflash] OTA PENDING flag set: auto-resuming after 3s (KEY2/ABORT_REFLASH cancels)");
    }
    evidence();
}

void loop() {
    if (!ready) { delay(10); return; }
    for (unsigned i = 0; i < 128 && Serial.available(); ++i) {
        char c = Serial.read();
        if (c == '\n') {
            host_line[host_used] = 0;
            if (!host_overflow && !strcmp(host_line, "FORCE_REFLASH_ONCE")) start_reflash();
            if (!host_overflow && !strcmp(host_line, "ABORT_REFLASH")) host_abort = true;
            if (!host_overflow && !strcmp(host_line, "QUERY_VER") && (phase == IDLE || phase == FINISHED)) {
                uint8_t v[3] = {0,0,0}; bool got = query_version(v);
                Serial.printf("{\"type\":\"ota_query\",\"stage\":\"manual\",\"got\":%s,\"ver\":\"%d.%d.%d\"}\n",
                              got ? "true" : "false", v[0], v[1], v[2]);
            }
            if (!host_overflow && !strcmp(host_line, "QUERY65") && (phase == IDLE || phase == FINISHED)) {
                // App-presence probe: aggregated joint position feedback (cmd 65)
                // is served by the AT32 *application*, not by the OTA bootloader.
                while (Serial1.available()) Serial1.read();
                send_cmd(65, nullptr, 0);
                uint8_t args[16], args_len = 0;
                bool got = wait_reply(Serial1, 65, args, &args_len, 800);
                if (got && args_len >= 12) {
                    int16_t p[6];
                    for (int j = 0; j < 6; j++) p[j] = (int16_t)(args[2*j] | (args[2*j+1] << 8));
                    Serial.printf("{\"type\":\"app_probe\",\"cmd\":65,\"got\":true,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",
                                  p[0], p[1], p[2], p[3], p[4], p[5]);
                } else {
                    Serial.printf("{\"type\":\"app_probe\",\"cmd\":65,\"got\":%s,\"args_len\":%u}\n",
                                  got ? "true" : "false", (unsigned)args_len);
                }
            }
            host_used = 0; host_overflow = false;
        } else if (c != '\r') {
            if (host_used < sizeof(host_line) - 1 && !host_overflow) host_line[host_used++] = c;
            else host_overflow = true;
        }
    }
    uint32_t now = millis();

    switch (phase) {
    case RESUME_WINDOW:
        if (abort_requested()) { finish("resume_aborted_power_cycle_at32_to_app"); break; }
        if (now - stage_ms >= 3000) {
            uint8_t args[8], args_len = 0;
            while (Serial1.available()) Serial1.read();
            send_cmd(CMD_FW_QUERY, nullptr, 0);
            bool bl = wait_reply(Serial1, CMD_FW_QUERY, args, &args_len, 2000);
            log_step("resume_bootloader_ready", bl);
            if (!bl) { finish("resume_bootloader_lost_at32_back_in_app_retry_fresh"); break; }
            if (abort_requested()) { finish("resume_aborted_power_cycle_at32_to_app"); break; }
            phase = ERASE; reason = "erasing";
            Serial.println("[Reflash] (resumed) ERASE starting; abort is now DISABLED; keep power stable");
            while (Serial1.available()) Serial1.read();
            send_cmd(CMD_FW_START, nullptr, 0);
            stage_ms = millis();
        }
        break;
    case PRE_QUERY:
        pre_got = query_version(pre_ver);
        Serial.printf("{\"type\":\"ota_query\",\"stage\":\"pre\",\"got\":%s,\"ver\":\"%d.%d.%d\"}\n",
                      pre_got ? "true" : "false", pre_ver[0], pre_ver[1], pre_ver[2]);
        if (!pre_got) { finish("pre_query_failed_at32_silent"); break; }
        phase = CONFIRM_WINDOW; stage_ms = now; reason = "abort_window_5s";
        Serial.println("[Reflash] 5s abort window: press KEY2 or send ABORT_REFLASH to cancel; erase has NOT started");
        break;
    case CONFIRM_WINDOW:
        if (abort_requested()) { finish("aborted_before_erase"); break; }
        if (now - stage_ms >= 5000) {
            phase = ENTER_BL; reason = "enter_bootloader";
            log_step("enter_bl_cmd90", true);
            nvs_set_pend(1); // from here until finish(), an unexpected reset resumes OTA at boot
            while (Serial1.available()) Serial1.read();
            send_cmd(CMD_FIRMWARE_UPDATE, nullptr, 0);
            stage_ms = millis();
        }
        break;
    case ENTER_BL:
        if (abort_requested()) { finish("aborted_before_erase"); break; }
        if (now - stage_ms >= 3000) {
            while (Serial1.available()) Serial1.read();
            uint8_t args[8], args_len = 0;
            send_cmd(CMD_FW_QUERY, nullptr, 0);
            bool bl = wait_reply(Serial1, CMD_FW_QUERY, args, &args_len, 2000);
            log_step("bootloader_ready", bl);
            if (!bl) { finish("bootloader_not_responding"); break; }
            if (abort_requested()) { finish("aborted_before_erase"); break; }
            phase = ERASE; reason = "erasing";
            Serial.println("[Reflash] ERASE starting; abort is now DISABLED; keep power stable");
            while (Serial1.available()) Serial1.read();
            send_cmd(CMD_FW_START, nullptr, 0);
            stage_ms = millis();
        }
        break;
    case ERASE: {
        uint8_t args[8], args_len = 0;
        bool ok = wait_reply(Serial1, CMD_FW_START, args, &args_len, 10000 - (now - stage_ms > 10000 ? 10000 : now - stage_ms));
        log_step("erase", ok);
        if (!ok) { finish("erase_timeout_brick_risk_power_cycle_and_retry"); break; }
        phase = WRITE; pkt_i = 0; reason = "writing"; stage_ms = millis();
        break; }
    case WRITE:
        if (pkt_i < pkt_total && now - stage_ms >= 15) {
            uint32_t offset = pkt_i * 128;
            uint16_t chunk = (offset + 128 > AT32_FW_SIZE) ? (AT32_FW_SIZE - offset) : 128;
            uint8_t padded = chunk; if (padded % 4) padded += 4 - (padded % 4);
            uint8_t pkt[2 + 132];
            pkt[0] = pkt_i & 0xFF; pkt[1] = (pkt_i >> 8) & 0xFF;
            memcpy(&pkt[2], &at32_firmware[offset], chunk);
            for (uint8_t j = chunk; j < padded; j++) pkt[2 + j] = 0xFF;
            send_cmd(CMD_FW_DATA, pkt, 2 + padded);
            while (Serial1.available()) Serial1.read();
            pkt_i++; stage_ms = now;
            if (pkt_i % 20 == 0 || pkt_i == pkt_total)
                Serial.printf("{\"type\":\"ota_progress\",\"pkt\":%lu,\"total\":%lu,\"pct\":%lu}\n",
                              (unsigned long)pkt_i, (unsigned long)pkt_total,
                              (unsigned long)(pkt_i * 100 / pkt_total));
        }
        if (pkt_i >= pkt_total) {
            log_step("write_all_packets", true);
            phase = END_FW; reason = "finalizing";
            while (Serial1.available()) Serial1.read();
            send_cmd(CMD_FW_END, nullptr, 0);
            stage_ms = millis();
        }
        break;
    case END_FW:
        if (now - stage_ms >= 3000) {
            while (Serial1.available()) Serial1.read();
            phase = VERIFY; reason = "verifying";
        }
        break;
    case VERIFY:
        post_got = query_version(post_ver);
        Serial.printf("{\"type\":\"ota_query\",\"stage\":\"post\",\"got\":%s,\"ver\":\"%d.%d.%d\"}\n",
                      post_got ? "true" : "false", post_ver[0], post_ver[1], post_ver[2]);
        ota_ok = post_got && post_ver[0] == 1 && post_ver[1] == 0 && post_ver[2] == 2;
        log_step("verify_version_1_0_2", ota_ok);
        finish(ota_ok ? "reflash_success_power_cycle_required" : "verify_failed_inspect_log");
        break;
    default: break;
    }

    if (now - last_report >= 1000 && phase != IDLE) { evidence(); last_report = now; }
    delay(1);
}
