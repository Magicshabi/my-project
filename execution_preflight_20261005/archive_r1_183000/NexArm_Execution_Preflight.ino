#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "RealJointFeedback.h"
#include "ProtocolSelfTest.h"
#include "TeachKeys.h"
#include "ProbePolicy.h"

// No motion, torque, mode, reset, ID, calibration, NVS write or host forwarding.
// UART transmission passes a fixed read-only whitelist at its only write site.
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* VERSION = "nexarm-execution-preflight-r1";
static CommProtocol_t rx, tx;
static TeachKeys keys;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE, 27, 26);
static HandleTeachRecord record;
static bool ready = false, oled_found = false, running = false, waiting = false;
static bool automatic_probe_done = false;
static uint32_t last_query = 0, last_rx = 0, last_display = 0, last_record = 0;
static uint32_t sent_ms = 0, next_read_ms = 0, actual_ms = 0, actual_seq = 0;
static int16_t actual[6] = {};
static uint8_t probe_round = 0, probe_slot = 0, observed_mask = 0, other_count = 0;
static uint8_t replies[2][4][8] = {};
static bool got[2][4] = {};
static const char* message = "Waiting to check";

static bool transmit(uint8_t id, uint8_t cmd, const uint8_t* args, size_t n) {
    if (!allowed_probe_tx(id, cmd, args, n)) {
        Serial.println("[Preflight] BLOCKED TX outside read-only whitelist");
        ready = false; message = "TX blocked"; return false;
    }
    uint16_t len = tx.tx_packet_complete(id, cmd, args, n);
    if (!len) return false;
    return Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet), len) == len;
}
static void report_record() {
    char line[RECORD_JSON_SIZE];
    size_t n = format_record_json(record, VERSION, "handle_free", false, line, sizeof(line));
    if (n) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
}
static void display() {
    if (!oled_found) return;
    char state[24];
    snprintf(state, sizeof(state), "K1:%s K2:%s", observed_mask & 1 ? "DOWN" : "UP", observed_mask & 2 ? "DOWN" : "UP");
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_6x12_tr);
        oled.drawStr(0, 13, "EXEC CHECK ID6");
        oled.drawStr(0, 29, "READ ONLY / NO MOVE");
        oled.drawStr(0, 45, message);
        oled.drawStr(0, 61, state);
    } while (oled.nextPage());
}
static uint16_t u16(const uint8_t* p) { return p[0] | static_cast<uint16_t>(p[1]) << 8; }
static void finish_probe() {
    running = waiting = false;
    bool complete = true, identity = true, consistent = true, plausible = true;
    for (uint8_t r = 0; r < 2; ++r) {
        for (uint8_t s = 0; s < 4; ++s) complete &= got[r][s];
        identity &= got[r][0] && replies[r][0][2] == PROBE_ID;
        plausible &= got[r][1] && replies[r][1][0] == 0 && got[r][2] &&
            replies[r][2][0] <= 1 && u16(replies[r][2] + 2) <= 4095 &&
            got[r][3] && u16(replies[r][3]) <= 4095;
    }
    for (uint8_t s = 0; s < 4; ++s) {
        if (s == 3) consistent &= abs(static_cast<int>(u16(replies[0][3])) - static_cast<int>(u16(replies[1][3]))) <= 8;
        else consistent &= memcmp(replies[0][s], replies[1][s], READ_SIZES[s]) == 0;
    }
    // This checks read-path plausibility only. It never authorizes writes.
    bool agrees = actual_seq && millis() - actual_ms <= 4000 && got[1][3] &&
        abs(static_cast<int>(u16(replies[1][3])) - actual[5]) <= 8;
    bool reads_ok = complete && identity && consistent && plausible && agrees;
    message = reads_ok ? "Read OK; NO RUN" : "Read unverified";
    Serial.printf("{\"type\":\"probe_result\",\"read_path_candidate\":%s,\"complete\":%s,\"identity\":%s,\"consistent\":%s,\"plausible\":%s,\"actual65_agrees\":%s,\"target_write_verified\":false,\"motion_enabled\":false}\n",
        reads_ok ? "true" : "false", complete ? "true" : "false", identity ? "true" : "false",
        consistent ? "true" : "false", plausible ? "true" : "false", agrees ? "true" : "false");
}
static void advance_read() {
    waiting = false;
    if (++probe_slot == 4) {
        probe_slot = 0;
        if (++probe_round == 2) { finish_probe(); return; }
    }
    next_read_ms = millis() + 150;
}
static void receive(PacketTypeDef* p) {
    const uint8_t id = p->elements.id, cmd = p->elements.cmd;
    const uint16_t n = p->elements.length - 2;
    if (id == 255 && cmd == 65 && decode_real_joint_positions(p->elements.args, n, actual)) {
        actual_ms = millis(); ++actual_seq;
        Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",
            static_cast<unsigned long>(actual_ms), actual[0], actual[1], actual[2], actual[3], actual[4], actual[5]);
        return;
    }
    if (waiting && id == PROBE_ID && cmd == 0 && n == READ_SIZES[probe_slot]) {
        memcpy(replies[probe_round][probe_slot], p->elements.args, n);
        got[probe_round][probe_slot] = true;
        Serial.printf("{\"type\":\"register_read\",\"round\":%u,\"id\":6,\"requested_register\":%u,\"data\":[", probe_round, READ_REGS[probe_slot]);
        for (uint16_t i = 0; i < n; ++i) Serial.printf("%s%u", i ? "," : "", p->elements.args[i]);
        Serial.println("]}");
        advance_read();
    } else if (other_count++ < 24) {
        Serial.printf("{\"type\":\"other_frame\",\"id\":%u,\"command_or_status\":%u,\"payload_size\":%u}\n", id, cmd, n);
    }
}
static void start_probe() {
    if (running || !ready) return;
    memset(replies, 0, sizeof(replies)); memset(got, 0, sizeof(got));
    probe_round = probe_slot = other_count = 0;
    running = true; waiting = false; next_read_ms = millis() + 250;
    message = "Checking ID6 reads";
    Serial.println("[Preflight] Starting two read-only rounds; no write/torque commands exist");
}
void setup() {
    Serial.begin(1000000); delay(300);
    pinMode(0, INPUT_PULLUP); pinMode(2, INPUT_PULLDOWN);
    Wire.setPins(26, 27); Wire.begin(); Wire.setClock(400000);
    Wire.beginTransmission(0x3C); oled_found = Wire.endTransmission() == 0;
    if (oled_found) oled.begin();
    Serial.printf("[OLED] %s at 0x3C\n", oled_found ? "FOUND" : "MISSING");
    bool tests = probe_policy_self_test() && protocol_boundary_self_test() &&
        handle_record_self_test() && real_joint_feedback_self_test() && teach_pin_levels_self_test();
    if (!tests) { message = "SELF TEST FAILED"; Serial.println("[PreflightTest] FAIL; UART disabled"); display(); return; }
    Serial.println("[PreflightTest] PASS: exhaustive TX whitelist, frame boundaries, records, actual65, keys");
    Preferences storage;
    bool stored = storage.begin("handle_free", true);
    if (stored) {
        stored = storage.getBytesLength("poses_v1") == sizeof(record) &&
            storage.getBytes("poses_v1", &record, sizeof(record)) == sizeof(record) &&
            valid_handle_record(record) && record.valid_mask == 255;
        storage.end();
    }
    if (!stored) { message = "Need 8 valid points"; Serial.println("[Preflight] Record missing/invalid; UART disabled"); display(); return; }
    rx.begin(); tx.begin(); rx.register_success_callback(receive);
    Serial1.setRxBufferSize(2048); Serial1.begin(1000000, SERIAL_8N1, 17, 16);
    ready = true;
    Serial.printf("[Preflight] %s READY; ID6 read-only; NVS read-only\n", VERSION);
    report_record(); display();
}
void loop() {
    if (!ready) { delay(10); return; }
    for (unsigned i = 0; i < 2048 && Serial1.available(); ++i) {
        uint32_t now = millis();
        if (rx.has_partial_frame() && now - last_rx > 100) rx.discard_partial_frame();
        last_rx = now; uint8_t b = Serial1.read(); rx.parsing(&b, 1);
    }
    uint32_t now = millis();
    if (rx.has_partial_frame() && now - last_rx > 100) rx.discard_partial_frame();
    // Host input is discarded, never parsed as or forwarded to a servo command.
    for (unsigned i = 0; i < 256 && Serial.available(); ++i) Serial.read();
    observed_mask = teach_pressed_mask(digitalRead(0), digitalRead(2));
    uint8_t key = keys.update(observed_mask, now);
    if (key == 1) start_probe();
    if (key == 2) { running = waiting = false; automatic_probe_done = true; message = "Stopped; NO MOVE"; }
    if (!automatic_probe_done && now > 4000) { automatic_probe_done = true; start_probe(); }
    now = millis();
    if (waiting && now - sent_ms >= 650) {
        Serial.printf("{\"type\":\"register_timeout\",\"round\":%u,\"id\":6,\"requested_register\":%u,\"rx_bytes\":%lu,\"parse_errors\":%lu}\n",
            probe_round, READ_REGS[probe_slot], static_cast<unsigned long>(rx.stats.bytes), static_cast<unsigned long>(rx.stats.errors));
        advance_read();
    }
    now = millis();
    if (running && !waiting && static_cast<int32_t>(now - next_read_ms) >= 0) {
        const uint8_t args[] = {READ_REGS[probe_slot], READ_SIZES[probe_slot]};
        waiting = true; sent_ms = now;
        if (!transmit(PROBE_ID, 2, args, sizeof(args))) { waiting = running = false; message = "TX failed"; }
    }
    // Do not interleave actual65 requests with a raw register transaction.
    if (!waiting && now - last_query >= 250) { transmit(255, 65, nullptr, 0); last_query = now; }
    if (now - last_display >= 200) { display(); last_display = now; }
    if (now - last_record >= 5000) { report_record(); last_record = now; }
    delay(1);
}
