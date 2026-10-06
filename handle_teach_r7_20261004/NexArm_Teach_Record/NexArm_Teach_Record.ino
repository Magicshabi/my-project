#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "RealJointFeedback.h"
#include "TeachCapture.h"
#include "TeachKeys.h"
#include "ProtocolSelfTest.h"

// Recording firmware: AT32 output is restricted to parameterless CMD65.
// No torque writes (including OFF), targets, homing, OTA, or host forwarding.
// Use only with an already freely movable, supported arm. AT32's own startup
// behavior and servo power/torque cannot be established by position replies.
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* const VERSION = "nexarm-teach-record-r7";
static const char* const NVS_NAMESPACE = "handle_free";
static const char* const NVS_KEY = "poses_v1";
static const char* const STAGES[8] = {
    "Red above OPEN", "Red down OPEN", "Red down GRIP", "Red lift GRIP",
    "T0 above GRIP", "T0 down GRIP", "T0 down OPEN", "T0 clear OPEN"
};
static CommProtocol_t at32_rx, at32_tx, host_rx, host_tx;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE, 27, 26);
static HandleTeachRecord record = new_handle_record();
static TeachCapture capture;
static TeachKeys keys;
enum CaptureState { IDLE, SETTLING, SAMPLING };
static CaptureState capture_state = IDLE;
static uint8_t slot = 0;
static bool self_test_passed = false, oled_found = false, storage_fault = false;
static bool replace_confirm = false, awaiting = false, capture_request = false;
static uint32_t started_ms = 0, requested_ms = 0, last_query_ms = 0;
static uint32_t last_oled_ms = 0, last_notice_ms = 0, actual_count = 0;
static int16_t actual[6] = {};
static const char* message = "K1:SAVE K2:NEXT";
static uint8_t observed_key_mask = 0;
static uint8_t logged_key_mask = 255;
static uint32_t last_key_log_ms = 0, last_release_sequence = 0;
static ProtocolStats request_rx_start;
static uint32_t last_at32_byte_ms = 0, last_host_byte_ms = 0;
static uint32_t query_timeouts = 0, last_timeout_log_ms = 0, last_rx_log_ms = 0;
static uint32_t reported_rx_errors = 0;

static void report_rx_health() {
    const ProtocolStats& s = at32_rx.stats;
    Serial.printf("{\"type\":\"rx_health\",\"ms\":%lu,\"bytes\":%lu,\"valid_frames\":%lu,\"errors\":%lu,\"length_errors\":%lu,\"checksum_errors\":%lu,\"partial_timeouts\":%lu,\"noise_bytes\":%lu,\"consecutive_errors\":%lu,\"last_error\":%u,\"query_timeouts\":%lu}\n",
        static_cast<unsigned long>(millis()), static_cast<unsigned long>(s.bytes),
        static_cast<unsigned long>(s.valid_frames), static_cast<unsigned long>(s.errors),
        static_cast<unsigned long>(s.length_errors), static_cast<unsigned long>(s.checksum_errors),
        static_cast<unsigned long>(s.partial_timeouts), static_cast<unsigned long>(s.noise_bytes),
        static_cast<unsigned long>(s.consecutive_errors), static_cast<unsigned>(at32_rx.error_state),
        static_cast<unsigned long>(query_timeouts));
    reported_rx_errors = s.errors;
    last_rx_log_ms = millis();
}

static void report_query_timeout(uint32_t now) {
    ++query_timeouts;
    if (query_timeouts != 1 && now - last_timeout_log_ms < 2000) return;
    const ProtocolStats& s = at32_rx.stats;
    uint32_t bytes = s.bytes - request_rx_start.bytes;
    uint32_t good = s.valid_frames - request_rx_start.valid_frames;
    uint32_t bad = s.errors - request_rx_start.errors;
    const char* kind = !bytes ? "no_rx_bytes" : good ? "no_matching_cmd65" :
                       bad ? "rx_parse_errors" : "partial_or_noise";
    Serial.printf("{\"type\":\"query_timeout\",\"ms\":%lu,\"kind\":\"%s\",\"rx_bytes\":%lu,\"valid_frames\":%lu,\"parse_errors\":%lu}\n",
        static_cast<unsigned long>(now), kind, static_cast<unsigned long>(bytes),
        static_cast<unsigned long>(good), static_cast<unsigned long>(bad));
    last_timeout_log_ms = now;
}

static void show_oled() {
    if (!oled_found) return;
    char title[24];
    snprintf(title, sizeof(title), "R7 NO LOCK %u/8", slot < 8 ? slot + 1 : 8);
    char key_line[24];
    snprintf(key_line, sizeof(key_line), "K1:%s K2:%s",
             (observed_key_mask & 1) ? "DOWN" : "UP",
             (observed_key_mask & 2) ? "DOWN" : "UP");
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_6x12_tr);
        oled.drawStr(0, 13, title);
        oled.drawStr(0, 29, slot < 8 ? STAGES[slot] : "8 saved; NO RUN");
        oled.drawStr(0, 45, message);
        oled.drawStr(0, 61, key_line);
    } while (oled.nextPage());
}

static void report_record() {
    char line[RECORD_JSON_SIZE];
    size_t length = format_record_json(record, VERSION, NVS_NAMESPACE, storage_fault, line, sizeof(line));
    if (length) Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    else Serial.println("[TeachRecord] JSON buffer error; snapshot not emitted");
}

static bool load_record() {
    Preferences storage;
    if (!storage.begin(NVS_NAMESPACE, false)) return false;
    if (!storage.isKey(NVS_KEY)) { storage.end(); return true; }
    HandleTeachRecord loaded = {};
    bool ok = storage.getBytesLength(NVS_KEY) == sizeof(loaded) &&
        storage.getBytes(NVS_KEY, &loaded, sizeof(loaded)) == sizeof(loaded) &&
        valid_handle_record(loaded);
    storage.end();
    if (ok) record = loaded;
    return ok;
}

static bool persist_record(const HandleTeachRecord& candidate) {
    if (!valid_handle_record(candidate)) return false;
    Preferences storage;
    if (!storage.begin(NVS_NAMESPACE, false)) return false;
    bool ok = storage.putBytes(NVS_KEY, &candidate, sizeof(candidate)) == sizeof(candidate);
    storage.end();
    if (!ok || !storage.begin(NVS_NAMESPACE, true)) return false;
    HandleTeachRecord readback = {};
    ok = storage.getBytesLength(NVS_KEY) == sizeof(readback) &&
        storage.getBytes(NVS_KEY, &readback, sizeof(readback)) == sizeof(readback) &&
        valid_handle_record(readback) && memcmp(&candidate, &readback, sizeof(candidate)) == 0;
    storage.end();
    return ok;
}

static void fail_capture(const char* reason) {
    capture_state = IDLE;
    capture_request = false;
    message = reason;
    Serial.printf("[TeachRecord] NOT SAVED: %s; no motor command sent\n", reason);
}

static void query_actual(bool for_capture) {
    if (!self_test_passed || awaiting) return;
    // Sole AT32 transmit site: constant read command, zero arguments.
    uint16_t length = at32_tx.tx_packet_complete(0xFF, 65, nullptr, 0);
    awaiting = true;
    capture_request = for_capture;
    requested_ms = last_query_ms = millis();
    request_rx_start = at32_rx.stats;
    Serial1.write(reinterpret_cast<const uint8_t*>(&at32_tx.tx_packet), length);
}

static void on_at32(PacketTypeDef* packet) {
    if (packet->elements.id != 0x5A || packet->elements.cmd != 65) return;
    if (packet->elements.length != 26 ||
        !decode_real_joint_positions(packet->elements.args, 24, actual)) {
        if (awaiting && capture_request) fail_capture("Bad feedback: retry");
        awaiting = false;
        return;
    }
    ++actual_count;
    Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",
        static_cast<unsigned long>(millis()), actual[0], actual[1], actual[2], actual[3], actual[4], actual[5]);
    if (awaiting && capture_request && capture_state == SAMPLING) {
        if (millis() - requested_ms > 400) fail_capture("Feedback timeout");
        else if (!capture.add(actual)) fail_capture("Unstable: retry K1");
    }
    awaiting = false;
    capture_request = false;
}

static void on_host(PacketTypeDef* packet) {
    // Compatibility with the existing read_handle_teach.py export tool.
    // All other host commands, including servo/coordinate/torque/OTA, ignored.
    if (!self_test_passed || storage_fault || packet->elements.id != 0xFF ||
        packet->elements.cmd != 105 || packet->elements.length != 2) return;
    uint16_t length = host_tx.tx_packet_complete(0xFF, 105,
        reinterpret_cast<uint8_t*>(&record), sizeof(record));
    Serial.write(reinterpret_cast<const uint8_t*>(&host_tx.tx_packet), length);
}

static void start_capture() {
    capture.reset();
    capture_state = SETTLING;
    started_ms = millis();
    capture_request = false;
    replace_confirm = false;
    message = "Hold still: reading";
    Serial.printf("[TeachRecord] Capture slot=%u; settle 600ms, then 5 CMD65 replies\n", slot);
}

static void save_capture() {
    int16_t captured[6];
    HandleTeachRecord candidate;
    if (!capture.result(captured) || !make_teach_candidate(record, slot, captured, candidate)) {
        fail_capture("Invalid: retry K1");
        return;
    }
    if (!persist_record(candidate)) {
        // A write may have committed even if its verification failed. Require
        // a restart/readback instead of claiming failure left flash unchanged.
        storage_fault = true;
        fail_capture("NVS error: restart");
        report_record();
        return;
    }
    record = candidate;
    capture_state = IDLE;
    message = "Saved; K2: next";
    Serial.printf("[TeachRecord] SAVED slot=%u joints=%d,%d,%d,%d,%d,%d; NO LOCK; support arm\n",
        slot, captured[0], captured[1], captured[2], captured[3], captured[4], captured[5]);
    report_record();
}

static void on_key(uint8_t key) {
    if (key) Serial.printf("{\"type\":\"key_click\",\"key\":%u,\"ms\":%lu,\"slot\":%u,\"capture_state\":%u,\"storage_fault\":%s}\n",
        key, static_cast<unsigned long>(millis()), slot, static_cast<unsigned>(capture_state),
        storage_fault ? "true" : "false");
    if (!key || !self_test_passed || storage_fault || capture_state != IDLE) return;
    if (replace_confirm) {
        replace_confirm = false;
        if (key == 1) start_capture();
        else message = "Kept; K2: next";
        return;
    }
    if (slot == 8) {
        if (key == 1) message = "Full; K2: review";
        if (key == 2) { slot = 0; message = "K1:redo K2:review"; }
        return;
    }
    if (key == 1) {
        if (record.valid_mask & (1u << slot)) {
            replace_confirm = true;
            message = "Redo? K1:Y K2:N";
            // The next short K1 confirms, K2 cancels; no flash write yet.
            Serial.println("[TeachRecord] Replace this and clear later points? K1 yes, K2 no");
        } else start_capture();
    } else if (key == 2) {
        if (!(record.valid_mask & (1u << slot))) { message = "Save first: K1"; return; }
        ++slot;
        message = slot == 8 ? "K2:review NO RUN" : "K1:SAVE K2:NEXT";
    }
}

void setup() {
    Serial.begin(1000000);
    delay(300);
    pinMode(0, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    Wire.setPins(26, 27);
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(0x3C);
    oled_found = Wire.endTransmission() == 0;
    if (oled_found) oled.begin();
    Serial.printf("[OLED] %s at 0x3C\n", oled_found ? "FOUND" : "MISSING");
    self_test_passed = real_joint_feedback_self_test() && handle_record_self_test() &&
                       teach_capture_self_test() && teach_keys_self_test() && protocol_boundary_self_test();
    if (!self_test_passed) {
        message = "SELF TEST FAILED";
        show_oled();
        Serial.println("[TeachRecordTest] FAIL; AT32 UART not initialized");
        return;
    }
    Serial.println("[TeachRecordTest] PASS: decode, CRC, capture, stage order, keys");
    Serial.println("[ProtocolBoundaryTest] PASS: 0/24/108/248/249/250 bytes, bad length/checksum, partial recovery, JSON bounds");
    storage_fault = !load_record();
    slot = first_missing_slot(record);
    if (storage_fault) message = "NVS error: restart";
    else if (slot == 8) message = "K2:review NO RUN";
    at32_rx.begin(); at32_tx.begin(); host_rx.begin(); host_tx.begin();
    at32_rx.register_success_callback(on_at32);
    host_rx.register_success_callback(on_host);
    Serial1.setRxBufferSize(2048);
    Serial1.begin(1000000, SERIAL_8N1, 17, 16);
    Serial.printf("[TeachRecord] %s READY; AT32 CMD65 only; K1 saves, never locks\n", VERSION);
    Serial.println("[TeachRecord] Servo power/torque unverified. Support arm; never force a stiff joint.");
    report_record();
    show_oled();
}

void loop() {
    if (!self_test_passed) { delay(100); return; }
    // Bound work per loop so corrupt input cannot starve keys or timeouts.
    for (uint16_t n = 0; n < 2048 && Serial1.available(); ++n) {
        uint32_t received_ms = millis();
        if (at32_rx.has_partial_frame() && received_ms - last_at32_byte_ms > 100)
            at32_rx.discard_partial_frame();
        last_at32_byte_ms = received_ms;
        uint8_t byte = Serial1.read(); at32_rx.parsing(&byte, 1);
    }
    for (uint16_t n = 0; n < 256 && Serial.available(); ++n) {
        uint32_t received_ms = millis();
        if (host_rx.has_partial_frame() && received_ms - last_host_byte_ms > 100)
            host_rx.discard_partial_frame();
        last_host_byte_ms = received_ms;
        uint8_t byte = Serial.read(); host_rx.parsing(&byte, 1);
    }
    uint32_t now = millis();
    if (at32_rx.has_partial_frame() && now - last_at32_byte_ms > 100) at32_rx.discard_partial_frame();
    if (host_rx.has_partial_frame() && now - last_host_byte_ms > 100) host_rx.discard_partial_frame();
    uint8_t mask = (digitalRead(0) == LOW ? 1 : 0) | (digitalRead(2) == LOW ? 2 : 0);
    observed_key_mask = mask;
    uint8_t click = keys.update(mask, now);
    if ((mask != logged_key_mask && now - last_key_log_ms >= 50) || now - last_key_log_ms >= 1000) {
        Serial.printf("{\"type\":\"key_pins\",\"ms\":%lu,\"pressed_mask\":%u,\"gpio0\":%u,\"gpio2\":%u,\"armed\":%s,\"stable_mask\":%u,\"pending_mask\":%u,\"blocked\":%s}\n",
            static_cast<unsigned long>(now), mask, (mask & 1) ? 0 : 1, (mask & 2) ? 0 : 1,
            keys.is_armed() ? "true" : "false", keys.stable_mask(), keys.pending_mask(),
            keys.is_blocked() ? "true" : "false");
        logged_key_mask = mask;
        last_key_log_ms = now;
    }
    if (keys.release_sequence != last_release_sequence) {
        Serial.printf("{\"type\":\"key_release\",\"ms\":%lu,\"key\":%u,\"duration_ms\":%lu,\"reason\":%u,\"click\":%u}\n",
            static_cast<unsigned long>(now), keys.released_key,
            static_cast<unsigned long>(keys.release_duration_ms), keys.release_reason, click);
        last_release_sequence = keys.release_sequence;
    }
    on_key(click);
    // on_key can start a capture using a later millis() value. Refresh before
    // unsigned elapsed-time comparisons (otherwise the same loop can time out).
    now = millis();
    if (awaiting && now - requested_ms > 400) {
        report_query_timeout(now);
        if (capture_request) fail_capture("No CMD65: retry K1");
        awaiting = false;
    }
    if (capture_state != IDLE && now - started_ms > 4000) fail_capture("Timeout: retry K1");
    if (capture_state == SETTLING && now - started_ms >= 600 && !awaiting)
        capture_state = SAMPLING;
    if (capture_state == SAMPLING && capture.count == 5) save_capture();
    if (!awaiting && capture_state == SAMPLING && now - last_query_ms >= 120) query_actual(true);
    else if (!awaiting && capture_state == IDLE && now - last_query_ms >= 250) query_actual(false);
    if (now - last_oled_ms >= 200) { show_oled(); last_oled_ms = now; }
    if (now - last_notice_ms >= 5000) { report_record(); report_rx_health(); last_notice_ms = now; }
    else if (at32_rx.stats.errors != reported_rx_errors && now - last_rx_log_ms >= 2000) report_rx_health();
    delay(1);
}
