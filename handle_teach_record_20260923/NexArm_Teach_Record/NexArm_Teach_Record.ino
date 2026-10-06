#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "RealJointFeedback.h"
#include "TeachCapture.h"
#include "TeachKeys.h"

// Recording firmware: AT32 output is restricted to parameterless CMD65.
// No torque writes (including OFF), targets, homing, OTA, or host forwarding.
// Use only with an already freely movable, supported arm. AT32's own startup
// behavior and servo power/torque cannot be established by position replies.
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* const VERSION = "nexarm-teach-record-r6";
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

static void show_oled() {
    if (!oled_found) return;
    char title[24];
    snprintf(title, sizeof(title), "RECORD R6  %u/8", slot < 8 ? slot + 1 : 8);
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_6x12_tr);
        oled.drawStr(0, 13, title);
        oled.drawStr(0, 29, slot < 8 ? STAGES[slot] : "8 saved; NO RUN");
        oled.drawStr(0, 45, message);
        oled.drawStr(0, 61, "NO LOCK; SUPPORT ARM");
    } while (oled.nextPage());
}

static void report_record() {
    Serial.printf("{\"type\":\"teach_record\",\"firmware\":\"%s\",\"namespace\":\"%s\",\"valid_mask\":%u,\"storage_fault\":%s,\"record_hex\":\"",
                  VERSION, NVS_NAMESPACE, record.valid_mask, storage_fault ? "true" : "false");
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
    for (size_t i = 0; i < sizeof(record); ++i) Serial.printf("%02x", bytes[i]);
    Serial.println("\",\"playback_enabled\":false}");
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
    uint8_t length = at32_tx.tx_packet_complete(0xFF, 65, nullptr, 0);
    awaiting = true;
    capture_request = for_capture;
    requested_ms = last_query_ms = millis();
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
    uint8_t length = host_tx.tx_packet_complete(0xFF, 105,
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
    if (!key || !self_test_passed || storage_fault || capture_state != IDLE) return;
    if (replace_confirm) {
        replace_confirm = false;
        if (key == 1) start_capture();
        else message = "Kept; K2: next";
        return;
    }
    if (slot == 8) {
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
                       teach_capture_self_test() && teach_keys_self_test();
    if (!self_test_passed) {
        message = "SELF TEST FAILED";
        show_oled();
        Serial.println("[TeachRecordTest] FAIL; AT32 UART not initialized");
        return;
    }
    Serial.println("[TeachRecordTest] PASS: decode, CRC, capture, stage order, keys");
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
        uint8_t byte = Serial1.read(); at32_rx.parsing(&byte, 1);
    }
    for (uint16_t n = 0; n < 256 && Serial.available(); ++n) {
        uint8_t byte = Serial.read(); host_rx.parsing(&byte, 1);
    }
    uint32_t now = millis();
    uint8_t mask = (digitalRead(0) == LOW ? 1 : 0) | (digitalRead(2) == LOW ? 2 : 0);
    on_key(keys.update(mask, now));
    // on_key can start a capture using a later millis() value. Refresh before
    // unsigned elapsed-time comparisons (otherwise the same loop can time out).
    now = millis();
    if (awaiting && now - requested_ms > 400) {
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
    if (now - last_notice_ms >= 5000) { report_record(); last_notice_ms = now; }
    delay(1);
}
