#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "CommProtocol.h"
#include "RealJointFeedback.h"

// ESP32 diagnostic only. No torque, goal, register-write, OTA, homing, NVS,
// gripper or motor control code. AT32's own behavior is outside this sketch.
// Verified local SerialServo_t pin mapping: RX=17, TX=16, UART1=1 Mbps.
static CommProtocol_t rx_protocol;
static CommProtocol_t tx_protocol;
static uint32_t actual_count = 0;
static uint32_t status_count = 0;
static uint32_t last_query_ms = 0;
static uint32_t last_notice_ms = 0;
static uint32_t last_oled_ms = 0;
static bool query_actual_next = true;
static bool self_test_passed = false;
static bool oled_found = false;
static bool actual_seen = false;
static int16_t id4_actual = 0;
// Factory NexArm wiring: SDA=26, SCL=27, SSD1306 I2C address 0x3C.
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE, 27, 26);

static void show_oled() {
    if (!oled_found) return;
    char line[25];
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_7x14B_tr);
        oled.drawStr(0, 14, "READ ONLY  ID4");
        oled.setFont(u8g2_font_7x14_tr);
        if (actual_seen) snprintf(line, sizeof(line), "Actual: %d", id4_actual);
        else snprintf(line, sizeof(line), "Actual: waiting");
        oled.drawStr(0, 30, line);
        snprintf(line, sizeof(line), "Rx65: %lu", static_cast<unsigned long>(actual_count));
        oled.drawStr(0, 46, line);
        oled.drawStr(0, 62, "KEY1 disabled");
    } while (oled.nextPage());
}

static int16_t read_i16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

static bool read_query_allowed(uint8_t command) {
    return command == 11 || command == 65;
}

static void query(uint8_t command) {
    // The only transmit site is guarded by this explicit read-only whitelist.
    if (!self_test_passed || !read_query_allowed(command)) return;
    uint8_t length = tx_protocol.tx_packet_complete(0xFF, command, nullptr, 0);
    Serial1.write(reinterpret_cast<const uint8_t*>(&tx_protocol.tx_packet), length);
}

static void on_response(PacketTypeDef* packet) {
    if (packet->elements.id != 0x5A) return;
    const uint8_t command = packet->elements.cmd;
    const uint8_t* data = packet->elements.args;
    if (!read_query_allowed(command)) return;
    if (packet->elements.length != 26) {
        Serial.printf("{\"type\":\"unexpected_length\",\"cmd\":%u,\"length\":%u}\n",
                      command, packet->elements.length);
        return;
    }
    int16_t pulses[6];
    if (command == 65) {
        if (!decode_real_joint_positions(data, 24, pulses)) {
            Serial.println("{\"type\":\"invalid_actual_positions\"}");
            return;
        }
        ++actual_count;
        id4_actual = pulses[3];
        actual_seen = true;
    } else {
        for (uint8_t i = 0; i < 6; ++i) pulses[i] = read_i16(data + 12 + 2*i);
        ++status_count;
    }
    Serial.printf("{\"type\":\"%s\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",
        command == 65 ? "actual65" : "status11", static_cast<unsigned long>(millis()),
        pulses[0], pulses[1], pulses[2], pulses[3], pulses[4], pulses[5]);
}

void setup() {
    Serial.begin(1000000);
    delay(300);
    Wire.setPins(26, 27);
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(0x3C);
    oled_found = Wire.endTransmission() == 0;
    if (oled_found) {
        oled.begin();
        show_oled();
        Serial.println("[OLED] Found address 0x3C; read-only status displayed");
    } else {
        Serial.println("[OLED] No I2C acknowledgement at 0x3C");
    }
    self_test_passed = real_joint_feedback_self_test();
    if (!self_test_passed) {
        Serial.println("[ReadOnlyTest] FAIL; AT32 UART not initialized");
        return;
    }
    Serial.println("[ReadOnlyTest] PASS: actual-position decoder");
    rx_protocol.begin();
    tx_protocol.begin();
    rx_protocol.register_success_callback(on_response);
    Serial1.setRxBufferSize(2048);
    Serial1.begin(1000000, SERIAL_8N1, 17, 16);
    last_query_ms = millis();
    Serial.println("[ReadOnly] nexarm-readonly-oled-r3 READY; only queries 11 and 65; no torque/target writes");
}

void loop() {
    if (!self_test_passed) { delay(100); return; }
    while (Serial1.available()) {
        uint8_t value = Serial1.read();
        rx_protocol.parsing(&value, 1);
    }
    // Never forward incoming USB bytes to AT32.
    while (Serial.available()) Serial.read();
    uint32_t now = millis();
    if (now - last_query_ms >= 500) {
        query(query_actual_next ? 65 : 11);
        query_actual_next = !query_actual_next;
        last_query_ms = now;
    }
    if (now - last_notice_ms >= 5000) {
        Serial.printf("[ReadOnly] actual_frames=%lu status_frames=%lu; alarm code unavailable\n",
            static_cast<unsigned long>(actual_count), static_cast<unsigned long>(status_count));
        last_notice_ms = now;
    }
    if (now - last_oled_ms >= 1000) {
        show_oled();
        last_oled_ms = now;
    }
    delay(1);
}
