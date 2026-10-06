#include "GestureTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "EspNow_Ctrl.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14
#define APP_EMPTY                0

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_STR           0x73
#define RPT_DETECT_HAND_KP       0x78

#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define HB_RUN                   0x01
#define HB_READY                 0x04

#define I2C_POLL_INTERVAL        10

static const uint8_t GT_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

GestureTracker gestureTracker;

// ── I2C low-level ──

void GestureTracker::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
    const uint16_t CHUNK = 30;
    uint16_t off = 0;
    while (off < len) {
        uint16_t n = min((uint16_t)CHUNK, (uint16_t)(len - off));
        Wire1.beginTransmission(K230_ADDR);
        Wire1.write((uint8_t)((memAddr + off) >> 8));
        Wire1.write((uint8_t)((memAddr + off) & 0xFF));
        Wire1.write(data + off, n);
        Wire1.endTransmission();
        off += n;
    }
}

void GestureTracker::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
    const uint16_t CHUNK = 32;
    uint16_t off = 0;
    while (off < len) {
        uint16_t n = min((uint16_t)CHUNK, (uint16_t)(len - off));
        Wire1.beginTransmission(K230_ADDR);
        Wire1.write((uint8_t)((memAddr + off) >> 8));
        Wire1.write((uint8_t)((memAddr + off) & 0xFF));
        Wire1.endTransmission(false);
        Wire1.requestFrom((uint16_t)K230_ADDR, (uint8_t)n);
        for (uint16_t i = 0; i < n && Wire1.available(); i++) {
            data[off + i] = Wire1.read();
        }
        off += n;
    }
}

// ── Mailbox v2 slot operations ──

GestureTracker::SlotMetaV2 GestureTracker::readSlotMeta(uint16_t offset) {
    uint8_t raw[8];
    i2cRead16(offset, raw, 8);
    SlotMetaV2 m;
    m.state      = raw[0];
    m.reserved0  = raw[1];
    m.generation = (raw[2] << 8) | raw[3];
    m.frame_len  = (raw[4] << 8) | raw[5];
    m.frame_xor  = raw[6];
    m.reserved1  = raw[7];
    return m;
}

void GestureTracker::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool GestureTracker::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, GT_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[GESTURE] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[GESTURE] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[GESTURE] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool GestureTracker::writeHostSlot(const uint8_t* frame, uint16_t len) {
    if (len > _slot_size || _slot_size == 0) return false;
    SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
    if (hm.state != SLOT_EMPTY) return false;
    _host_gen = (_host_gen + 1) & 0xFFFF;
    if (_host_gen == 0) _host_gen = 1;
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) xor_val ^= frame[i];
    SlotMetaV2 wm = {SLOT_WRITING, 0, _host_gen, len, xor_val, 0};
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    i2cWrite16(_host_slot_data_offset, frame, len);
    wm.state = SLOT_READY;
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    return true;
}

int GestureTracker::readDevSlot(uint8_t* buf, uint16_t bufSize) {
    SlotMetaV2 dm = readSlotMeta(DEV_SLOT_META_OFFSET);
    if (dm.state != SLOT_READY) return 0;
    if (dm.generation == _dev_gen) {
        SlotMetaV2 ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
        writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
        return 0;
    }
    uint16_t flen = dm.frame_len;
    if (flen < 8 || flen > bufSize || flen > _slot_size) {
        SlotMetaV2 ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
        writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
        _dev_gen = dm.generation;
        return 0;
    }
    i2cRead16(_dev_slot_data_offset, buf, flen);
    SlotMetaV2 ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
    writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
    _dev_gen = dm.generation;
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < flen; i++) xor_val ^= buf[i];
    if (xor_val != dm.frame_xor) return -1;
    return (int)flen;
}

// ── Protocol v2 frame build ──

uint8_t GestureTracker::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t GestureTracker::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn)
{
    buf[0] = FRAME_H0; buf[1] = FRAME_H1;
    buf[2] = (uint8_t)(plen >> 8); buf[3] = (uint8_t)(plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD; buf[5] = func; buf[6] = txn;
    if (payload && plen > 0) memcpy(&buf[7], payload, plen);
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= buf[i];
    buf[7 + plen] = xor_val;
    return 8 + plen;
}

bool GestureTracker::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void GestureTracker::begin() {
    _busy = false;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;
    _lastGesture = GESTURE_NONE;
    memset(_lastGestureStr, 0, sizeof(_lastGestureStr));
    _pendingGesture = GESTURE_NONE;
    _pendingGestureCount = 0;
    Wire1.setClock(400000);
}

void GestureTracker::start() {
    if (_busy) return;
    _busy = true;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;
    _lastGesture = GESTURE_NONE;
    memset(_lastGestureStr, 0, sizeof(_lastGestureStr));
    _pendingGesture = GESTURE_NONE;
    _pendingGestureCount = 0;

    arm.move(S_X, S_Y, S_Z, S_P, 0, 0, 1000);
    delay(1500);
    Serial.printf("[GESTURE] init pos: x=%.0f y=%.0f z=%.0f pitch=%.0f\n", S_X, S_Y, S_Z, S_P);

    if (!initMailbox()) {
        Serial.println("[GESTURE] mailbox init fail, abort");
        _busy = false;
        return;
    }

    sendCmd(CMD_SET_MODE, (const uint8_t[]){APP_HAND_RECOGNITION}, 1);
    Serial.printf("[GESTURE] SET_MODE=%d sent, waiting ready...\n", APP_HAND_RECOGNITION);

    bool mode_ok = false;
    for (int retry = 0; retry < 150; retry++) {
        delay(100);
        uint8_t buf[MAX_FRAME_SIZE];
        int got = readDevSlot(buf, sizeof(buf));
        if (got > 0 && got >= 8 && buf[0] == FRAME_H0 && buf[1] == FRAME_H1) {
            uint8_t ctrl = buf[4];
            uint8_t func = buf[5];
            uint8_t ftype = ctrl & 0xC0;
            uint16_t plen = (buf[2] << 8) | buf[3];

            Serial.printf("[GESTURE] raw: %02X %02X %02X %02X %02X %02X %02X %02X | ftype=0x%02X func=0x%02X plen=%d\n",
                          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], ftype, func, plen);

            if (ftype == CTRL_TYPE_RPT && func == RPT_HEARTBEAT && plen >= 2) {
                uint8_t hb_mode = buf[7];
                uint8_t hb_status = buf[8];
                Serial.printf("[GESTURE] HB mode=%d status=0x%02X\n", hb_mode, hb_status);
                if (hb_mode == APP_HAND_RECOGNITION && (hb_status & HB_READY)) {
                    mode_ok = true; break;
                }
            } else {
                Serial.printf("[GESTURE] frame ftype=0x%02X func=0x%02X plen=%d\n", ftype, func, plen);
                if (plen > 0 && plen <= 16) {
                    Serial.print("[GESTURE]   data:");
                    for (uint16_t d = 0; d < plen && d < 16; d++) Serial.printf(" %02X", buf[7+d]);
                    Serial.println();
                }
            }
        }
    }
    if (mode_ok) {
        Serial.println("[GESTURE] Mode confirmed!");
    } else {
        Serial.println("[GESTURE] WARNING: mode not confirmed");
    }

    delay(50);
    forceConfig();

    _currentState = GT_RUNNING;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_i2c_time = millis();
    _cmd_sent = false;
    Serial.println("[GESTURE] Running!");
}

void GestureTracker::stop() {
    if (!_busy) return;
    _busy = false;
    _lastGesture = GESTURE_NONE;
    memset(_lastGestureStr, 0, sizeof(_lastGestureStr));
    _pendingGesture = GESTURE_NONE;
    _pendingGestureCount = 0;
    _currentState = GT_IDLE;

    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);
    arm.move(200, 0, 200, 0, 0, 0, 1500);
    Serial.println("[GESTURE] Stop");
}

void GestureTracker::forceConfig() {
    // Simple approach: send cmd, short delay, don't wait for RSP
    // K230 processes commands even if we don't read the RSP
    auto safeSend = [this](uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label) {
        // Drain dev slot
        uint8_t drain[MAX_FRAME_SIZE];
        readDevSlot(drain, sizeof(drain));
        delay(30);

        // Wait host slot empty (short)
        for (int i = 0; i < 30; i++) {
            SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
            if (hm.state == SLOT_EMPTY) break;
            readDevSlot(drain, sizeof(drain));
            delay(10);
        }

        bool ok = sendCmd(cmd, payload, plen);
        Serial.printf("[GESTURE] %s ok=%d\n", label, ok);

        // Wait for K230 to consume host slot
        for (int i = 0; i < 30; i++) {
            SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
            if (hm.state == SLOT_EMPTY) break;
            delay(10);
        }
        // Drain RSP
        readDevSlot(drain, sizeof(drain));
        delay(20);
    };

    safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
    safeSend(CMD_SET_CONF_THRESH, (const uint8_t[]){50}, 1, "SET_THRESH=50");
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");

    Serial.println("[GESTURE] forceConfig done");
}

// ── Gesture string parsing ──

GestureType GestureTracker::parseGestureString(const char* str, int len) {
    if (!str || len <= 0) return GESTURE_NONE;
    if (strcmp(str, "gun") == 0 || strcmp(str, "Gun") == 0)      return GESTURE_GUN;
    if (strcmp(str, "other") == 0 || strcmp(str, "Other") == 0)  return GESTURE_OTHER;
    if (strcmp(str, "five") == 0 || strcmp(str, "Five") == 0)    return GESTURE_FIVE;
    if (strcmp(str, "yeah") == 0 || strcmp(str, "Yeah") == 0)    return GESTURE_YEAH;
    return GESTURE_UNKNOWN;
}

// ── Gesture result handling ──

void GestureTracker::handleGestureResult(const char* gesture_str) {
    _lastGesture = parseGestureString(gesture_str, strlen(gesture_str));
    strncpy(_lastGestureStr, gesture_str, sizeof(_lastGestureStr) - 1);

    static GestureType last_triggered = GESTURE_NONE;
    static unsigned long last_trigger_time = 0;
    unsigned long now = millis();

    const char* names[] = {"NONE", "GUN", "OTHER", "YEAH", "FIVE", "UNKNOWN"};
    Serial.printf("[GESTURE] -> %s ('%s')\n", names[_lastGesture], gesture_str);

    if (_lastGesture == GESTURE_UNKNOWN) return;

    if (_lastGesture == _pendingGesture) {
        if (_pendingGestureCount < 255) _pendingGestureCount++;
    } else {
        _pendingGesture = _lastGesture;
        _pendingGestureCount = 1;
    }

    if (_pendingGestureCount < 15) return;

    if (_lastGesture == last_triggered && now - last_trigger_time < 3000) return;
    last_triggered = _lastGesture;
    last_trigger_time = now;
    _pendingGesture = GESTURE_NONE;
    _pendingGestureCount = 0;

    Serial.printf("[GESTURE] >>> TRIGGER: %s\n", names[_lastGesture]);

    switch (_lastGesture) {
        case GESTURE_GUN:
            Serial.println("[GESTURE] GUN -> action group 0");
            arm.board.action_group_run(0);
            break;
        case GESTURE_OTHER:
            Serial.println("[GESTURE] OTHER -> action group 0");
            arm.board.action_group_run(0);
            break;
        case GESTURE_YEAH:
            Serial.println("[GESTURE] YEAH -> action group 2");
            arm.board.action_group_run(2);
            break;
        case GESTURE_FIVE:
            Serial.println("[GESTURE] FIVE -> action group 3");
            arm.board.action_group_run(3);
            break;
        default: break;
    }
}

// ── RPT handlers ──

void GestureTracker::handleBbox(const uint8_t* payload, uint16_t plen) {
    if (plen < 1) return;
    uint8_t count = payload[0];
    if (count == 0) return;
    if (plen < 9) return;

    uint16_t off = 1;
    uint16_t cx = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t bw = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t bh = (payload[off] << 8) | payload[off+1]; off += 2;

    char gesture_str[32] = {0};
    int conf = 0;

    if (off < plen) {
        uint8_t ext_cnt = payload[off++];
        for (int e = 0; e < ext_cnt && off < plen; e++) {
            uint8_t etype = payload[off++];
            if (etype == 1) {
                if (off < plen) {
                    uint8_t slen = payload[off++];
                    if (slen > 0 && off + slen <= plen) {
                        int copy_len = (slen < 31) ? slen : 31;
                        memcpy(gesture_str, &payload[off], copy_len);
                        gesture_str[copy_len] = '\0';
                    }
                    off += slen;
                }
            } else if (etype == 2) {
                if (off + 2 <= plen) { conf = (payload[off] << 8) | payload[off+1]; off += 2; }
            } else if (etype == 3) {
                off += 2;
            } else if (etype == 4) {
                if (off < plen) { uint8_t kp_len = payload[off++]; off += kp_len; }
            } else {
                off += 2;
            }
        }
    }

    static unsigned long last_result = 0;
    if (millis() - last_result > 300) {
        Serial.printf("[GESTURE] bbox cx=%d cy=%d w=%d h=%d gesture='%s' conf=%d\n",
                      cx, cy, bw, bh, gesture_str, conf);
        last_result = millis();
    }

    if (gesture_str[0] != '\0') handleGestureResult(gesture_str);
}

void GestureTracker::handleHandKP(const uint8_t* payload, uint16_t plen) {
    if (plen < 1) return;
    uint8_t count = payload[0];
    if (count == 0) return;

    uint16_t off = 1;
    for (int h = 0; h < count && off + 8 <= plen; h++) {
        uint16_t cx = (payload[off] << 8) | payload[off+1]; off += 2;
        uint16_t cy = (payload[off] << 8) | payload[off+1]; off += 2;
        uint16_t bw = (payload[off] << 8) | payload[off+1]; off += 2;
        uint16_t bh = (payload[off] << 8) | payload[off+1]; off += 2;

        // 21 keypoints, 2 bytes each
        if (off + 42 <= plen) off += 42;

        char gesture_str[32] = {0};
        int conf = 0;
        if (off < plen) {
            uint8_t ext_cnt = payload[off++];
            for (int e = 0; e < ext_cnt && off < plen; e++) {
                uint8_t etype = payload[off++];
                if (etype == 1) {
                    if (off < plen) {
                        uint8_t slen = payload[off++];
                        if (slen > 0 && off + slen <= plen) {
                            int copy_len = (slen < 31) ? slen : 31;
                            memcpy(gesture_str, &payload[off], copy_len);
                            gesture_str[copy_len] = '\0';
                        }
                        off += slen;
                    }
                } else if (etype == 2) {
                    if (off + 2 <= plen) { conf = (payload[off] << 8) | payload[off+1]; off += 2; }
                } else if (etype == 3) {
                    off += 2;
                } else {
                    off += 2;
                }
            }
        }

        static unsigned long last_kp_print = 0;
        if (millis() - last_kp_print > 300) {
            Serial.printf("[GESTURE] hand[%d] cx=%d cy=%d w=%d h=%d gesture='%s' conf=%d\n",
                          h, cx, cy, bw, bh, gesture_str, conf);
            last_kp_print = millis();
        }

        if (gesture_str[0] != '\0') handleGestureResult(gesture_str);
    }
}

void GestureTracker::handleDetectStr(const uint8_t* payload, uint16_t plen) {
    if (plen <= 0) return;
    int slen = (plen < 31) ? plen : 31;
    char str[32] = {0};
    memcpy(str, payload, slen);
    str[slen] = '\0';
    Serial.printf("[GESTURE] STR: '%s'\n", str);
    handleGestureResult(str);
}

// ── Frame parsing ──

void GestureTracker::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8) return;
    if (frame[0] != FRAME_H0 || frame[1] != FRAME_H1) return;
    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) return;
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= frame[i];
    if (xor_val != frame[7 + plen]) return;
    uint8_t ftype = frame[4] & 0xC0;
    uint8_t func = frame[5];
    const uint8_t* payload = &frame[7];
    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_DETECT_BBOX)        handleBbox(payload, plen);
        else if (func == RPT_DETECT_HAND_KP) handleHandKP(payload, plen);
        else if (func == RPT_DETECT_STR)     handleDetectStr(payload, plen);
    }
}

// ── Main update loop ──

void GestureTracker::update() {
    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) {
        static unsigned long last_dbg = 0;
        if (now - last_dbg > 500) {
            uint8_t ftype = (got >= 5) ? (buf[4] & 0xC0) : 0;
            uint8_t func = (got >= 6) ? buf[5] : 0;
            uint16_t plen = (got >= 4) ? ((buf[2]<<8)|buf[3]) : 0;
            Serial.printf("[GESTURE] update got=%d ftype=0x%02X func=0x%02X plen=%d\n", got, ftype, func, plen);
            last_dbg = now;
        }
        parseFrame(buf, (uint16_t)got);
    }
}
