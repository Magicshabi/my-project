#include "AprilTagTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

// Protocol v2 constants
#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_QUAD          0x7B
#define APP_APRILTAG             32
#define APP_EMPTY                0

// Ctrl field masks
#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

// Heartbeat status bits
#define HB_RUN                   0x01
#define HB_READY                 0x04

// Tracking tuning
#define MOVE_INTERVAL            30
#define I2C_POLL_INTERVAL        10
#define CENTER_DEADBAND          10
#define CENTER_RELEASE_BAND      16
#define D_TERM_MAX               40.0f
#define TRACK_MOVE_DURATION      50
#define LOST_TIMEOUT_MS          300
#define TARGET_Y_MAX             250.0f
#define TARGET_Y_MIN            -250.0f
#define TARGET_Z_MAX             300.0f
#define TARGET_Z_MIN             175.0f

static const uint8_t TAG_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

AprilTagTracker aprilTagTracker;

// ── I2C low-level ──

void AprilTagTracker::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void AprilTagTracker::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

AprilTagTracker::SlotMetaV2 AprilTagTracker::readSlotMeta(uint16_t offset) {
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

void AprilTagTracker::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool AprilTagTracker::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    Serial.printf("[TAG] mailbox header: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  header[0], header[1], header[2], header[3],
                  header[4], header[5], header[6], header[7]);
    if (memcmp(header, TAG_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[TAG] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[TAG] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[TAG] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool AprilTagTracker::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int AprilTagTracker::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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
    if (xor_val != dm.frame_xor) {
        Serial.printf("[TAG] dev slot xor fail: calc=%02X exp=%02X\n", xor_val, dm.frame_xor);
        return -1;
    }
    return (int)flen;
}

// ── Protocol v2 frame build ──

uint8_t AprilTagTracker::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t AprilTagTracker::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn)
{
    buf[0] = FRAME_H0;
    buf[1] = FRAME_H1;
    buf[2] = (uint8_t)(plen >> 8);
    buf[3] = (uint8_t)(plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD;
    buf[5] = func;
    buf[6] = txn;
    if (payload && plen > 0) memcpy(&buf[7], payload, plen);
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= buf[i];
    buf[7 + plen] = xor_val;
    return 8 + plen;
}

bool AprilTagTracker::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void AprilTagTracker::begin() {
    _busy = false;
    _txn = 0;
    _last_ex = 0;
    _last_ey = 0;
    _target_x = 200;
    _target_y = 0;
    _target_z = 200;
    _last_detect_time = 0;
    _center_locked = false;
    _host_gen = 0; _dev_gen = 0;
    Wire1.setClock(400000);
}

void AprilTagTracker::start() {
    if (_busy) return;
    _busy = true;
    _txn = 0;
    _last_ex = 0; _last_ey = 0;
    _target_x = 200; _target_y = 0; _target_z = 200;
    _last_detect_time = 0;
    _center_locked = false;
    _host_gen = 0; _dev_gen = 0;

    arm.move(_target_x, _target_y, _target_z, 0, 0, 0, 1000);

    _ignore_data_until = millis() + 1500;
    _last_i2c_time = millis();
    _cmd_sent = false;

    delay(500);
    if (!initMailbox()) {
        Serial.println("[TAG] mailbox init fail, abort");
        _busy = false;
        return;
    }

    _currentState = T_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    Serial.println("[TAG] start -> SET_MODE");
}

void AprilTagTracker::stop() {
    if (!_busy) return;
    _busy = false;
    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);
    arm.move(200, 0, 200, 0, 0, 0, 500);
    Serial.println("[TAG] Stop");
}

void AprilTagTracker::forceConfig() {
    uint8_t thresh = 5;
    sendCmd(CMD_SET_CONF_THRESH, &thresh, 1);
    delay(30);
    uint8_t simple_off = 0;
    sendCmd(CMD_SET_SIMPLE_RESULT, &simple_off, 1);
    delay(30);
    uint8_t run_en = 0;
    sendCmd(CMD_DISABLE_RUN, &run_en, 1);
    delay(30);
    Serial.println("[TAG] forceConfig done");
}

void AprilTagTracker::runStateMachine(uint8_t mode, uint8_t status) {
    bool isRun   = status & HB_RUN;
    bool isReady = status & HB_READY;

    Serial.printf("[TAG] HB mode=%d status=0x%02X ready=%d run=%d state=%d\n",
                  mode, status, isReady, isRun, _currentState);

    if (_currentState == T_STATE_WAIT_READY) {
        if (mode == APP_APRILTAG && isReady) {
            _currentState = T_STATE_CONFIG_PARAMS;
            _last_state_time = millis();
            _cmd_sent = false;
            Serial.println("[TAG] -> CONFIG_PARAMS");
        }
    } else if (_currentState == T_STATE_RUNNING && !isRun) {
        _currentState = T_STATE_ENABLE_RUN;
        _last_state_time = millis();
        Serial.println("[TAG] lost RUN -> ENABLE_RUN");
    }
}

void AprilTagTracker::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    runStateMachine(payload[0], payload[1]);
}

void AprilTagTracker::handleBbox(const uint8_t* payload, uint16_t plen) {
    static unsigned long last_move_cmd_time = 0;

    if (_currentState < T_STATE_RUNNING) {
        _currentState = T_STATE_RUNNING;
        _last_state_time = millis();
        Serial.println("[TAG] -> RUNNING");
    }
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0 || off + 8 > plen) return;

    uint16_t cx = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t cy = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bw = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bh = (payload[off]<<8)|payload[off+1]; off += 2;

    trackCenter((int)cx, (int)cy, last_move_cmd_time);
}

void AprilTagTracker::handleQuad(const uint8_t* payload, uint16_t plen) {
    if (_currentState < T_STATE_RUNNING) {
        _currentState = T_STATE_RUNNING;
        _last_state_time = millis();
        Serial.println("[TAG] -> RUNNING (quad)");
    }
    if (millis() < _ignore_data_until) {
        return;
    }
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) {
        static unsigned long last_empty = 0;
        if (millis() - last_empty > 1000) {
            Serial.println("[TAG] quad count=0");
            last_empty = millis();
        }
        return;
    }
    if (off + 16 > plen) {
        Serial.printf("[TAG] quad plen too short: %d\n", plen);
        return;
    }

    // 4 corners: [x0:s16][y0:s16]...[x3:s16][y3:s16]
    int16_t qx[4], qy[4];
    for (uint8_t i = 0; i < 4; i++) {
        qx[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
        qy[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
    }

    // Center of 4 corners
    int cx = (qx[0] + qx[1] + qx[2] + qx[3]) / 4;
    int cy = (qy[0] + qy[1] + qy[2] + qy[3]) / 4;

    static unsigned long last_move_cmd_time = 0;
    trackCenter(cx, cy, last_move_cmd_time);
}

void AprilTagTracker::trackCenter(int cx, int cy, unsigned long &last_move_cmd_time) {
    unsigned long now = millis();

    // Reset D term after lost timeout
    if (now - _last_detect_time > LOST_TIMEOUT_MS) {
        _last_ex = 0; _last_ey = 0;
    }
    _last_detect_time = now;

    int ex = 160 - cx;
    int ey = 120 - cy;

    static unsigned long last_log = 0;
    if (now - last_log > 500) {
        Serial.printf("[TAG] cx=%d cy=%d ex=%d ey=%d ty=%.0f tz=%.0f\n",
                      cx, cy, ex, ey, _target_y, _target_z);
        last_log = now;
    }

    // Hysteresis deadband
    if (_center_locked) {
        if (abs(ex) <= CENTER_RELEASE_BAND && abs(ey) <= CENTER_RELEASE_BAND) {
            ex = 0; ey = 0;
        } else {
            _center_locked = false;
        }
    }
    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        _center_locked = true;
        _last_ex = 0; _last_ey = 0;
        return;
    }

    float dx = constrain((float)(ex - _last_ex), -D_TERM_MAX, D_TERM_MAX);
    float dy = constrain((float)(ey - _last_ey), -D_TERM_MAX, D_TERM_MAX);
    float out_y = ex * kp + dx * kd;
    float out_z = ey * kp + dy * kd;
    _last_ex = ex; _last_ey = ey;

    _target_y = constrain(_target_y + out_y, TARGET_Y_MIN, TARGET_Y_MAX);
    _target_z = constrain(_target_z + out_z, TARGET_Z_MIN, TARGET_Z_MAX);

    if (now - last_move_cmd_time >= MOVE_INTERVAL) {
        arm.move(_target_x, _target_y, _target_z, 0, 0, 0, TRACK_MOVE_DURATION);
        last_move_cmd_time = now;
    }
}

// ── Frame parsing (v2 format) ──

void AprilTagTracker::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8) return;
    if (frame[0] != FRAME_H0 || frame[1] != FRAME_H1) return;

    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) return;

    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= frame[i];
    if (xor_val != frame[7 + plen]) {
        Serial.printf("[TAG] frame xor fail calc=%02X got=%02X\n", xor_val, frame[7 + plen]);
        return;
    }

    uint8_t ctrl = frame[4];
    uint8_t func = frame[5];
    const uint8_t* payload = &frame[7];
    uint8_t ftype = ctrl & 0xC0;

    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_HEARTBEAT)        handleHeartbeat(payload, plen);
        else if (func == RPT_DETECT_QUAD) handleQuad(payload, plen);
        else if (func == RPT_DETECT_BBOX) handleBbox(payload, plen);
    }
}

// ── Main update loop ──

void AprilTagTracker::update() {
    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) {
        parseFrame(buf, (uint16_t)got);
    }

    switch (_currentState) {
        case T_STATE_SET_MODE: {
            uint8_t m = APP_APRILTAG;
            if (sendCmd(CMD_SET_MODE, &m, 1)) {
                _currentState = T_STATE_WAIT_READY;
                _last_state_time = now;
                Serial.println("[TAG] SET_MODE(32) -> WAIT_READY");
            }
            break;
        }
        case T_STATE_WAIT_READY:
            if (now - _last_poll_time > 500) {
                sendCmd(CMD_REQUEST_STATUS);
                _last_poll_time = now;
            }
            if (now - _last_state_time > 10000) {
                _currentState = T_STATE_SET_MODE;
                _last_state_time = now;
                Serial.println("[TAG] WAIT_READY timeout, retry");
            }
            break;

        case T_STATE_CONFIG_PARAMS:
            if (!_cmd_sent) {
                forceConfig();
                _cmd_sent = true;
                _last_state_time = now;
            }
            if (now - _last_state_time > 800) {
                _currentState = T_STATE_ENABLE_RUN;
                _last_state_time = now;
                Serial.println("[TAG] -> ENABLE_RUN");
            }
            break;

        case T_STATE_ENABLE_RUN:
            if (now - _last_state_time > 500) {
                uint8_t run_en = 0;
                sendCmd(CMD_DISABLE_RUN, &run_en, 1);
                _currentState = T_STATE_RUNNING;
                _last_state_time = now;
                Serial.println("[TAG] -> RUNNING!");
            }
            break;

        case T_STATE_RUNNING:
            break;

        default:
            break;
    }
}
