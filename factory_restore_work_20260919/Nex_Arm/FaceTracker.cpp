#include "FaceTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

// Protocol v2 constants
#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14
#define CMD_FACE_SET_RECOG_CONF  0x23
#define CMD_FACE_HIGH_PRECISION  0x24

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_CENTER        0x79
#define RPT_DETECT_FACE_KP       0x7A

#define APP_EMPTY                0
#define APP_FACE_DETECTION       1

// Ctrl field masks
#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

// Heartbeat status bits
#define HB_RUN                   0x01
#define HB_RESULT                0x02
#define HB_READY                 0x04

// Tracking tuning
#define MOVE_INTERVAL            30
#define I2C_POLL_INTERVAL        5
#define CENTER_DEADBAND          10
#define CENTER_RELEASE_BAND      16
#define TRACK_MOVE_DURATION      50

// Mailbox v2 magic
static const uint8_t MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32}; // "WLM2"

FaceTracker faceTracker;

namespace {
bool g_face_center_locked = false;
}

// ── I2C low-level (16-bit memaddr for mailbox v2) ──

void FaceTracker::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void FaceTracker::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

SlotMeta FaceTracker::readSlotMeta(uint16_t offset) {
    uint8_t raw[8];
    i2cRead16(offset, raw, 8);
    SlotMeta m;
    m.state      = raw[0];
    m.reserved0  = raw[1];
    m.generation = (raw[2] << 8) | raw[3];
    m.frame_len  = (raw[4] << 8) | raw[5];
    m.frame_xor  = raw[6];
    m.reserved1  = raw[7];
    return m;
}

void FaceTracker::writeSlotMeta(uint16_t offset, const SlotMeta& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool FaceTracker::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    Serial.printf("[FACE] mailbox header: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  header[0], header[1], header[2], header[3],
                  header[4], header[5], header[6], header[7]);
    if (memcmp(header, MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[FACE] mailbox magic mismatch, not WLM2");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[FACE] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[FACE] mailbox ok slot_size=%d host_data@%d dev_data@%d\n",
                  _slot_size, _host_slot_data_offset, _dev_slot_data_offset);
    return true;
}

bool FaceTracker::writeHostSlot(const uint8_t* frame, uint16_t len) {
    if (len > _slot_size || _slot_size == 0) return false;
    SlotMeta hm = readSlotMeta(HOST_SLOT_META_OFFSET);
    if (hm.state != SLOT_EMPTY) return false;

    _host_gen = (_host_gen + 1) & 0xFFFF;
    if (_host_gen == 0) _host_gen = 1;

    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) xor_val ^= frame[i];

    SlotMeta wm = {SLOT_WRITING, 0, _host_gen, len, xor_val, 0};
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    i2cWrite16(_host_slot_data_offset, frame, len);
    wm.state = SLOT_READY;
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    return true;
}

int FaceTracker::readDevSlot(uint8_t* buf, uint16_t bufSize) {
    SlotMeta dm = readSlotMeta(DEV_SLOT_META_OFFSET);
    if (dm.state != SLOT_READY) return 0;
    if (dm.generation == _dev_gen) {
        // Stuck READY with same generation — re-ack
        SlotMeta ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
        writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
        return 0;
    }
    uint16_t flen = dm.frame_len;
    if (flen < 8 || flen > bufSize || flen > _slot_size) {
        SlotMeta ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
        writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
        _dev_gen = dm.generation;
        return 0;
    }
    i2cRead16(_dev_slot_data_offset, buf, flen);

    // Ack immediately
    SlotMeta ack = {SLOT_EMPTY, 0, dm.generation, 0, 0, 0};
    writeSlotMeta(DEV_SLOT_META_OFFSET, ack);
    _dev_gen = dm.generation;

    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < flen; i++) xor_val ^= buf[i];
    if (xor_val != dm.frame_xor) {
        Serial.printf("[FACE] dev slot xor fail: calc=%02X exp=%02X\n", xor_val, dm.frame_xor);
        return -1;
    }
    return (int)flen;
}

// ── Protocol v2 frame build ──

uint8_t FaceTracker::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t FaceTracker::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn)
{
    buf[0] = FRAME_H0;
    buf[1] = FRAME_H1;
    buf[2] = (uint8_t)(plen >> 8);
    buf[3] = (uint8_t)(plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD;  // no continuation, seq=0
    buf[5] = func;
    buf[6] = txn;
    if (payload && plen > 0) memcpy(&buf[7], payload, plen);
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= buf[i];
    buf[7 + plen] = xor_val;
    return 8 + plen;
}

bool FaceTracker::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void FaceTracker::begin() {
    _busy = false;
    _txn = 0;
    _last_ex = 0;
    _last_ey = 0;
    _last_i2c_time = 0;
    _host_gen = 0; _dev_gen = 0;
    g_face_center_locked = false;
    Wire1.setClock(400000);
}

void FaceTracker::start() {
    if (_busy) return;
    _busy = true;
    _txn = 0;
    _last_ex = 0; _last_ey = 0;
    _target_x = 200; _target_y = 0; _target_z = 200;
    g_face_center_locked = false;
    arm.move(_target_x, _target_y, _target_z, 0, 0, 0, 1000);
    _ignore_data_until = millis() + 1500;
    _last_i2c_time = millis();
    _cmd_sent = false;

    delay(500);
    if (!initMailbox()) {
        Serial.println("[FACE] mailbox init fail, abort");
        _busy = false;
        return;
    }

    _currentState = FT_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    Serial.println("[FACE] start -> SET_MODE");
}

void FaceTracker::stop() {
    if (!_busy) return;
    _busy = false;
    g_face_center_locked = false;
    _currentState = FT_IDLE;
    _cmd_sent = false;
    _last_ex = 0; _last_ey = 0;

    uint8_t run_off = 1;
    sendCmd(CMD_DISABLE_RUN, &run_off, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);

    arm.move(200, 0, 200, 0, 0, 0, 1500);
    Serial.println("[FACE] stop");
}

void FaceTracker::forceConfig() {
    uint8_t thresh = 30;
    sendCmd(CMD_SET_CONF_THRESH, &thresh, 1);
    delay(30);
    Serial.println("[FACE] forceConfig done (FaceDetection)");
}

void FaceTracker::runStateMachine(uint8_t mode, uint8_t status) {
    bool isRun   = status & HB_RUN;
    bool isReady = status & HB_READY;

    if (_currentState == FT_WAIT_READY) {
        if (mode == APP_FACE_DETECTION && isReady) {
            _currentState = FT_CONFIG_PARAMS;
            _last_state_time = millis();
            _cmd_sent = false;
        }
    } else if (_currentState == FT_RUNNING && !isRun) {
        _currentState = FT_ENABLE_RUN;
        _last_state_time = millis();
    }
}

void FaceTracker::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    static unsigned long last_hb_log = 0;
    if (millis() - last_hb_log > 1000) {
        Serial.printf("[FACE] HB mode=%d status=0x%02X state=%d\n",
                      payload[0], payload[1], _currentState);
        last_hb_log = millis();
    }
    runStateMachine(payload[0], payload[1]);
}

void FaceTracker::handleFaceKp(const uint8_t* payload, uint16_t plen) {
    (void)payload; (void)plen;
}

void FaceTracker::handleCenter(const uint8_t* payload, uint16_t plen) {
    static unsigned long last_move_cmd_time = 0;

    if (_currentState < FT_RUNNING) {
        _currentState = FT_RUNNING;
        _last_state_time = millis();
    }
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) return;

    if (off + 4 > plen) return;
    uint16_t cx = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off+1]; off += 2;

    static unsigned long last_log = 0;
    if (millis() - last_log > 500) {
        Serial.printf("[FACE] detect cx=%d cy=%d count=%d\n", cx, cy, count);
        last_log = millis();
    }

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    if (g_face_center_locked) {
        if (abs(ex) <= CENTER_RELEASE_BAND && abs(ey) <= CENTER_RELEASE_BAND) {
            ex = 0; ey = 0;
        } else {
            g_face_center_locked = false;
        }
    }
    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        g_face_center_locked = true;
        _last_ex = 0; _last_ey = 0;
        return;
    }

    float dx = (float)(ex - _last_ex);
    float dy = (float)(ey - _last_ey);
    float out_y = ex * kp + dx * kd;
    float out_z = ey * kp + dy * kd;
    _last_ex = ex; _last_ey = ey;

    _target_y = constrain(_target_y + out_y, -250.0f, 250.0f);
    _target_z = constrain(_target_z + out_z, 175.0f, 350.0f);

    unsigned long now = millis();
    if (now - last_move_cmd_time >= MOVE_INTERVAL) {
        arm.move(_target_x, _target_y, _target_z, 0, 0, 0, TRACK_MOVE_DURATION);
        last_move_cmd_time = now;
    }
}

void FaceTracker::handleBbox(const uint8_t* payload, uint16_t plen) {
    static unsigned long last_move_cmd_time = 0;

    if (_currentState < FT_RUNNING) {
        _currentState = FT_RUNNING;
        _last_state_time = millis();
    }
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) return;

    if (off + 8 > plen) return;
    uint16_t cx = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t cy = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bw = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bh = (payload[off]<<8)|payload[off+1]; off += 2;

    static unsigned long last_log = 0;
    if (millis() - last_log > 500) {
        Serial.printf("[FACE] bbox detect cx=%d cy=%d count=%d\n", cx, cy, count);
        last_log = millis();
    }

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    if (g_face_center_locked) {
        if (abs(ex) <= CENTER_RELEASE_BAND && abs(ey) <= CENTER_RELEASE_BAND) {
            ex = 0; ey = 0;
        } else {
            g_face_center_locked = false;
        }
    }
    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        g_face_center_locked = true;
        _last_ex = 0; _last_ey = 0;
        return;
    }

    float dx = (float)(ex - _last_ex);
    float dy = (float)(ey - _last_ey);
    float out_y = (float)ex * kp + dx * kd;
    float out_z = (float)ey * kp + dy * kd;
    _last_ex = ex; _last_ey = ey;

    _target_y = constrain(_target_y + out_y, -250.0f, 250.0f);
    _target_z = constrain(_target_z + out_z, 175.0f, 350.0f);

    unsigned long now = millis();
    if (now - last_move_cmd_time >= MOVE_INTERVAL) {
        arm.move(_target_x, _target_y, _target_z, 0, 0, 0, TRACK_MOVE_DURATION);
        last_move_cmd_time = now;
    }
}

// ── Frame parsing (v2 format) ──

void FaceTracker::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8) return;
    if (frame[0] != FRAME_H0 || frame[1] != FRAME_H1) return;

    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) return;

    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= frame[i];
    if (xor_val != frame[7 + plen]) {
        Serial.printf("[FACE] frame xor fail calc=%02X got=%02X\n", xor_val, frame[7 + plen]);
        return;
    }

    uint8_t ctrl = frame[4];
    uint8_t func = frame[5];
    const uint8_t* payload = &frame[7];
    uint8_t ftype = ctrl & 0xC0;

    static unsigned long last_frame_log = 0;
    if (millis() - last_frame_log > 300) {
        Serial.printf("[FACE] rx type=%02X func=%02X plen=%d\n", ftype, func, plen);
        last_frame_log = millis();
    }

    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_HEARTBEAT)           handleHeartbeat(payload, plen);
        else if (func == RPT_DETECT_BBOX)    handleBbox(payload, plen);
        else if (func == RPT_DETECT_CENTER)  handleCenter(payload, plen);
        else if (func == RPT_DETECT_FACE_KP) handleFaceKp(payload, plen);
    }
}

// ── Main update loop ──

void FaceTracker::update() {
    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    // Poll device slot for incoming frames
    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) {
        parseFrame(buf, (uint16_t)got);
    }

    // Periodic state debug
    static unsigned long last_state_log = 0;
    if (now - last_state_log > 2000) {
        Serial.printf("[FACE] state=%d busy=%d\n", _currentState, _busy);
        last_state_log = now;
    }

    switch (_currentState) {
        case FT_SET_MODE: {
            uint8_t mode = APP_FACE_DETECTION;
            if (sendCmd(CMD_SET_MODE, &mode, 1)) {
                _currentState = FT_WAIT_READY;
                _last_state_time = now;
                Serial.println("[FACE] SET_MODE -> WAIT_READY");
            }
            break;
        }
        case FT_WAIT_READY:
            if (now - _last_poll_time > 500) {
                sendCmd(CMD_REQUEST_STATUS);
                _last_poll_time = now;
            }
            if (now - _last_state_time > 10000) {
                _currentState = FT_SET_MODE;
                _last_state_time = now;
                Serial.println("[FACE] WAIT_READY timeout, retry");
            }
            break;

        case FT_CONFIG_PARAMS:
            if (!_cmd_sent) {
                forceConfig();
                _cmd_sent = true;
                _last_state_time = now;
            }
            if (now - _last_state_time > 500) {
                _currentState = FT_ENABLE_RUN;
                _last_state_time = now;
                _cmd_sent = false;
                Serial.println("[FACE] -> ENABLE_RUN");
            }
            break;

        case FT_ENABLE_RUN: {
            if (now - _last_state_time > 300) {
                uint8_t run_en = 0;
                sendCmd(CMD_DISABLE_RUN, &run_en, 1);
                _currentState = FT_RUNNING;
                _last_state_time = now;
                Serial.println("[FACE] ENABLE_RUN -> RUNNING");
            }
            break;
        }
        case FT_RUNNING:
            break;

        default:
            break;
    }
}
