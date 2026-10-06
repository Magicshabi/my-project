#include "ColorTrackerRot.h"
#include <Wire.h>
#include "Robot_Arm.h"

// Protocol v2 constants
#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14
#define CMD_COLOR_SET_TARGET     0x40

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_COLOR         0x75
#define RPT_DETECT_CENTER        0x79

#define APP_EMPTY                0
#define APP_SINGLE_COLOR         21

// Ctrl field masks
#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

// Heartbeat status bits
#define HB_RUN                   0x01
#define HB_RESULT                0x02
#define HB_READY                 0x04

#define MOVE_INTERVAL            30
#define CENTER_DEADBAND          10
#define CENTER_RELEASE_BAND      16
#define D_TERM_MAX               40.0f
#define TRACK_MOVE_DURATION      50
#define LOST_TIMEOUT_MS          300
#define TARGET_Y_MAX             250.0f
#define TARGET_Y_MIN            -250.0f
#define TARGET_Z_MAX             300.0f
#define TARGET_Z_MIN             175.0f
#define I2C_POLL_INTERVAL        5

// Mailbox v2 magic
static const uint8_t CT_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32}; // "WLM2"

ColorTrackerRot colorTrackerRot;

// ── I2C low-level (16-bit memaddr for mailbox v2) ──

void ColorTrackerRot::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void ColorTrackerRot::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

ColorTrackerRot::SlotMetaV2 ColorTrackerRot::readSlotMeta(uint16_t offset) {
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

void ColorTrackerRot::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool ColorTrackerRot::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    Serial.printf("[COLOR] mailbox header: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  header[0], header[1], header[2], header[3],
                  header[4], header[5], header[6], header[7]);
    if (memcmp(header, CT_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[COLOR] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[COLOR] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[COLOR] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool ColorTrackerRot::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int ColorTrackerRot::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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
        Serial.printf("[COLOR] dev slot xor fail: calc=%02X exp=%02X\n", xor_val, dm.frame_xor);
        return -1;
    }
    return (int)flen;
}

// ── Protocol v2 frame build ──

uint8_t ColorTrackerRot::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t ColorTrackerRot::buildFrame(uint8_t* buf, uint8_t func,
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

bool ColorTrackerRot::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void ColorTrackerRot::begin() {
    _busy = false;
    _center_locked = false;
    _txn = 0;
    _last_ex = 0;
    _last_ey = 0;
    _f_tx = 200;
    _f_ty = 0;
    _f_tz = 200;
    _host_gen = 0; _dev_gen = 0;
    Wire1.setClock(400000);
}

void ColorTrackerRot::start(const char* colorName) {
    if (_busy) return;
    _busy = true;
    _center_locked = false;
    _target_color = String(colorName);
    Serial.printf("[COLOR] start color='%s'\n", colorName);
    _txn = 0;
    _last_ex = 0;
    _last_ey = 0;
    _host_gen = 0; _dev_gen = 0;

    _f_tx = 200;
    _f_ty = 0;
    _f_tz = 200;
    arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, 1000);
    _ignore_data_until = millis() + 1500;
    _last_i2c_time = millis();
    _cmd_sent = false;

    delay(500);
    if (!initMailbox()) {
        Serial.println("[COLOR] mailbox init fail, abort");
        _busy = false;
        return;
    }

    // 先确保 K230 处于空闲状态（上一个追踪器可能没完全退出）
    uint8_t run_off = 1;
    sendCmd(CMD_DISABLE_RUN, &run_off, 1);
    delay(50);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(100);

    _currentState = CT_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    Serial.println("[COLOR] start -> SET_MODE");
}

void ColorTrackerRot::stop() {
    if (!_busy) return;
    _busy = false;
    _center_locked = false;
    _cmd_sent = false;
    _last_ex = 0;
    _last_ey = 0;

    uint8_t run_off = 1;
    sendCmd(CMD_DISABLE_RUN, &run_off, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);

    arm.move(200, 0, 200, 0, 0, 0, 1500);
    Serial.println("[COLOR] stop");
}

void ColorTrackerRot::forceConfig() {
    uint8_t thresh = 30;
    sendCmd(CMD_SET_CONF_THRESH, &thresh, 1);
    delay(30);

    // Query color thresh to check if name exists
    if (_target_color.length() > 0) {
        uint8_t query_buf[32];
        uint8_t n_len = _target_color.length();
        if (n_len > 30) n_len = 30;
        query_buf[0] = n_len;
        memcpy(&query_buf[1], _target_color.c_str(), n_len);

        // Try SET_TARGET
        bool ok = sendCmd(CMD_COLOR_SET_TARGET, query_buf, n_len + 1);
        Serial.printf("[COLOR] SET_TARGET '%s' len=%d ok=%d\n", _target_color.c_str(), n_len, ok);
        delay(50);

        // Also query thresh to see RSP
        sendCmd(0x42, query_buf, n_len + 1);  // CMD_COLOR_GET_THRESH
        Serial.printf("[COLOR] GET_THRESH '%s' sent\n", _target_color.c_str());
        delay(50);
    }

    uint8_t simple = 0;
    sendCmd(CMD_SET_SIMPLE_RESULT, &simple, 1);
    delay(30);
    Serial.println("[COLOR] forceConfig done");
}

void ColorTrackerRot::runStateMachine(uint8_t mode, uint8_t status) {
    bool isReady = status & HB_READY;

    if (_currentState == CT_STATE_WAIT_READY) {
        if (mode == APP_SINGLE_COLOR && isReady) {
            _currentState = CT_STATE_CONFIG_PARAMS;
            _last_state_time = millis();
            _cmd_sent = false;
            Serial.println("[COLOR] -> CONFIG_PARAMS");
        }
    } else if (_currentState == CT_STATE_RUNNING && !(status & HB_RUN)) {
        _currentState = CT_STATE_ENABLE_RUN;
        _last_state_time = millis();
    }
}

void ColorTrackerRot::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    static unsigned long last_hb_log = 0;
    if (millis() - last_hb_log > 1000) {
        Serial.printf("[COLOR] HB mode=%d status=0x%02X state=%d\n",
                      payload[0], payload[1], _currentState);
        last_hb_log = millis();
    }
    runStateMachine(payload[0], payload[1]);
}

void ColorTrackerRot::handleColor(const uint8_t* payload, uint16_t plen) {
    static unsigned long last_move_cmd_time = 0;

    if (_currentState < CT_STATE_RUNNING) {
        _currentState = CT_STATE_RUNNING;
        _last_state_time = millis();
    }
    if (millis() < _ignore_data_until) return;
    if (plen < 3) return;

    // Parse multi_color format: color_count + [name_len + name + blob_count + blobs...]
    uint16_t off = 0;
    uint8_t color_count = payload[off++];
    if (color_count == 0) return;

    // Read first color group
    if (off >= plen) return;
    uint8_t name_len = payload[off++];
    off += name_len; // skip name
    if (off >= plen) return;

    uint8_t blob_count = payload[off++];
    if (blob_count == 0 || off + 10 > plen) return;

    // Read first blob: cx(2) + cy(2) + w(2) + h(2) + angle(2)
    uint16_t cx = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t w  = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t h  = (payload[off] << 8) | payload[off + 1]; off += 2;
    // skip angle
    off += 2;

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    if (_center_locked) {
        if (abs(ex) <= CENTER_RELEASE_BAND && abs(ey) <= CENTER_RELEASE_BAND) {
            ex = 0;
            ey = 0;
        } else {
            _center_locked = false;
        }
    }

    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        _center_locked = true;
        _last_ex = 0;
        _last_ey = 0;
        return;
    }

    float dx = constrain((float)(ex - _last_ex), -D_TERM_MAX, D_TERM_MAX);
    float dy = constrain((float)(ey - _last_ey), -D_TERM_MAX, D_TERM_MAX);

    float out_y = (float)ex * kp + dx * kd;
    float out_z = (float)ey * kp + dy * kd;
    _last_ex = ex;
    _last_ey = ey;

    _f_ty = constrain(_f_ty + out_y, TARGET_Y_MIN, TARGET_Y_MAX);
    _f_tz = constrain(_f_tz + out_z, TARGET_Z_MIN, TARGET_Z_MAX);

    unsigned long now = millis();
    if (now - last_move_cmd_time >= MOVE_INTERVAL) {
        arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, TRACK_MOVE_DURATION);
        last_move_cmd_time = now;
    }
}

void ColorTrackerRot::handleCenter(const uint8_t* payload, uint16_t plen) {
    static unsigned long last_move_cmd_time = 0;

    if (_currentState < CT_STATE_RUNNING) {
        _currentState = CT_STATE_RUNNING;
        _last_state_time = millis();
    }
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) return;
    if (off + 4 > plen) return;

    uint16_t cx = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off+1]; off += 2;

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

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
    float out_y = (float)ex * kp + dx * kd;
    float out_z = (float)ey * kp + dy * kd;
    _last_ex = ex; _last_ey = ey;

    _f_ty = constrain(_f_ty + out_y, TARGET_Y_MIN, TARGET_Y_MAX);
    _f_tz = constrain(_f_tz + out_z, TARGET_Z_MIN, TARGET_Z_MAX);

    unsigned long now = millis();
    if (now - last_move_cmd_time >= MOVE_INTERVAL) {
        arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, TRACK_MOVE_DURATION);
        last_move_cmd_time = now;
    }
}

// ── Frame parsing (v2 format) ──

void ColorTrackerRot::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8) return;
    if (frame[0] != FRAME_H0 || frame[1] != FRAME_H1) return;

    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) return;

    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= frame[i];
    if (xor_val != frame[7 + plen]) return;

    uint8_t ctrl = frame[4];
    uint8_t func = frame[5];
    const uint8_t* payload = &frame[7];
    uint8_t ftype = ctrl & 0xC0;

    if (ftype == CTRL_TYPE_RSP) {
        Serial.printf("[COLOR] RSP func=0x%02X plen=%d data:", func, plen);
        for (uint16_t i = 0; i < plen && i < 32; i++) Serial.printf(" %02X", payload[i]);
        Serial.println();
    }
    if (ftype == CTRL_TYPE_RPT) {
        static unsigned long last_rpt_log = 0;
        if (millis() - last_rpt_log > 500) {
            Serial.printf("[COLOR] RPT func=0x%02X plen=%d\n", func, plen);
            last_rpt_log = millis();
        }
        if (func == RPT_HEARTBEAT)         handleHeartbeat(payload, plen);
        else if (func == RPT_DETECT_COLOR) handleColor(payload, plen);
        else if (func == RPT_DETECT_CENTER) handleCenter(payload, plen);
    }
}

// ── Main update loop ──

void ColorTrackerRot::update() {
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
        case CT_STATE_SET_MODE: {
            uint8_t mode = APP_SINGLE_COLOR;
            if (sendCmd(CMD_SET_MODE, &mode, 1)) {
                _currentState = CT_STATE_WAIT_READY;
                _last_state_time = now;
                Serial.println("[COLOR] SET_MODE -> WAIT_READY");
            }
            break;
        }
        case CT_STATE_WAIT_READY:
            if (now - _last_poll_time > 100) {
                sendCmd(CMD_REQUEST_STATUS);
                _last_poll_time = now;
            }
            if (now - _last_state_time > 10000) {
                _currentState = CT_STATE_SET_MODE;
                _last_state_time = now;
                Serial.println("[COLOR] WAIT_READY timeout, retry");
            }
            break;

        case CT_STATE_CONFIG_PARAMS:
            if (!_cmd_sent) {
                forceConfig();
                _cmd_sent = true;
                _last_state_time = now;
            }
            if (now - _last_state_time > 200) {
                _currentState = CT_STATE_ENABLE_RUN;
                _last_state_time = now;
                _cmd_sent = false;
                Serial.println("[COLOR] -> ENABLE_RUN");
            }
            break;

        case CT_STATE_ENABLE_RUN: {
            if (now - _last_state_time > 100) {
                uint8_t run_en = 0;
                sendCmd(CMD_DISABLE_RUN, &run_en, 1);
                _currentState = CT_STATE_RUNNING;
                _last_state_time = now;
                Serial.println("[COLOR] ENABLE_RUN -> RUNNING");
            }
            break;
        }
        case CT_STATE_RUNNING:
            break;

        default:
            break;
    }
}



