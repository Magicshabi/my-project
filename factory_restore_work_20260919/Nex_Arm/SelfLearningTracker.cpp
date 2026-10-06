#include "SelfLearningTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_DISABLE_RUN          0x14
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_COLOR_LEARNING_SET_POINT 0x47
#define CMD_SELFLEARN_SET_RECT   0x56

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72

#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define APP_SELF_LEARNING        31
#define APP_EMPTY                0
#define I2C_POLL_INTERVAL        10

static const uint8_t SL_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

SelfLearningTracker selfLearningTracker;

// ── I2C low-level ──

void SelfLearningTracker::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void SelfLearningTracker::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

SelfLearningTracker::SlotMetaV2 SelfLearningTracker::readSlotMeta(uint16_t offset) {
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

void SelfLearningTracker::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool SelfLearningTracker::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, SL_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[SELFLEARN] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[SELFLEARN] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[SELFLEARN] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool SelfLearningTracker::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int SelfLearningTracker::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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

uint8_t SelfLearningTracker::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t SelfLearningTracker::buildFrame(uint8_t* buf, uint8_t func,
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

bool SelfLearningTracker::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void SelfLearningTracker::begin() {
    _busy = false;
    _is_tracking = false;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;
    _last_ex = 0;
    _last_ey = 0;
    _f_tx = 0;
    _f_tz = 200;
    Wire1.setClock(400000);
}

void SelfLearningTracker::start() {
    _busy = true;
    _is_tracking = false;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;

    Serial.println("[SELFLEARN] Step 1: Mode 24 (ColorPicking) Init...");

    if (!initMailbox()) {
        Serial.println("[SELFLEARN] mailbox init fail, abort");
        _busy = false;
        return;
    }

    uint8_t mode = APP_SELF_LEARNING;
    sendCmd(CMD_SET_MODE, &mode, 1);
    delay(800);

    // 显示引导矩形框
    uint8_t rect_data[8];
    uint16_t rx = 150, ry = 110, rw = 20, rh = 20;
    rect_data[0] = (rx >> 8) & 0xFF; rect_data[1] = rx & 0xFF;
    rect_data[2] = (ry >> 8) & 0xFF; rect_data[3] = ry & 0xFF;
    rect_data[4] = (rw >> 8) & 0xFF; rect_data[5] = rw & 0xFF;
    rect_data[6] = (rh >> 8) & 0xFF; rect_data[7] = rh & 0xFF;
    sendCmd(CMD_SELFLEARN_SET_RECT, rect_data, 8);
    delay(50);

    uint8_t simple_off = 0;
    sendCmd(CMD_SET_SIMPLE_RESULT, &simple_off, 1);
    delay(30);
    uint8_t run_en = 0;
    sendCmd(CMD_DISABLE_RUN, &run_en, 1);

    _last_i2c_time = millis();
    _last_poll_time = millis();

    arm.move(0, 200, 200, 0, 0, 0, 0);
    Serial.println("[SELFLEARN] Guide box ready. Move object into the box.");
}

void SelfLearningTracker::confirm(String targetID, int x, int y) {
    if (!_busy) return;
    _currentID = targetID;

    // CMD_COLOR_LEARNING_SET_POINT (0x47): X(2) + Y(2) + NameLen(1) + Name
    uint8_t payload[32];
    payload[0] = (x >> 8) & 0xFF;
    payload[1] = x & 0xFF;
    payload[2] = (y >> 8) & 0xFF;
    payload[3] = y & 0xFF;
    uint8_t n_len = _currentID.length();
    payload[4] = n_len;
    memcpy(&payload[5], _currentID.c_str(), n_len);

    // Stop previous tracking via DISABLE_RUN(val=1)
    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(50);

    sendCmd(CMD_COLOR_LEARNING_SET_POINT, payload, 5 + n_len);
    delay(50);

    uint8_t run_en = 0;
    sendCmd(CMD_DISABLE_RUN, &run_en, 1);

    _is_tracking = true;
    _last_ex = 0; _last_ey = 0;
    Serial.printf("[SELFLEARN] Pick Color at %d,%d ID:%s\n", x, y, _currentID.c_str());
}

void SelfLearningTracker::stop() {
    if (!_busy) return;
    _busy = false;
    _is_tracking = false;

    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);

    arm.move(200, 0, 200, 0, 0, 0, 1000);
    Serial.println("[SELFLEARN] Stopped.");
}

// ── BBOX result handling (SelfLearning mode) ──

void SelfLearningTracker::handleColor(const uint8_t* payload, uint16_t plen) {
    if (!_is_tracking) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) return;

    int best_idx = -1;
    uint16_t best_cx = 0, best_cy = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (off + 8 > plen) break;
        uint16_t cx = (payload[off]<<8)|payload[off+1]; off += 2;
        uint16_t cy = (payload[off]<<8)|payload[off+1]; off += 2;
        uint16_t bw = (payload[off]<<8)|payload[off+1]; off += 2;
        uint16_t bh = (payload[off]<<8)|payload[off+1]; off += 2;

        char name[32] = {0};
        if (off < plen) {
            uint8_t extra_count = payload[off++];
            for (uint8_t e = 0; e < extra_count && off < plen; e++) {
                uint8_t etype = payload[off++];
                if (etype == 1 && off < plen) {
                    uint8_t slen = payload[off++];
                    if (slen > 0 && off + slen <= plen) {
                        uint8_t cplen = slen < 31 ? slen : 31;
                        memcpy(name, &payload[off], cplen);
                        name[cplen] = '\0';
                    }
                    off += slen;
                } else if ((etype == 2 || etype == 3) && off + 2 <= plen) {
                    off += 2;
                }
            }
        }

        if (best_idx < 0) {
            if (_currentID == "1" || String(name) == _currentID) {
                best_idx = i;
                best_cx = cx;
                best_cy = cy;
            }
        }
    }

    if (best_idx < 0) return;

    int ex = 160 - (int)best_cx;
    int ey = 120 - (int)best_cy;

    float ox = (float)ex * kp + (float)(ex - _last_ex) * kd;
    float oz = (float)ey * kp + (float)(ey - _last_ey) * kd;
    _last_ex = ex;
    _last_ey = ey;

    _f_tx -= ox;
    _f_tz -= oz;
    _f_tx = constrain(_f_tx, -160.0f, 160.0f);
    _f_tz = constrain(_f_tz, 60.0f, 320.0f);

    if (abs(ex) > 2 || abs(ey) > 2) {
        arm.move(_f_tx, 200, _f_tz, 0, 0, 0, 0);
    }
}

// ── Frame parsing ──

void SelfLearningTracker::parseFrame(const uint8_t* frame, uint16_t len) {
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
        if (func == RPT_DETECT_BBOX) handleColor(payload, plen);
    }
}

// ── Main update loop ──

void SelfLearningTracker::update() {
    if (!_busy || !_is_tracking) return;

    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) parseFrame(buf, (uint16_t)got);

    if (now - _last_poll_time > 50) {
        // Don't send CMD_REQUEST_STATUS during RUNNING, RSP overwrites detection data
        _last_poll_time = now;
    }
}
