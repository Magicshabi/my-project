#include "CalibrationGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "usb_ctrl.h"
#include "Espnow_ctrl.h"

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

#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define HB_RUN                   0x01
#define HB_READY                 0x04

#define I2C_POLL_INTERVAL        10

static const uint8_t CB_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

CalibrationGrabber calibrationGrabber;

// ── I2C low-level (mailbox v2) ──

void CalibrationGrabber::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
    const uint16_t CHUNK = 30;
    for (uint16_t off = 0; off < len; off += CHUNK) {
        uint16_t n = min((uint16_t)CHUNK, (uint16_t)(len - off));
        uint16_t addr = memAddr + off;
        Wire1.beginTransmission(K230_ADDR);
        Wire1.write((uint8_t)(addr >> 8));
        Wire1.write((uint8_t)(addr & 0xFF));
        Wire1.write(data + off, n);
        Wire1.endTransmission();
    }
}

void CalibrationGrabber::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
    const uint16_t CHUNK = 30;
    for (uint16_t off = 0; off < len; off += CHUNK) {
        uint16_t n = min((uint16_t)CHUNK, (uint16_t)(len - off));
        uint16_t addr = memAddr + off;
        Wire1.beginTransmission(K230_ADDR);
        Wire1.write((uint8_t)(addr >> 8));
        Wire1.write((uint8_t)(addr & 0xFF));
        Wire1.endTransmission(false);
        Wire1.requestFrom(K230_ADDR, (uint8_t)n);
        for (uint16_t i = 0; i < n && Wire1.available(); i++)
            data[off + i] = Wire1.read();
    }
}

bool CalibrationGrabber::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, CB_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[Calib] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) return false;
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[Calib] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

CalibrationGrabber::SlotMetaV2 CalibrationGrabber::readSlotMeta(uint16_t offset) {
    uint8_t raw[8];
    i2cRead16(offset, raw, 8);
    return { raw[0], raw[1], (uint16_t)((raw[2]<<8)|raw[3]),
             (uint16_t)((raw[4]<<8)|raw[5]), raw[6], raw[7] };
}

void CalibrationGrabber::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = { m.state, m.reserved0,
        (uint8_t)(m.generation>>8), (uint8_t)(m.generation&0xFF),
        (uint8_t)(m.frame_len>>8), (uint8_t)(m.frame_len&0xFF),
        m.frame_xor, m.reserved1 };
    i2cWrite16(offset, raw, 8);
}

bool CalibrationGrabber::writeHostSlot(const uint8_t* frame, uint16_t len) {
    if (len > _slot_size || _slot_size == 0) return false;
    SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
    if (hm.state != SLOT_EMPTY) return false;
    _host_gen++;
    SlotMetaV2 wm = { SLOT_WRITING, 0, _host_gen, 0, 0, 0 };
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    i2cWrite16(_host_slot_data_offset, frame, len);
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) xor_val ^= frame[i];
    SlotMetaV2 rm = { SLOT_READY, 0, _host_gen, len, xor_val, 0 };
    writeSlotMeta(HOST_SLOT_META_OFFSET, rm);
    return true;
}

int CalibrationGrabber::readDevSlot(uint8_t* buf, uint16_t bufSize) {
    SlotMetaV2 dm = readSlotMeta(DEV_SLOT_META_OFFSET);
    if (dm.state != SLOT_READY) return 0;
    if (dm.generation == _dev_gen) return 0;
    if (dm.frame_len == 0 || dm.frame_len > bufSize) {
        _dev_gen = dm.generation;
        SlotMetaV2 em = { SLOT_EMPTY, 0, dm.generation, 0, 0, 0 };
        writeSlotMeta(DEV_SLOT_META_OFFSET, em);
        return 0;
    }
    i2cRead16(_dev_slot_data_offset, buf, dm.frame_len);
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < dm.frame_len; i++) xor_val ^= buf[i];
    _dev_gen = dm.generation;
    SlotMetaV2 em = { SLOT_EMPTY, 0, dm.generation, 0, 0, 0 };
    writeSlotMeta(DEV_SLOT_META_OFFSET, em);
    if (xor_val != dm.frame_xor) return 0;
    return dm.frame_len;
}

// ── Protocol v2 frame ──

uint8_t CalibrationGrabber::nextTxn() { _txn++; if (_txn == 0) _txn = 1; return _txn; }

uint16_t CalibrationGrabber::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn) {
    buf[0] = FRAME_H0; buf[1] = FRAME_H1;
    buf[2] = (plen >> 8); buf[3] = (plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD;
    buf[5] = func;
    buf[6] = txn ? txn : nextTxn();
    if (payload && plen > 0) memcpy(&buf[7], payload, plen);
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= buf[i];
    buf[7 + plen] = xor_val;
    return 8 + plen;
}

bool CalibrationGrabber::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[128];
    uint16_t flen = buildFrame(frame, func, payload, plen);
    return writeHostSlot(frame, flen);
}

bool CalibrationGrabber::safeSend(uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label) {
    uint8_t tmp[MAX_FRAME_SIZE];
    readDevSlot(tmp, sizeof(tmp));
    delay(50);
    for (int i = 0; i < 20; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        readDevSlot(tmp, sizeof(tmp));
        delay(50);
    }
    bool ok = sendCmd(cmd, payload, plen);
    Serial.printf("[Calib] %s ok=%d\n", label, ok);
    for (int i = 0; i < 20; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        delay(10);
    }
    readDevSlot(tmp, sizeof(tmp));
    delay(20);
    return ok;
}

// ── Lifecycle ──

void CalibrationGrabber::begin() {
    _busy = false;
    _txn = 0;
    Wire1.setClock(400000);
}

void CalibrationGrabber::start() {
    if (_busy) {
        // Already running, just reset to search position
        arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
        _currentState = CB_STATE_SEARCH;
        _last_state_time = millis();
        _ignore_data_until = millis() + 500;
        Serial.println("[Calib] restart -> SEARCH");
        return;
    }
    _busy = true;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;

    // Move to search position (looking down)
    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
    _ignore_data_until = millis() + 1500;
    _last_i2c_time = millis();

    delay(500);
    if (!initMailbox()) {
        Serial.println("[Calib] mailbox init fail");
        _busy = false;
        return;
    }

    // Async state machine will handle mode switch
    _currentState = CB_STATE_SET_MODE;
    _last_state_time = millis();
    _cmd_sent = false;
    Serial.println("[Calib] start -> SET_MODE");
}

void CalibrationGrabber::grab() {
    if (!_busy || _currentState != CB_STATE_SEARCH) return;
    _currentState = CB_STATE_DOWN;
    _last_state_time = millis();
    Serial.println("[Calib] manual grab triggered");
}

void CalibrationGrabber::resetArm() {
    if (!_busy) return;
    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 2000);
    _currentState = CB_STATE_RESET;
    _last_state_time = millis();
    Serial.println("[Calib] resetArm");
}

void CalibrationGrabber::stop() {
    if (!_busy) return;
    _busy = false;
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){1}, 1, "DISABLE_RUN");
    arm.move(200, 0, 200, -90, 0, C_OPEN, 2000);
    Serial.println("[Calib] stop");
}

void CalibrationGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
    Serial.printf("[Calib] offsets x=%.1f y=%.1f z=%.1f\n", x_off, y_off, z_grab);
}

// ── Heartbeat → async state machine ──

void CalibrationGrabber::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    uint8_t mode = payload[0];
    uint8_t status = payload[1];
    runStateMachine(mode, status);
}

void CalibrationGrabber::runStateMachine(uint8_t mode, uint8_t status) {
    bool isReady = status & HB_READY;
    unsigned long now = millis();

    switch (_currentState) {
        case CB_STATE_SET_MODE:
            if (!_cmd_sent) {
                uint8_t m = APP_APRILTAG;
                sendCmd(CMD_SET_MODE, &m, 1);
                _cmd_sent = true;
                _last_state_time = now;
                Serial.println("[Calib] SET_MODE=32 sent");
            }
            if (now - _last_state_time > 1000) {
                _currentState = CB_STATE_WAIT_READY;
                _last_state_time = now;
            }
            break;

        case CB_STATE_WAIT_READY:
            if (mode == APP_APRILTAG && isReady) {
                _currentState = CB_STATE_CONFIG_PARAMS;
                _last_state_time = now;
                _cmd_sent = false;
                Serial.printf("[Calib] HB mode=%d ready -> CONFIG\n", mode);
            } else if (now - _last_state_time > 15000) {
                Serial.println("[Calib] timeout, forcing config");
                _currentState = CB_STATE_CONFIG_PARAMS;
                _last_state_time = now;
                _cmd_sent = false;
            }
            break;

        case CB_STATE_CONFIG_PARAMS:
            if (!_cmd_sent) {
                forceConfig();
                _cmd_sent = true;
                _last_state_time = now;
                // Go directly to SEARCH, don't wait for next heartbeat
                _currentState = CB_STATE_SEARCH;
                Serial.println("[Calib] -> SEARCH (ready for calibration)");
            }
            break;

        default:
            break;
    }
}

void CalibrationGrabber::forceConfig() {
    safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
    safeSend(CMD_SET_CONF_THRESH, (const uint8_t[]){5}, 1, "SET_THRESH=5");
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");
}

// ── BBOX handler (只显示偏差，不追踪) ──

void CalibrationGrabber::handleBbox(const uint8_t* payload, uint16_t plen) {
    if (_currentState != CB_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0 || off + 8 > plen) return;

    uint16_t cx = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t cy = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bw = (payload[off]<<8)|payload[off+1]; off += 2;
    uint16_t bh = (payload[off]<<8)|payload[off+1]; off += 2;

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    // 只显示偏差，不追踪
    unsigned long now = millis();
    if (now - _last_print_time > 500) {
        char msg_buf[256];
        int msg_len = 0;

        if (abs(ex) < 5 && abs(ey) < 5) {
            msg_len = snprintf(msg_buf, sizeof(msg_buf),
                "[校准] 完美对准中心！(X偏:%d Y偏:%d) -> 发送 02 指令下落测试\r\n", ex, ey);
        } else {
            String str_x = (ex > 0) ? "目标偏左" : (ex < 0 ? "目标偏右" : "X已居中");
            String str_y = (ey > 0) ? "目标偏前" : (ey < 0 ? "目标偏后" : "Y已居中");
            msg_len = snprintf(msg_buf, sizeof(msg_buf),
                "[校准] %s %d 像素 | %s %d 像素\r\n",
                str_x.c_str(), abs(ex), str_y.c_str(), abs(ey));
        }
        Serial.print(msg_buf);
        espnow_ctrl.sendData((uint8_t*)msg_buf, msg_len);
        _last_print_time = now;
    }
}

// ── QUAD → BBOX conversion ──

void CalibrationGrabber::handleQuad(const uint8_t* payload, uint16_t plen) {
    if (_currentState != CB_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0 || off + 16 > plen) return;

    int16_t qx[4], qy[4];
    for (uint8_t i = 0; i < 4; i++) {
        qx[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
        qy[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
    }

    int16_t min_x = qx[0], max_x = qx[0], min_y = qy[0], max_y = qy[0];
    for (uint8_t i = 1; i < 4; i++) {
        if (qx[i] < min_x) min_x = qx[i];
        if (qx[i] > max_x) max_x = qx[i];
        if (qy[i] < min_y) min_y = qy[i];
        if (qy[i] > max_y) max_y = qy[i];
    }

    // Compute center from corners
    uint16_t cx = (uint16_t)((min_x + max_x) / 2);
    uint16_t cy = (uint16_t)((min_y + max_y) / 2);
    uint16_t bw = (uint16_t)(max_x - min_x);
    uint16_t bh = (uint16_t)(max_y - min_y);

    uint8_t fake[16];
    uint16_t fp = 0;
    fake[fp++] = 1;
    fake[fp++] = (cx>>8); fake[fp++] = (cx&0xFF);
    fake[fp++] = (cy>>8); fake[fp++] = (cy&0xFF);
    fake[fp++] = (bw>>8); fake[fp++] = (bw&0xFF);
    fake[fp++] = (bh>>8); fake[fp++] = (bh&0xFF);
    handleBbox(fake, fp);
}

// ── Frame parsing (v2 format) ──

void CalibrationGrabber::parseFrame(const uint8_t* frame, uint16_t len) {
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

    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_HEARTBEAT)        handleHeartbeat(payload, plen);
        else if (func == RPT_DETECT_BBOX) handleBbox(payload, plen);
        else if (func == RPT_DETECT_QUAD) handleQuad(payload, plen);
    }
}

// ── Grab state machine (手动下落测试) ──

void CalibrationGrabber::updateGrabStateMachine() {
    unsigned long now = millis();
    switch (_currentState) {
        case CB_STATE_DOWN:
            if (now - _last_state_time > 100) {
                float gx = S_X + X_COMP;
                float gy = S_Y + Y_COMP;
                Serial.printf("[Calib] DOWN gx=%.1f gy=%.1f gz=%.1f\n", gx, gy, GRAB_Z);
                arm.move(gx, gy, GRAB_Z, S_P, 0, C_OPEN, 1000);
                _currentState = CB_STATE_HOLD;
                _last_state_time = now;
            }
            break;

        case CB_STATE_HOLD:
            // Wait for resetArm() call
            break;

        case CB_STATE_RELEASE:
            if (now - _last_state_time > 100) {
                arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
                _currentState = CB_STATE_RESET;
                _last_state_time = now;
            }
            break;

        case CB_STATE_RESET:
            if (now - _last_state_time > 2000) {
                _ignore_data_until = millis() + 500;
                _currentState = CB_STATE_SEARCH;
                _last_state_time = now;
                Serial.println("[Calib] RESET -> SEARCH");
            }
            break;

        default:
            break;
    }
}

// ── Main update loop ──

void CalibrationGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();

    // Grab state machine
    if (_currentState >= CB_STATE_DOWN && _currentState <= CB_STATE_RESET) {
        updateGrabStateMachine();
        return;
    }

    // Poll I2C
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) parseFrame(buf, (uint16_t)got);
}
