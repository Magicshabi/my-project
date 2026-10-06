#include "ColorGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include <math.h>

// Protocol v2 constants
#define CMD_SET_MODE             0x01
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14
#define APP_EMPTY                0
#define CMD_COLOR_SET_TARGET     0x40

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_COLOR         0x75

#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define HB_READY                 0x04

#define I2C_POLL_INTERVAL        10
#define CENTER_DEADBAND          15
#define CENTER_RELEASE_BAND      25
#define D_TERM_MAX               40.0f
#define MOVE_INTERVAL            40
#define TRACK_MOVE_DURATION      30

static const uint8_t CG_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

ColorGrabber colorGrabber;

// ── I2C low-level ──

void ColorGrabber::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void ColorGrabber::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

// ── Mailbox v2 ──

bool ColorGrabber::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, CG_MAILBOX_MAGIC, 4) != 0) return false;
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) return false;
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[CGRAB] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

ColorGrabber::SlotMetaV2 ColorGrabber::readSlotMeta(uint16_t offset) {
    uint8_t raw[8]; i2cRead16(offset, raw, 8);
    return { raw[0], raw[1], (uint16_t)((raw[2]<<8)|raw[3]),
             (uint16_t)((raw[4]<<8)|raw[5]), raw[6], raw[7] };
}

void ColorGrabber::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = { m.state, m.reserved0,
        (uint8_t)(m.generation>>8), (uint8_t)(m.generation&0xFF),
        (uint8_t)(m.frame_len>>8), (uint8_t)(m.frame_len&0xFF),
        m.frame_xor, m.reserved1 };
    i2cWrite16(offset, raw, 8);
}

bool ColorGrabber::writeHostSlot(const uint8_t* frame, uint16_t len) {
    if (len > _slot_size || _slot_size == 0) return false;
    SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
    if (hm.state != SLOT_EMPTY) return false;
    _host_gen++;
    SlotMetaV2 wm = { SLOT_WRITING, 0, _host_gen, 0, 0, 0 };
    writeSlotMeta(HOST_SLOT_META_OFFSET, wm);
    i2cWrite16(_host_slot_data_offset, frame, len);
    uint8_t xv = 0;
    for (uint16_t i = 0; i < len; i++) xv ^= frame[i];
    SlotMetaV2 rm = { SLOT_READY, 0, _host_gen, len, xv, 0 };
    writeSlotMeta(HOST_SLOT_META_OFFSET, rm);
    return true;
}

int ColorGrabber::readDevSlot(uint8_t* buf, uint16_t bufSize) {
    SlotMetaV2 dm = readSlotMeta(DEV_SLOT_META_OFFSET);
    if (dm.state != SLOT_READY || dm.generation == _dev_gen) return 0;
    if (dm.frame_len == 0 || dm.frame_len > bufSize) {
        _dev_gen = dm.generation;
        SlotMetaV2 em = { SLOT_EMPTY, 0, dm.generation, 0, 0, 0 };
        writeSlotMeta(DEV_SLOT_META_OFFSET, em);
        return 0;
    }
    i2cRead16(_dev_slot_data_offset, buf, dm.frame_len);
    uint8_t xv = 0;
    for (uint16_t i = 0; i < dm.frame_len; i++) xv ^= buf[i];
    _dev_gen = dm.generation;
    SlotMetaV2 em = { SLOT_EMPTY, 0, dm.generation, 0, 0, 0 };
    writeSlotMeta(DEV_SLOT_META_OFFSET, em);
    if (xv != dm.frame_xor) return 0;
    return dm.frame_len;
}

// ── Frame build / send ──

uint8_t ColorGrabber::nextTxn() { _txn++; if (!_txn) _txn = 1; return _txn; }

uint16_t ColorGrabber::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn) {
    buf[0] = FRAME_H0; buf[1] = FRAME_H1;
    buf[2] = (plen >> 8); buf[3] = (plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD;
    buf[5] = func;
    buf[6] = txn ? txn : nextTxn();
    if (payload && plen) memcpy(&buf[7], payload, plen);
    uint8_t xv = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xv ^= buf[i];
    buf[7 + plen] = xv;
    return 8 + plen;
}

bool ColorGrabber::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[128];
    uint16_t flen = buildFrame(frame, func, payload, plen);
    return writeHostSlot(frame, flen);
}

bool ColorGrabber::safeSend(uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label) {
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
    Serial.printf("[CGRAB] %s ok=%d\n", label, ok);
    for (int i = 0; i < 20; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        delay(10);
    }
    readDevSlot(tmp, sizeof(tmp));
    delay(20);
    return ok;
}

// ── Send color target ──

void ColorGrabber::sendColorTarget() {
    uint8_t buf[32];
    uint8_t n = _current_color.length();
    if (n > 30) n = 30;
    buf[0] = n;
    memcpy(&buf[1], _current_color.c_str(), n);
    safeSend(CMD_COLOR_SET_TARGET, buf, n + 1, "SET_COLOR");
}

// ── Lifecycle ──

void ColorGrabber::begin() {
    _busy = false;
    _txn = 0;
    Wire1.setClock(400000);
}

void ColorGrabber::start(const char* colors[], uint8_t count) {
    if (_busy) stop();

    _color_count = (count > 8) ? 8 : count;
    _color_index = 0;
    _stack_count = 0;
    for (uint8_t i = 0; i < _color_count; i++)
        _color_list[i] = String(colors[i]);
    _current_color = _color_list[0];

    _busy = true;
    _txn = 0;
    _host_gen = 0; _dev_gen = 0;
    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z;
    _last_ex = 0; _last_ey = 0;
    _center_locked = false;
    _stableCount = 0;

    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
    delay(1500);

    if (!initMailbox()) {
        Serial.println("[CGRAB] mailbox init fail");
        _busy = false;
        return;
    }

    // 先确保 K230 处于空闲状态（上一个追踪器可能没完全退出）
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){1}, 1, "PRE_DISABLE");
    delay(50);
    safeSend(CMD_SET_MODE, (const uint8_t[]){0}, 1, "PRE_EMPTY");
    delay(100);

    // Blocking: set mode and wait for heartbeat
    safeSend(CMD_SET_MODE, (const uint8_t[]){APP_SINGLE_COLOR}, 1, "SET_MODE=21");

    bool mode_ok = false;
    for (int r = 0; r < 150; r++) {
        delay(100);
        uint8_t buf[MAX_FRAME_SIZE];
        int got = readDevSlot(buf, sizeof(buf));
        if (got >= 8 && buf[0] == FRAME_H0 && buf[1] == FRAME_H1) {
            uint8_t ftype = buf[4] & 0xC0;
            uint8_t func = buf[5];
            if (ftype == CTRL_TYPE_RPT && func == RPT_HEARTBEAT) {
                uint16_t pl = (buf[2] << 8) | buf[3];
                if (pl >= 2) {
                    uint8_t m = buf[7], s = buf[8];
                    Serial.printf("[CGRAB] HB mode=%d status=0x%02X\n", m, s);
                    if (m == APP_SINGLE_COLOR && (s & HB_READY)) {
                        mode_ok = true; break;
                    }
                }
            }
        }
    }
    if (!mode_ok) Serial.println("[CGRAB] mode not confirmed");
    else Serial.println("[CGRAB] Mode confirmed!");

    // Configure
    safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
    safeSend(CMD_SET_CONF_THRESH, (const uint8_t[]){50}, 1, "SET_THRESH=50");
    sendColorTarget();
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");

    _currentState = CG_STATE_SEARCH;
    _last_state_time = millis();
    _last_i2c_time = millis();
    _ignore_data_until = millis() + 1000;
    Serial.printf("[CGRAB] -> SEARCH color='%s' (%d total)\n",
                  _current_color.c_str(), _color_count);
}

void ColorGrabber::stop() {
    if (!_busy) return;
    _busy = false;
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){1}, 1, "DISABLE_RUN");
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);
    arm.move(200, 0, 200, -90, 0, C_OPEN, 1500);
    Serial.println("[CGRAB] stop");
}

void ColorGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

// ── Heartbeat → state machine ──

void ColorGrabber::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    runStateMachine(payload[0], payload[1]);
}

void ColorGrabber::runStateMachine(uint8_t mode, uint8_t status) {
    unsigned long now = millis();

    switch (_currentState) {
        case CG_STATE_SET_MODE:
            if (!_cmd_sent) {
                uint8_t m = APP_SINGLE_COLOR;
                sendCmd(CMD_SET_MODE, &m, 1);
                _cmd_sent = true;
                _last_state_time = now;
                Serial.println("[CGRAB] SET_MODE=21 sent");
            }
            if (now - _last_state_time > 1000) {
                _currentState = CG_STATE_WAIT_READY;
                _last_state_time = now;
            }
            break;

        case CG_STATE_WAIT_READY:
            if (mode == APP_SINGLE_COLOR && (status & HB_READY)) {
                Serial.printf("[CGRAB] HB mode=%d ready\n", mode);
                _currentState = CG_STATE_CONFIG;
                _last_state_time = now;
                _cmd_sent = false;
            } else if (now - _last_state_time > 15000) {
                Serial.println("[CGRAB] timeout, forcing config");
                _currentState = CG_STATE_CONFIG;
                _last_state_time = now;
                _cmd_sent = false;
            }
            break;

        case CG_STATE_CONFIG:
            if (!_cmd_sent) {
                safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
                safeSend(CMD_SET_CONF_THRESH, (const uint8_t[]){50}, 1, "SET_THRESH=50");
                safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");
                sendColorTarget();
                _cmd_sent = true;
                _currentState = CG_STATE_SEARCH;
                _last_state_time = now;
                _ignore_data_until = millis() + 1000;
                Serial.printf("[CGRAB] -> SEARCH color='%s'\n", _current_color.c_str());
            }
            break;

        default:
            break;
    }
}

// ── Color detection → PD tracking ──

void ColorGrabber::handleColor(const uint8_t* payload, uint16_t plen) {
    if (_currentState != CG_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    // RPT_DETECT_COLOR (0x75) format:
    // [group_count:u8] + group_count * ([color_name:string_u8][blob_count:u8] + blob_count * ([cx:u16][cy:u16][w:u16][h:u16][angle:s16]))
    uint16_t off = 0;
    uint8_t group_count = payload[off++];
    if (group_count == 0) { _last_ex = 0; _last_ey = 0; _stableCount = 0; return; }

    // Find matching color group
    bool found = false;
    uint16_t cx = 0, cy = 0, bw = 0, bh = 0;

    for (uint8_t g = 0; g < group_count && off < plen; g++) {
        // Read color_name (string_u8: [len:u8][chars])
        if (off >= plen) break;
        uint8_t name_len = payload[off++];
        if (off + name_len > plen) break;
        char name[32] = {0};
        uint8_t nl = (name_len > 31) ? 31 : name_len;
        memcpy(name, &payload[off], nl);
        off += name_len;

        // Read blob_count
        if (off >= plen) break;
        uint8_t blob_count = payload[off++];

        // Check if this is our target color
        bool match = (_current_color.equalsIgnoreCase(name));

        for (uint8_t b = 0; b < blob_count && off + 10 <= plen; b++) {
            uint16_t bcx = (payload[off]<<8)|payload[off+1]; off += 2;
            uint16_t bcy = (payload[off]<<8)|payload[off+1]; off += 2;
            uint16_t bw_ = (payload[off]<<8)|payload[off+1]; off += 2;
            uint16_t bh_ = (payload[off]<<8)|payload[off+1]; off += 2;
            int16_t  ang = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;

            if (match && !found && bw_ < 280 && bh_ < 200) {
                cx = bcx; cy = bcy; bw = bw_; bh = bh_;
                found = true;
            }
        }
    }

    if (!found) { _last_ex = 0; _last_ey = 0; _stableCount = 0; return; }

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    unsigned long now = millis();

    // Centered → stable timer
    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        _center_locked = true;
        _last_ex = 0; _last_ey = 0;
        if (_stableCount == 0) _stable_start_time = now;
        _stableCount++;
        if (now - _stable_start_time >= 3000) {
            _grab_x = _f_tx;
            _grab_y = _f_ty;
            Serial.printf("[CGRAB] Stable 3s! x=%.1f y=%.1f -> DOWN\n", _grab_x, _grab_y);
            _currentState = CG_STATE_DOWN;
            _last_state_time = now;
            _stableCount = 0;
        }
        return;
    }
    _stableCount = 0;

    if (_center_locked) {
        if (abs(ex) > CENTER_RELEASE_BAND || abs(ey) > CENTER_RELEASE_BAND)
            _center_locked = false;
        else return;
    }

    // PD tracking
    float dx = constrain((float)ex - _last_ex, -D_TERM_MAX, D_TERM_MAX);
    float dy = constrain((float)ey - _last_ey, -D_TERM_MAX, D_TERM_MAX);
    float out_x = (float)ey * kp + dy * kd;
    float out_y = (float)ex * kp + dx * kd;
    _last_ex = (float)ex; _last_ey = (float)ey;

    _f_tx = constrain(_f_tx + out_x, S_X - 120.0f, S_X + 120.0f);
    _f_ty = constrain(_f_ty + out_y, -300.0f, 300.0f);

    static unsigned long last_move = 0;
    if (now - last_move >= MOVE_INTERVAL) {
        arm.move(_f_tx, _f_ty, S_Z, S_P, 0, C_OPEN, TRACK_MOVE_DURATION);
        last_move = now;
    }
}

// ── Frame parsing ──

void ColorGrabber::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8 || frame[0] != FRAME_H0 || frame[1] != FRAME_H1) return;
    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) return;

    uint8_t ftype = frame[4] & 0xC0;
    uint8_t func = frame[5];
    const uint8_t* payload = &frame[7];

    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_HEARTBEAT)         handleHeartbeat(payload, plen);
        else if (func == RPT_DETECT_COLOR) handleColor(payload, plen);
        else if (func == RPT_DETECT_BBOX && plen >= 1 && payload[0] == 0) {
            // Empty bbox = no detection, reset stable count
            if (_currentState == CG_STATE_SEARCH) {
                _last_ex = 0; _last_ey = 0; _stableCount = 0;
            }
        }
    }
}

// ── Grab state machine ──

void ColorGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();

    // Grab states
    if (_currentState >= CG_STATE_DOWN && _currentState <= CG_STATE_RESET) {
        unsigned long elapsed = now - _last_state_time;

        switch (_currentState) {
            case CG_STATE_DOWN:
                if (elapsed > 100) {
                    // 旋转补偿：和 AprilTagGrabber 一致
                    float base_rad = atan2f(_grab_y, _grab_x);
                    float cos_b = cosf(base_rad);
                    float sin_b = sinf(base_rad);
                    float rx = X_COMP * cos_b - Y_COMP * sin_b;
                    float ry = X_COMP * sin_b + Y_COMP * cos_b;
                    float gx = _grab_x + rx;
                    float gy = _grab_y + ry;
                    Serial.printf("[CGRAB] DOWN raw=(%.1f,%.1f) base=%.1f° comp=(%.1f,%.1f) -> gx=%.1f gy=%.1f gz=%.1f\n",
                                  _grab_x, _grab_y, base_rad * 180.0f / M_PI, rx, ry, gx, gy, GRAB_Z);
                    arm.move(gx, gy, GRAB_Z, S_P, 0, C_OPEN, 1000);
                    _currentState = CG_STATE_GRAB;
                    _last_state_time = now;
                }
                break;

            case CG_STATE_GRAB:
                if (elapsed > 2500) {
                    float gx = _grab_x + X_COMP;
                    float gy = _grab_y + Y_COMP;
                    Serial.println("[CGRAB] GRAB close");
                    arm.move(gx, gy, GRAB_Z, S_P, 0, C_CLOSE, 500);
                    _currentState = CG_STATE_LIFT;
                    _last_state_time = now;
                }
                break;

            case CG_STATE_LIFT:
                if (elapsed > 800) {
                    Serial.println("[CGRAB] LIFT");
                    arm.move(_grab_x + X_COMP, _grab_y + Y_COMP, LIFT_Z, S_P, 0, C_CLOSE, 1000);
                    _currentState = CG_STATE_PLACE;
                    _last_state_time = now;
                }
                break;

            case CG_STATE_PLACE:
                if (elapsed > 1500) {
                    float pz = PLACE_Z + _stack_count * BLOCK_H;
                    Serial.printf("[CGRAB] PLACE #%d z=%.1f\n", _stack_count, pz);
                    arm.move(PLACE_X, PLACE_Y, pz, S_P, 0, C_CLOSE, 1500);
                    _currentState = CG_STATE_RELEASE;
                    _last_state_time = now;
                }
                break;

            case CG_STATE_RELEASE:
                if (elapsed > 1800) {
                    float rz = PLACE_Z + _stack_count * BLOCK_H + 40;
                    Serial.println("[CGRAB] RELEASE");
                    arm.move(PLACE_X, PLACE_Y, rz, S_P, 0, C_OPEN, 500);
                    _stack_count++;
                    _currentState = CG_STATE_RESET;
                    _last_state_time = now;
                }
                break;

            case CG_STATE_RESET:
                if (elapsed > 1000) {
                    _color_index++;
                    if (_color_index < _color_count) {
                        // Next color
                        _current_color = _color_list[_color_index];
                        Serial.printf("[CGRAB] Next: '%s' (%d/%d)\n",
                                      _current_color.c_str(), _color_index + 1, _color_count);
                        sendColorTarget();

                        arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
                        _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z;
                        _last_ex = 0; _last_ey = 0;
                        _stableCount = 0;
                        _center_locked = false;
                        _ignore_data_until = millis() + 1500;
                        _currentState = CG_STATE_SEARCH;
                        _last_state_time = millis();
                    } else {
                        Serial.printf("[CGRAB] Done! %d blocks stacked\n", _stack_count);
                        arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
                        // arm.move(S_X, S_Y, S_Z, , 0, C_OPEN, 1000);
                        _busy = false;
                    }
                }
                break;

            default: break;
        }
        return;
    }

    // Poll I2C
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) parseFrame(buf, (uint16_t)got);
}
