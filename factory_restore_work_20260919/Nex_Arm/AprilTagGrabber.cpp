#include "AprilTagGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include <math.h>

// Protocol v2 constants
#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_QUAD          0x7B
#define RPT_DETECT_CENTER        0x79
#define APP_APRILTAG             32
#define APP_EMPTY                0

// Ctrl field masks
#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define HB_RUN                   0x01
#define HB_READY                 0x04

#define MOVE_INTERVAL            40
#define CENTER_DEADBAND          8
#define CENTER_RELEASE_BAND     8
#define CENTER_SLOW_BAND         30
#define D_TERM_MAX               40.0f
#define TRACK_MOVE_DURATION      50
#define X_SCREENY_COMP_GAIN      3.20f
#define Y_TRACK_COMP_GAIN        0.10f
#define Y_SCREENX_COMP_GAIN      0.20f
#define I2C_POLL_INTERVAL        10

static const uint8_t GRAB_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

AprilTagGrabber aprilTagGrabber;

// ── I2C low-level ──

void AprilTagGrabber::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void AprilTagGrabber::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

AprilTagGrabber::SlotMetaV2 AprilTagGrabber::readSlotMeta(uint16_t offset) {
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

void AprilTagGrabber::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool AprilTagGrabber::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    Serial.printf("[GRAB] mailbox header: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  header[0], header[1], header[2], header[3],
                  header[4], header[5], header[6], header[7]);
    if (memcmp(header, GRAB_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[GRAB] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[GRAB] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[GRAB] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool AprilTagGrabber::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int AprilTagGrabber::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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
        Serial.printf("[GRAB] dev slot xor fail: calc=%02X exp=%02X\n", xor_val, dm.frame_xor);
        return -1;
    }
    return (int)flen;
}

// ── Protocol v2 frame build ──

uint8_t AprilTagGrabber::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t AprilTagGrabber::buildFrame(uint8_t* buf, uint8_t func,
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

bool AprilTagGrabber::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void AprilTagGrabber::begin() {
    _busy = false;
    _center_locked = false;
    _grab_comp_locked = false;
    _txn = 0;
    _last_ex = 0; _last_ey = 0; _last_ew = 0;
    _grab_comp_ex = 0; _grab_comp_ey = 0;
    _grabOpenHoldSent = false;
    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z;
    _filtered_cx = 160.0f;
    _filtered_cy = 120.0f;
    _last_move_time = 0;
    _last_bbox_time = 0;
    _has_valid_bbox = false;
    _host_gen = 0; _dev_gen = 0;
    Wire1.setClock(400000);
}

void AprilTagGrabber::start() {
    if (_busy) return;
    _busy = true;
    _txn = 0;
    _stableCount = 0;
    _center_locked = false;
    _grab_comp_locked = false;
    _last_ex = 0; _last_ey = 0; _last_ew = 0;
    _grab_comp_ex = 0; _grab_comp_ey = 0;
    _grabOpenHoldSent = false;
    _host_gen = 0; _dev_gen = 0;

    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z;
    _filtered_cx = 160.0f;
    _filtered_cy = 120.0f;
    _last_move_time = 0;
    _last_bbox_time = 0;
    _has_valid_bbox = false;
    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
    delay(1500);
    Serial.printf("[GRAB] init pos: x=%.0f y=%.0f z=%.0f pitch=%.0f\n", S_X, S_Y, S_Z, S_P);

    if (!initMailbox()) {
        Serial.println("[GRAB] mailbox init fail, abort");
        _busy = false;
        return;
    }

    // Send SET_MODE and poll heartbeat to confirm
    sendCmd(CMD_SET_MODE, (const uint8_t[]){APP_APRILTAG}, 1);
    Serial.println("[GRAB] SET_MODE sent, waiting ready...");

    bool mode_ok = false;
    for (int retry = 0; retry < 150; retry++) {
        delay(100);
        uint8_t buf[MAX_FRAME_SIZE];
        int got = readDevSlot(buf, sizeof(buf));
        if (got > 0 && got >= 8) {
            if (buf[0] == FRAME_H0 && buf[1] == FRAME_H1) {
                uint8_t ftype = buf[4] & 0xC0;
                uint8_t func = buf[5];
                if (ftype == CTRL_TYPE_RPT && func == RPT_HEARTBEAT) {
                    uint16_t plen = (buf[2] << 8) | buf[3];
                    if (plen >= 2) {
                        uint8_t hb_mode = buf[7];
                        uint8_t hb_status = buf[8];
                        Serial.printf("[GRAB] HB mode=%d status=0x%02X\n", hb_mode, hb_status);
                        if (hb_mode == APP_APRILTAG && (hb_status & 0x04)) {
                            mode_ok = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    if (mode_ok) {
        Serial.println("[GRAB] Mode confirmed!");
    } else {
        Serial.println("[GRAB] WARNING: mode not confirmed");
    }

    delay(50);
    forceConfig();

    _ignore_data_until = millis() + 500;
    _currentState = G_STATE_SEARCH;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_i2c_time = millis();
    _cmd_sent = false;
    Serial.println("[GRAB] Start -> SEARCH");
}

void AprilTagGrabber::stop() {
    if (!_busy) return;
    _busy = false;
    _center_locked = false;
    _grab_comp_locked = false;
    _has_valid_bbox = false;
    _grabOpenHoldSent = false;
    _grab_comp_ex = 0;
    _grab_comp_ey = 0;
    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);
    Serial.println("[GRAB] Stop");
}

void AprilTagGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

void AprilTagGrabber::calcCompensatedGrabTarget(float& x, float& y) const {
    float base_rad = atan2f(_f_ty, _f_tx);
    float cos_b = cosf(base_rad);
    float sin_b = sinf(base_rad);
    float rx = X_COMP * cos_b - Y_COMP * sin_b;
    float ry = X_COMP * sin_b + Y_COMP * cos_b;

    Serial.printf("[GRAB] comp: base=%.1f° rx=%.1f ry=%.1f\n",
                  base_rad * 180.0f / M_PI, rx, ry);

    x = constrain(_f_tx + rx, S_X - 120.0f, S_X + 240.0f);
    y = constrain(_f_ty + ry, -500.0f, 500.0f);
}

void AprilTagGrabber::forceConfig() {
    auto safeSend = [this](uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label) {
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
        Serial.printf("[GRAB] %s ok=%d\n", label, ok);
        for (int i = 0; i < 20; i++) {
            SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
            if (hm.state == SLOT_EMPTY) break;
            delay(10);
        }
        readDevSlot(tmp, sizeof(tmp));
        delay(20);
    };

    safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
    safeSend(CMD_SET_CONF_THRESH, (const uint8_t[]){5}, 1, "SET_THRESH=5");
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");
    Serial.println("[GRAB] forceConfig done");
}

void AprilTagGrabber::handleBbox(const uint8_t* payload, uint16_t plen) {
    if (_currentState != G_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) {
        _has_valid_bbox = false;
        _last_ex = 0; _last_ey = 0; _last_ew = 0;
        return;
    }
    if (off + 8 > plen) return;

    uint16_t cx = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t bw = (payload[off] << 8) | payload[off + 1]; off += 2;
    uint16_t bh = (payload[off] << 8) | payload[off + 1]; off += 2;

    if (bw > 280 || bh > 200) return;

    int ex = 160 - (int)cx;
    int ey = 120 - (int)cy;

    unsigned long now = millis();

    if (_center_locked) {
        if (abs(ex) > CENTER_RELEASE_BAND || abs(ey) > CENTER_RELEASE_BAND) {
            _center_locked = false;
        } else {
            ex = 0; ey = 0;
        }
    }

    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        _center_locked = true;
        _last_ex = 0; _last_ey = 0;
        _last_ew = (float)(60 - (int)bw);
        if (_stableCount == 0) {
            _stable_start_time = now;
        }
        _stableCount++;
        // Stable for 3 seconds → execute grab sequence
        if (now - _stable_start_time >= 3000) {
            _grab_x = _f_tx;
            _grab_y = _f_ty;
            Serial.printf("[GRAB] Stable 3s! raw_tx=%.1f raw_ty=%.1f\n", _grab_x, _grab_y);
            calcCompensatedGrabTarget(_grab_x, _grab_y);
            Serial.printf("[GRAB] After comp: x=%.1f y=%.1f X_COMP=%.1f Y_COMP=%.1f wrist=%.1f\n",
                          _grab_x, _grab_y, X_COMP, Y_COMP, _tag_angle);
            _currentState = G_STATE_DOWN;
            _last_state_time = now;
            _stableCount = 0;
        }
        return;
    }
    _stableCount = 0;  // Reset if not centered

    float dx = constrain((float)ex - _last_ex, -D_TERM_MAX, D_TERM_MAX);
    float dy = constrain((float)ey - _last_ey, -D_TERM_MAX, D_TERM_MAX);

    // ey -> X axis (forward/back), ex -> Y axis (left/right)
    // X axis needs higher gain (perspective: 1 pixel = more mm when further away)
    float out_x = (float)ey * 0.08f + dy * 0.02f;
    float out_y = (float)ex * 0.08f + dx * 0.02f;

    _last_ex = (float)ex;
    _last_ey = (float)ey;
    _last_ew = (float)(60 - (int)bw);
    _stableCount = 0;
    _has_valid_bbox = true;
    _last_bbox_time = now;

    _f_tx = constrain(_f_tx + out_x, S_X - 120.0f, S_X + 120.0f);
    _f_ty = constrain(_f_ty + out_y, -300.0f, 300.0f);

    if (now - _last_move_time >= MOVE_INTERVAL) {
        arm.move(_f_tx, _f_ty, S_Z, S_P, 0, C_OPEN, TRACK_MOVE_DURATION);
        _last_move_time = now;
    }

    static unsigned long last_print = 0;
    if (now - last_print > 300) {
        Serial.printf("[GRAB] cx=%d cy=%d ex=%d ey=%d tx=%.0f ty=%.0f\n",
                      cx, cy, ex, ey, _f_tx, _f_ty);
        last_print = now;
    }
}

void AprilTagGrabber::handleQuad(const uint8_t* payload, uint16_t plen) {
    // QUAD format: [count:u8] + count * ([x0:s16][y0:s16]...[x3:s16][y3:s16] + [extra_count:u8] + extra)
    // Convert quad corners to bbox center/size and feed into existing tracking logic
    if (_currentState != G_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) {
        _has_valid_bbox = false;
        _last_ex = 0; _last_ey = 0; _last_ew = 0;
        return;
    }
    if (off + 16 > plen) return;  // 4 corners * 2 coords * 2 bytes = 16

    int16_t x[4], y[4];
    for (uint8_t i = 0; i < 4; i++) {
        x[i] = (int16_t)((payload[off] << 8) | payload[off+1]); off += 2;
        y[i] = (int16_t)((payload[off] << 8) | payload[off+1]); off += 2;
    }

    // Skip extra items
    if (off < plen) {
        uint8_t extra_count = payload[off++];
        for (uint8_t e = 0; e < extra_count && off < plen; e++) {
            uint8_t etype = payload[off++];
            if (etype == 1 && off < plen) { uint8_t slen = payload[off++]; off += slen; }
            else if ((etype == 2 || etype == 3) && off + 2 <= plen) { off += 2; }
        }
    }

    // Calculate tag rotation angle from edge (x0,y0)->(x1,y1)
    float raw_angle = atan2f((float)(y[1] - y[0]), (float)(x[1] - x[0])) * 180.0f / M_PI;
    // Map to ±45°: fold every 90° back into [-45, 45]
    float wrist = fmodf(raw_angle, 90.0f);
    if (wrist > 45.0f) wrist -= 90.0f;
    else if (wrist < -45.0f) wrist += 90.0f;

    // 相机跟底座一起转，raw_angle 是相对底座的角度
    // 底座角度 = atan2(ty, tx)，加上底座角度得到世界坐标系下标签的实际角度
    // 爪子也跟底座转，所以 roll = 世界角度 - 底座角度 = wrist + 底座角度 - 底座角度？
    // 不对——爪子 roll=0 时方向和底座一致，相机也和底座一致
    // 相机看到标签角度 wrist，就是标签相对底座（也就是相对爪子 roll=0）的角度
    // 所以 roll = wrist 就能让爪子对齐标签
    // 那 wrist 是世界坐标系角度，爪子跟底座转了 base_deg，roll = wrist - base_deg
    _tag_angle = -wrist;

    static unsigned long last_angle_log = 0;
    if (millis() - last_angle_log > 500) {
        Serial.printf("[GRAB] tag raw=%.1f° wrist=%.1f° servo=%d\n",
                      raw_angle, wrist, (int)(_tag_angle * 10.0f));
        last_angle_log = millis();
    }

    // Compute center and bounding box from 4 corners
    int16_t min_x = x[0], max_x = x[0], min_y = y[0], max_y = y[0];
    for (uint8_t i = 1; i < 4; i++) {
        if (x[i] < min_x) min_x = x[i];
        if (x[i] > max_x) max_x = x[i];
        if (y[i] < min_y) min_y = y[i];
        if (y[i] > max_y) max_y = y[i];
    }
    uint16_t bw = (uint16_t)(max_x - min_x);
    uint16_t bh = (uint16_t)(max_y - min_y);
    uint16_t cx = (uint16_t)((min_x + max_x) / 2);
    uint16_t cy = (uint16_t)((min_y + max_y) / 2);

    // Reuse handleBbox logic: build a fake bbox payload (center_x/center_y/w/h)
    uint8_t fake[9];
    fake[0] = 1;  // count
    fake[1] = (cx >> 8); fake[2] = (cx & 0xFF);
    fake[3] = (cy >> 8); fake[4] = (cy & 0xFF);
    fake[5] = (bw >> 8); fake[6] = (bw & 0xFF);
    fake[7] = (bh >> 8); fake[8] = (bh & 0xFF);
    handleBbox(fake, 9);
}

// ── Frame parsing (v2 format) ──

void AprilTagGrabber::parseFrame(const uint8_t* frame, uint16_t len) {
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
        if (func == RPT_DETECT_BBOX) handleBbox(payload, plen);
        else if (func == RPT_DETECT_QUAD) handleQuad(payload, plen);
    }
}

// ── Main update loop ──

void AprilTagGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    // Poll device slot
    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) {
        parseFrame(buf, (uint16_t)got);
    }

    switch (_currentState) {
        case G_STATE_SEARCH:
            // Just poll dev slot for detection data, no CMD_REQUEST_STATUS
            break;

        case G_STATE_DOWN:
        {
            Serial.printf("[GRAB] G_STATE_DOWN: _f_tx=%.1f _f_ty=%.1f (before comp)\n", _f_tx, _f_ty);
            _grab_x = _f_tx;
            _grab_y = _f_ty;
            calcCompensatedGrabTarget(_grab_x, _grab_y);
            Serial.printf("[GRAB] G_STATE_DOWN: grab_x=%.1f grab_y=%.1f roll=%.1f S_Z=%.1f\n",
                          _grab_x, _grab_y, _tag_angle, S_Z);

            // 在当前高度设好 roll，xyz 不变
            arm.move(_grab_x, _grab_y, S_Z, S_P, _tag_angle, C_OPEN, 500);
            _currentState = G_STATE_WRIST;
            _last_state_time = now;
            break;
        }

        case G_STATE_WRIST:
            if (now - _last_state_time > 1000) {
                Serial.printf("[GRAB] WRIST done -> DOWN grab_x=%.1f grab_y=%.1f GRAB_Z=%.1f roll=%.1f\n",
                              _grab_x, _grab_y, GRAB_Z, _tag_angle);
                arm.move(_grab_x, _grab_y, GRAB_Z, S_P, _tag_angle, C_OPEN, 1000);
                _currentState = G_STATE_GRAB;
                _grabOpenHoldSent = false;
                _last_state_time = now;
            }
            break;

        case G_STATE_GRAB:
            if (!_grabOpenHoldSent && now - _last_state_time > 2500) {
                // Hold position, keep claw open
                Serial.printf("[GRAB] GRAB hold open x=%.1f y=%.1f\n", _grab_x, _grab_y);
                arm.move(_grab_x, _grab_y, GRAB_Z, S_P, _tag_angle, C_OPEN, 500);
                _grabOpenHoldSent = true;
                _last_state_time = now;
            } else if (_grabOpenHoldSent && now - _last_state_time > 500) {
                // Close claw at locked position
                Serial.printf("[GRAB] GRAB claw close x=%.1f y=%.1f\n", _grab_x, _grab_y);
                arm.move(_grab_x, _grab_y, GRAB_Z, S_P, _tag_angle, C_CLOSE, 500);
                _grabOpenHoldSent = false;
                _currentState = G_STATE_LIFT;
                _last_state_time = now;
            }
            break;

        case G_STATE_LIFT:
            if (now - _last_state_time > 800) {
                // Lift straight up from locked position
                Serial.printf("[GRAB] LIFT x=%.1f y=%.1f\n", _grab_x, _grab_y);
                arm.move(_grab_x, _grab_y, LIFT_Z, S_P, _tag_angle, C_CLOSE, 800);
                _currentState = G_STATE_PLACE;
                _last_state_time = now;
            }
            break;

        case G_STATE_PLACE:
            if (now - _last_state_time > 2000) {
                Serial.println("[GRAB] PLACE");
                arm.move(S_X, P_Y, P_Z, S_P, 0, C_CLOSE, 1000);
                _currentState = G_STATE_RELEASE;
                _last_state_time = now;
            }
            break;

        case G_STATE_RELEASE:
            if (!_clawState) {
                if (now - _last_state_time > 2000) {
                    Serial.println("[GRAB] Wait");
                    arm.move(S_X, P_Y, P_Z, S_P, 0, C_OPEN, 800);
                    _last_state_time = now;
                    _clawState = true;
                }
            } else {
                if (now - _last_state_time > 1000) {
                    Serial.println("[GRAB] RELEASE");
                    arm.move(S_X, P_Y, P_Z, S_P, 0, C_OPEN, 800);
                    _currentState = G_STATE_RESET;  
                    _last_state_time = now;
                }
            }
            break;

        case G_STATE_RESET:
            if (now - _last_state_time > 1000) {
                Serial.println("[GRAB] RESET -> done");
                arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
                stop();
            }
            break;

        default: break;
    }
}
