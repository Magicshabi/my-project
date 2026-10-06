#include "GarbageGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "EspNow_Ctrl.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10
#define CMD_SET_SIMPLE_RESULT    0x13
#define CMD_DISABLE_RUN          0x14

#define RPT_HEARTBEAT            0x70
#define RPT_DETECT_BBOX          0x72
#define RPT_DETECT_QUAD          0x7B
#define RPT_DETECT_CENTER        0x79
#define APP_GARBAGE              17
#define APP_EMPTY                0

#define CTRL_TYPE_CMD            0x00
#define CTRL_TYPE_RSP            0x40
#define CTRL_TYPE_RPT            0x80

#define HB_RUN                   0x01
#define HB_READY                 0x04

#define MOVE_INTERVAL            40
#define TRACK_FILTER_ALPHA       0.40f
#define CENTER_DEADBAND          15
#define CENTER_RELEASE_BAND      25
#define CENTER_SLOW_BAND         30
#define D_TERM_MAX               40.0f
#define TRACK_MOVE_DURATION      50
#define X_SCREENY_COMP_GAIN      2.50f
#define Y_TRACK_COMP_GAIN        0.10f
#define Y_SCREENX_COMP_GAIN      0.20f
#define I2C_POLL_INTERVAL        10

static const uint8_t GB_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

namespace {
int inferGarbageBin(const char* class_name) {
    if (!class_name || class_name[0] == '\0') return 0;
    String name = String(class_name);
    name.toLowerCase();
    if (name == "plasticbottle") return 0;
    if (name == "toothbrush") return 0;
    if (name == "umbrella") return 0;
    if (name == "bananapeel") return 1;
    if (name == "brokenbones") return 1;
    if (name == "ketchup") return 1;
    if (name == "marker") return 2;
    if (name == "oralliquidbottle") return 2;
    if (name == "storagebattery") return 2;
    if (name == "cigaretteend") return 3;
    if (name == "disposablechopsticks") return 3;
    if (name == "plate") return 3;
    return 0;
}
}

GarbageGrabber garbageGrabber;

// ── I2C low-level ──

void GarbageGrabber::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void GarbageGrabber::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

GarbageGrabber::SlotMetaV2 GarbageGrabber::readSlotMeta(uint16_t offset) {
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

void GarbageGrabber::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool GarbageGrabber::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, GB_MAILBOX_MAGIC, 4) != 0) {
        Serial.println("[GARBAGE] mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf("[GARBAGE] bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf("[GARBAGE] mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool GarbageGrabber::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int GarbageGrabber::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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

uint8_t GarbageGrabber::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint16_t GarbageGrabber::buildFrame(uint8_t* buf, uint8_t func,
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

bool GarbageGrabber::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t flen = buildFrame(frame, func, payload, plen, nextTxn());
    return writeHostSlot(frame, flen);
}

// ── Lifecycle ──

void GarbageGrabber::begin() {
    _busy = false;
    _center_locked = false;
    _grab_comp_locked = false;
    _txn = 0;
    _last_ex = 0; _last_ey = 0; _last_ew = 0;
    _grab_comp_ex = 0; _grab_comp_ey = 0;
    _grabOpenHoldSent = false;
    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z;
    _filtered_cx = 160.0f; _filtered_cy = 120.0f;
    _host_gen = 0; _dev_gen = 0;
    Wire1.setClock(400000);
}

void GarbageGrabber::start() {
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
    _filtered_cx = 160.0f; _filtered_cy = 120.0f;
    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
    delay(1500);

    if (!initMailbox()) {
        Serial.println("[GARBAGE] mailbox init fail, abort");
        _busy = false;
        return;
    }

    sendCmd(CMD_SET_MODE, (const uint8_t[]){APP_GARBAGE}, 1);
    Serial.println("[GARBAGE] SET_MODE sent, waiting ready...");

    bool mode_ok = false;
    for (int retry = 0; retry < 150; retry++) {
        delay(100);
        uint8_t buf[MAX_FRAME_SIZE];
        int got = readDevSlot(buf, sizeof(buf));
        if (got > 0 && got >= 8 && buf[0] == FRAME_H0 && buf[1] == FRAME_H1) {
            uint8_t ftype = buf[4] & 0xC0;
            uint8_t func = buf[5];
            if (ftype == CTRL_TYPE_RPT && func == RPT_HEARTBEAT) {
                uint16_t plen = (buf[2] << 8) | buf[3];
                if (plen >= 2) {
                    uint8_t hb_mode = buf[7];
                    uint8_t hb_status = buf[8];
                    Serial.printf("[GARBAGE] HB mode=%d status=0x%02X\n", hb_mode, hb_status);
                    if (hb_mode == APP_GARBAGE && (hb_status & HB_READY)) {
                        mode_ok = true; break;
                    }
                }
            }
        }
    }
    if (!mode_ok) Serial.println("[GARBAGE] WARNING: mode not confirmed");

    delay(50);
    forceConfig();

    _ignore_data_until = millis() + 500;
    _currentState = GB_STATE_SEARCH;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_i2c_time = millis();
    _cmd_sent = false;
    Serial.println("[GARBAGE] Start -> SEARCH");
}

void GarbageGrabber::stop() {
    if (!_busy) return;
    _busy = false;
    _center_locked = false;
    _grab_comp_locked = false;
    _grab_comp_ex = 0; _grab_comp_ey = 0;
    _grabOpenHoldSent = false;
    uint8_t run_disable = 1;
    sendCmd(CMD_DISABLE_RUN, &run_disable, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);
    arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1500);
    Serial.println("[GARBAGE] Stop");
}

void GarbageGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

void GarbageGrabber::setThreshold(uint8_t thresh) {
    conf_threshold = thresh;
}

void GarbageGrabber::calcCompensatedGrabTarget(float& x, float& y) const {
    x = constrain(_f_tx + X_COMP, S_X - 120.0f, S_X + 240.0f);
    y = constrain(_f_ty + Y_COMP, -500.0f, 500.0f);
}

void GarbageGrabber::forceConfig() {
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
        Serial.printf("[GARBAGE] %s ok=%d\n", label, ok);
        for (int i = 0; i < 20; i++) {
            SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
            if (hm.state == SLOT_EMPTY) break;
            delay(10);
        }
        readDevSlot(tmp, sizeof(tmp));
        delay(20);
    };

    safeSend(CMD_SET_SIMPLE_RESULT, (const uint8_t[]){0}, 1, "SET_SIMPLE=0");
    safeSend(CMD_SET_CONF_THRESH, &conf_threshold, 1, "SET_THRESH");
    safeSend(CMD_DISABLE_RUN, (const uint8_t[]){0}, 1, "ENABLE_RUN");
    Serial.printf("[GARBAGE] forceConfig done (thresh=%d)\n", conf_threshold);
}

// ── Bbox handling ──

void GarbageGrabber::handleBbox(const uint8_t* payload, uint16_t plen) {
    if (_currentState != GB_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) {
        _last_ex = 0; _last_ey = 0; _last_ew = 0;
        return;
    }
    if (off + 8 > plen) return;

    uint16_t cx = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t cy = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t bw = (payload[off] << 8) | payload[off+1]; off += 2;
    uint16_t bh = (payload[off] << 8) | payload[off+1]; off += 2;

    if (bw > 280 || bh > 200) return;

    char class_name[32] = {0};
    int score = -1;
    int score_x100 = -1;
    if (off < plen) {
        uint8_t ext_cnt = payload[off++];
        for (int e = 0; e < ext_cnt; e++) {
            if (off >= plen) break;
            uint8_t etype = payload[off++];
            if (etype == 2) {
                if (off + 2 <= plen) { score_x100 = (payload[off] << 8) | payload[off+1]; off += 2; }
            } else if (etype == 3) {
                if (off + 2 <= plen) { score = (int16_t)((payload[off] << 8) | payload[off+1]); off += 2; }
            } else if (etype == 1) {
                if (off < plen) {
                    uint8_t slen = payload[off++];
                    uint8_t copy_len = slen < 31 ? slen : 31;
                    if (off + slen <= plen) { memcpy(class_name, &payload[off], copy_len); class_name[copy_len] = '\0'; }
                    off += slen;
                }
            } else { off += 2; }
        }
    }

    int effective_score = score;
    if (effective_score < 0 && score_x100 >= 0) effective_score = score_x100 / 100;
    int target_bin = inferGarbageBin(class_name);

    if (effective_score >= 0 && effective_score < conf_threshold) return;

    int raw_cx = (int)cx;
    int raw_cy = (int)cy;
    _filtered_cx += ((float)raw_cx - _filtered_cx) * TRACK_FILTER_ALPHA;
    _filtered_cy += ((float)raw_cy - _filtered_cy) * TRACK_FILTER_ALPHA;
    int fcx = (int)(_filtered_cx + 0.5f);
    int fcy = (int)(_filtered_cy + 0.5f);
    int ex = 160 - fcx;
    int ey = 120 - fcy;
    int ew = 60 - (int)bw;
    if (!_grab_comp_locked) {
        _grab_comp_ex = (float)ex; _grab_comp_ey = (float)ey;
        _grab_comp_locked = true;
    }

    if (_center_locked) {
        if (abs(ex) > CENTER_RELEASE_BAND || abs(ey) > CENTER_RELEASE_BAND) _center_locked = false;
        else { ex = 0; ey = 0; }
    }

    if (abs(ex) <= CENTER_DEADBAND && abs(ey) <= CENTER_DEADBAND) {
        _center_locked = true;
        _last_ex = 0; _last_ey = 0; _last_ew = (float)ew;
        if (_stableCount == 0) _stable_start_time = millis();
        _stableCount++;
        if (millis() - _stable_start_time >= 3000) {
            _target_id = target_bin;
            _grab_x = _f_tx;
            _grab_y = _f_ty;
            calcCompensatedGrabTarget(_grab_x, _grab_y);
            Serial.printf("[GARBAGE] Stable 3s! class='%s' bin=%d x=%.1f y=%.1f\n",
                          class_name, target_bin, _grab_x, _grab_y);
            _currentState = GB_STATE_DOWN;
            _last_state_time = millis();
            _stableCount = 0;
        }
        return;
    }
    _stableCount = 0;

    float dx = constrain((float)ex - _last_ex, -D_TERM_MAX, D_TERM_MAX);
    float dy = constrain((float)ey - _last_ey, -D_TERM_MAX, D_TERM_MAX);
    float out_x = (float)ey * kp + dy * kd;
    float out_y = (float)ex * kp + dx * kd;
    _last_ex = ex; _last_ey = ey; _last_ew = (float)ew;

    _f_tx = constrain(_f_tx + out_x, S_X - 120.0f, S_X + 120.0f);
    _f_ty = constrain(_f_ty + out_y, -300.0f, 300.0f);

    unsigned long now = millis();
    static unsigned long last_move = 0;
    if (now - last_move >= MOVE_INTERVAL) {
        arm.move(_f_tx, _f_ty, S_Z, S_P, 0, C_OPEN, TRACK_MOVE_DURATION);
        last_move = now;
    }
    _stableCount = 0;
}

// ── Frame parsing ──

void GarbageGrabber::parseFrame(const uint8_t* frame, uint16_t len) {
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
        if (func == RPT_DETECT_BBOX) handleBbox(payload, plen);
        else if (func == RPT_DETECT_QUAD) handleQuad(payload, plen);
    }
}

void GarbageGrabber::handleQuad(const uint8_t* payload, uint16_t plen) {
    // QUAD: [count:u8] + count * ([x0:s16][y0:s16]...[x3:s16][y3:s16] + [extra_count:u8] + extra)
    // Convert to bbox and reuse handleBbox
    if (_currentState != GB_STATE_SEARCH) return;
    if (millis() < _ignore_data_until) return;
    if (plen < 1) return;

    uint16_t off = 0;
    uint8_t count = payload[off++];
    if (count == 0) {
        _last_ex = 0; _last_ey = 0; _last_ew = 0;
        return;
    }
    if (off + 16 > plen) return;

    int16_t qx[4], qy[4];
    for (uint8_t i = 0; i < 4; i++) {
        qx[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
        qy[i] = (int16_t)((payload[off]<<8)|payload[off+1]); off += 2;
    }

    // Read extra (label + score)
    uint8_t extra_buf[64];
    uint16_t extra_len = 0;
    if (off < plen) {
        uint8_t ext_cnt = payload[off++];
        uint16_t ext_start = off;
        for (uint8_t e = 0; e < ext_cnt && off < plen; e++) {
            uint8_t etype = payload[off++];
            if (etype == 1 && off < plen) { uint8_t slen = payload[off++]; off += slen; }
            else if ((etype == 2 || etype == 3) && off + 2 <= plen) { off += 2; }
        }
        extra_len = off - ext_start;
        if (extra_len > 0) {
            uint16_t cp = extra_len < sizeof(extra_buf) ? extra_len : sizeof(extra_buf);
            memcpy(extra_buf, &payload[ext_start - 1], cp + 1); // include ext_cnt byte
            extra_len = cp + 1;
        }
    }

    // Compute center from corners
    int16_t min_x = qx[0], max_x = qx[0], min_y = qy[0], max_y = qy[0];
    for (uint8_t i = 1; i < 4; i++) {
        if (qx[i] < min_x) min_x = qx[i];
        if (qx[i] > max_x) max_x = qx[i];
        if (qy[i] < min_y) min_y = qy[i];
        if (qy[i] > max_y) max_y = qy[i];
    }
    uint16_t cx = (uint16_t)((min_x + max_x) / 2);
    uint16_t cy = (uint16_t)((min_y + max_y) / 2);
    uint16_t bw = (uint16_t)(max_x - min_x);
    uint16_t bh = (uint16_t)(max_y - min_y);

    // Build fake bbox payload: [count=1][cx:u16][cy:u16][w:u16][h:u16] + extra
    uint8_t fake[80];
    uint16_t fp = 0;
    fake[fp++] = 1;
    fake[fp++] = (cx >> 8); fake[fp++] = (cx & 0xFF);
    fake[fp++] = (cy >> 8); fake[fp++] = (cy & 0xFF);
    fake[fp++] = (bw >> 8); fake[fp++] = (bw & 0xFF);
    fake[fp++] = (bh >> 8); fake[fp++] = (bh & 0xFF);
    if (extra_len > 0 && fp + extra_len < sizeof(fake)) {
        memcpy(&fake[fp], extra_buf, extra_len);
        fp += extra_len;
    }
    handleBbox(fake, fp);
}

// ── Main update loop ──

void GarbageGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < I2C_POLL_INTERVAL) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) parseFrame(buf, (uint16_t)got);

    switch (_currentState) {
        case GB_STATE_SEARCH:
            break;

        case GB_STATE_DOWN: {
            Serial.printf("[GARBAGE] DOWN x=%.1f y=%.1f z=%.1f\n", _grab_x, _grab_y, GRAB_Z);
            arm.move(_grab_x, _grab_y, GRAB_Z, S_P, 0, C_OPEN, 1500);
            _currentState = GB_STATE_GRAB;
            _grabOpenHoldSent = false;
            _last_state_time = now;
            break;
        }

        case GB_STATE_GRAB:
            if (!_grabOpenHoldSent && now - _last_state_time > 2500) {
                arm.move(_grab_x, _grab_y, GRAB_Z, S_P, 0, C_OPEN, 500);
                _grabOpenHoldSent = true;
                _last_state_time = now;
            } else if (_grabOpenHoldSent && now - _last_state_time > 500) {
                Serial.println("[GARBAGE] GRAB close");
                arm.move(_grab_x, _grab_y, GRAB_Z, S_P, 0, C_CLOSE, 500);
                _grabOpenHoldSent = false;
                _currentState = GB_STATE_LIFT;
                _last_state_time = now;
            }
            break;

        case GB_STATE_LIFT:
            if (now - _last_state_time > 800) {
                Serial.println("[GARBAGE] LIFT");
                arm.move(_grab_x, _grab_y, LIFT_Z, S_P, 0, C_CLOSE, 1000);
                _currentState = GB_STATE_PLACE;
                _last_state_time = now;
            }
            break;

        case GB_STATE_PLACE:
            if (now - _last_state_time > 2000) {
                int bin = (_target_id > 0) ? (_target_id % 4) : 0;
                if (bin == 0) place_y = -150.0f;
                else if (bin == 1) place_y = -50.0f;
                else if (bin == 2) place_y = 50.0f;
                else if (bin == 3) place_y = 150.0f;
                arm.move(S_X, place_y, P_Z, S_P, 0, C_CLOSE, 1500);
                _currentState = GB_STATE_RELEASE;
                _last_state_time = now;
            }
            break;

        case GB_STATE_RELEASE:
            if (!_clawState) {
                if (now - _last_state_time > 2000) {
                    arm.move(S_X, place_y, P_Z, S_P, 0, C_OPEN, 1000);
                    _last_state_time = now;
                    _clawState = true;
                }
            } else {
                if (now - _last_state_time > 3000) {
                    arm.move(S_X, 0, P_Z, S_P, 0, C_OPEN, 1000);
                    _currentState = GB_STATE_RESET;
                    _last_state_time = now;
                }
            }
            break;

        case GB_STATE_RESET:
            if (now - _last_state_time > 1000) {
                arm.move(S_X, S_Y, S_Z, S_P, 0, C_OPEN, 1000);
                stop();
            }
            break;

        default: break;
    }
}
