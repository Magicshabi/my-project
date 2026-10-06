// AiLLMControl — MCP tool control for robotic arm (Mailbox v2)
#include "AiLLMControl.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "ColorTrackerRot.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

AiLLMControl aiLLMControl;

// ── Data type constants ──
namespace {
constexpr uint8_t TYPE_NULL   = 0x00;
constexpr uint8_t TYPE_BOOL_F = 0x01;
constexpr uint8_t TYPE_BOOL_T = 0x02;
constexpr uint8_t TYPE_INT8   = 0x03;
constexpr uint8_t TYPE_INT16  = 0x04;
constexpr uint8_t TYPE_INT32  = 0x05;
constexpr uint8_t TYPE_UINT8  = 0x06;
constexpr uint8_t TYPE_UINT16 = 0x07;
constexpr uint8_t TYPE_UINT32 = 0x08;
constexpr uint8_t TYPE_STRING = 0x09;
constexpr uint8_t TYPE_ARRAY  = 0x0A;
constexpr uint8_t TYPE_DICT   = 0x0B;

#define CTRL_TYPE_CMD  0x00
#define CTRL_TYPE_RSP  0x40
#define CTRL_TYPE_RPT  0x80
#define CMD_SET_MODE   0x01
#define CMD_DISABLE_RUN 0x14
#define APP_EMPTY      0

bool isActionText(const char* text) {
    return strstr(text, "look_down") || strstr(text, "低头") ||
           strstr(text, "look_up") || strstr(text, "抬头") ||
           strstr(text, "look_left") || strstr(text, "向左") ||
           strstr(text, "look_right") || strstr(text, "向右") ||
           strstr(text, "open_claw") || strstr(text, "张开") ||
           strstr(text, "close_claw") || strstr(text, "闭合") ||
           strstr(text, "go_up") || strstr(text, "升高") ||
           strstr(text, "go_down") || strstr(text, "降低") ||
           strstr(text, "reset") || strstr(text, "初始") ||
           strstr(text, "nod") || strstr(text, "点头") ||
           strstr(text, "shake") || strstr(text, "摇头");
}
} // namespace

// ── DataPacker ──
class DataPacker {
public:
    uint8_t buf[2048];
    uint16_t len = 0;
    void addString(const char* s) {
        buf[len++] = TYPE_STRING;
        uint16_t slen = strlen(s);
        buf[len++] = (slen >> 8) & 0xFF;
        buf[len++] = slen & 0xFF;
        memcpy(&buf[len], s, slen);
        len += slen;
    }
    void addBool(bool v) { buf[len++] = v ? TYPE_BOOL_T : TYPE_BOOL_F; }
    void addUint8(uint8_t v) { buf[len++] = TYPE_UINT8; buf[len++] = v; }
    void beginArray(uint16_t c) { buf[len++] = TYPE_ARRAY; buf[len++] = (c >> 8) & 0xFF; buf[len++] = c & 0xFF; }
    void beginDict(uint16_t c) { buf[len++] = TYPE_DICT; buf[len++] = (c >> 8) & 0xFF; buf[len++] = c & 0xFF; }
    void endArray() {}
    void endDict() {}
};

static const uint8_t MCP_MAILBOX_MAGIC[4] = {0x57, 0x4C, 0x4D, 0x32};

// ── I2C low-level ──

void AiLLMControl::i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len) {
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

void AiLLMControl::i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len) {
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

AiLLMControl::SlotMetaV2 AiLLMControl::readSlotMeta(uint16_t offset) {
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

void AiLLMControl::writeSlotMeta(uint16_t offset, const SlotMetaV2& m) {
    uint8_t raw[8] = {
        m.state, m.reserved0,
        (uint8_t)(m.generation >> 8), (uint8_t)(m.generation & 0xFF),
        (uint8_t)(m.frame_len >> 8),  (uint8_t)(m.frame_len & 0xFF),
        m.frame_xor, m.reserved1
    };
    i2cWrite16(offset, raw, 8);
}

bool AiLLMControl::initMailbox() {
    uint8_t header[8];
    i2cRead16(0, header, 8);
    if (memcmp(header, MCP_MAILBOX_MAGIC, 4) != 0) {
        Serial.println(">>>> mailbox magic mismatch");
        return false;
    }
    _slot_size = (header[6] << 8) | header[7];
    if (_slot_size == 0 || _slot_size > (MAILBOX_SIZE - MAILBOX_HEADER_SIZE) / 2) {
        Serial.printf(">>>> bad slot_size=%d\n", _slot_size);
        return false;
    }
    _host_slot_data_offset = MAILBOX_HEADER_SIZE;
    _dev_slot_data_offset  = MAILBOX_HEADER_SIZE + _slot_size;
    _host_gen = readSlotMeta(HOST_SLOT_META_OFFSET).generation;
    _dev_gen  = readSlotMeta(DEV_SLOT_META_OFFSET).generation;
    Serial.printf(">>>> mailbox ok slot_size=%d\n", _slot_size);
    return true;
}

bool AiLLMControl::writeHostSlot(const uint8_t* frame, uint16_t len) {
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

int AiLLMControl::readDevSlot(uint8_t* buf, uint16_t bufSize) {
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

uint8_t AiLLMControl::nextTxn() {
    _txn++;
    if (_txn == 0) _txn = 1;
    return _txn;
}

uint8_t AiLLMControl::nextSeq() {
    uint8_t seq = _seq & CTRL_SEQ_MASK;
    _seq = (_seq + 1) & CTRL_SEQ_MASK;
    return seq;
}

uint16_t AiLLMControl::buildFrame(uint8_t* buf, uint8_t func,
    const uint8_t* payload, uint16_t plen, uint8_t txn, uint8_t seq, bool continuation)
{
    buf[0] = FRAME_H0; buf[1] = FRAME_H1;
    buf[2] = (uint8_t)(plen >> 8); buf[3] = (uint8_t)(plen & 0xFF);
    buf[4] = CTRL_TYPE_CMD | (continuation ? CTRL_CONT : 0) | (seq & CTRL_SEQ_MASK);
    buf[5] = func; buf[6] = txn;
    if (payload && plen > 0) memcpy(&buf[7], payload, plen);
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= buf[i];
    buf[7 + plen] = xor_val;
    return 8 + plen;
}

bool AiLLMControl::sendCmd(uint8_t func, const uint8_t* payload, uint16_t plen) {
    if (_slot_size <= 8) {
        Serial.println(">>>> mailbox slot not ready");
        return false;
    }
    if (plen > MAX_FRAME_SIZE - 8) {
        Serial.printf(">>>> payload too large func=0x%02X len=%d\n", func, plen);
        return false;
    }

    const uint16_t maxPayload = min((uint16_t)(MAX_FRAME_SIZE - 8), (uint16_t)(_slot_size - 8));
    uint16_t offset = 0;
    uint8_t txn = nextTxn();
    bool sentAny = false;

    do {
        uint16_t chunkLen = (plen == 0) ? 0 : min(maxPayload, (uint16_t)(plen - offset));
        bool continuation = (offset + chunkLen) < plen;

        bool slotReady = false;
        for (int i = 0; i < 40; i++) {
            SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
            if (hm.state == SLOT_EMPTY) {
                slotReady = true;
                break;
            }
            uint8_t tmp[MAX_FRAME_SIZE];
            int got = readDevSlot(tmp, sizeof(tmp));
            if (got > 0) parseFrame(tmp, (uint16_t)got);
            delay(25);
        }
        if (!slotReady) {
            Serial.printf(">>>> host slot busy, cannot send func=0x%02X off=%d\n", func, offset);
            return false;
        }

        uint8_t frame[MAX_FRAME_SIZE];
        uint16_t flen = buildFrame(frame, func,
                                   payload ? payload + offset : nullptr,
                                   chunkLen,
                                   txn,
                                   nextSeq(),
                                   continuation);
        if (!writeHostSlot(frame, flen)) {
            Serial.printf(">>>> send fragment failed func=0x%02X off=%d len=%d slot=%d\n",
                          func, offset, chunkLen, _slot_size);
            return false;
        }

        sentAny = true;
        offset += chunkLen;
    } while (offset < plen);

    return sentAny;
}

// ── Lifecycle ──

void AiLLMControl::begin() {
    _busy = false;
    _auto_start_pending = false;
    _start_attempts = 0;
    _txn = 0;
    _seq = 0;
    _host_gen = 0; _dev_gen = 0;
    _last_start_attempt_time = 0;
    resetRxFragment();
    Wire1.begin(21, 22);
    Wire1.setClock(400000);
    Wire1.setBufferSize(256);
}

void AiLLMControl::start() {
    if (_busy) return;
    _busy = true;
    _txn = 0;
    _seq = 0;
    _host_gen = 0; _dev_gen = 0;
    resetRxFragment();

    // 移到初始位置
    arm.move(200, 0, 200, 0, 0, 0, 1500);
    _cur_x = 200; _cur_y = 0; _cur_z = 200; _cur_pitch = 0; _cur_claw = 0;

    if (!initMailbox()) {
        Serial.println(">>>> mailbox init fail, abort");
        _busy = false;
        _auto_start_pending = true;
        return;
    }

    // 发送 MCP tools，如果收到 ERR_NOT_READY 则重试
    Serial.println(">>>> SET_MCP_TOOLS");
    bool mcp_ok = false;
    for (int attempt = 0; attempt < 20; attempt++) {
        setupMCPTools();

        // 等 RSP
        delay(500);
        uint8_t buf[MAX_FRAME_SIZE];
        int got = readDevSlot(buf, sizeof(buf));
        if (got > 0) {
            // 在 payload 里找 err byte：帧格式 [H0 H1 plen_hi plen_lo ctrl func txn payload... xor]
            // RSP 的 ctrl & 0xC0 == 0x40, func == 0x6C
            // payload[0] = err_code
            if (got >= 8) {
                uint8_t ctrl = buf[4];
                uint8_t func = buf[5];
                uint8_t err = buf[7];
                Serial.printf(">>>> RSP ctrl=0x%02X func=0x%02X err=0x%02X (attempt %d)\n",
                              ctrl, func, err, attempt + 1);
                if (err == 0x00) {
                    mcp_ok = true;
                    break;
                }
                if (err != 0x06) break;  // 非 NOT_READY，不重试
            }
        }
        Serial.printf(">>>> MCP retry %d/20...\n", attempt + 1);
        delay(500);
    }

    if (!mcp_ok) {
        Serial.println(">>>> MCP tools registration failed");
    }

    _last_poll_time = millis();
    _last_i2c_time = millis();
    _auto_start_pending = false;
    Serial.println(">>>> MCP tools registered, listening for tool calls...");
}

void AiLLMControl::stop() {
    _auto_start_pending = false;
    if (!_busy) return;
    _busy = false;
    _grab_state = LG_IDLE;

    uint8_t run_off = 1;
    sendCmd(CMD_DISABLE_RUN, &run_off, 1);
    delay(80);
    uint8_t empty_mode = APP_EMPTY;
    sendCmd(CMD_SET_MODE, &empty_mode, 1);
    delay(120);

    Serial.println(">>>> MCP control stop");
}

// ── MCP tool definitions ──

void AiLLMControl::setupMCPTools() {
    DataPacker p;
    p.beginArray(1);
      p.beginDict(3);
        p.addString("type");
        p.addString("function");
        p.addString("function");
        p.beginDict(3);
          p.addString("name");
          p.addString("move_arm");
          p.addString("description");
          p.addString("当用户要求机械臂动作时必须调用此工具，包括点头、摇头、抬头、低头、向左看、向右看、张开爪子、闭合爪子。");
          p.addString("parameters");
          p.beginDict(3);
            p.addString("type");
            p.addString("object");
            p.addString("properties");
            p.beginDict(1);
              p.addString("name");
              p.beginDict(2);
                p.addString("type");
                p.addString("string");
                p.addString("description");
                p.addString("动作名称，可选 nod、shake、look_up、look_down、look_left、look_right、open_claw、close_claw。");
          p.addString("required");
          p.beginArray(1);
            p.addString("name");
      p.addString("block");
      p.addUint8(5);

    p.endArray();

    // Drain slot before sending large MCP tools payload
    uint8_t tmp[MAX_FRAME_SIZE];
    readDevSlot(tmp, sizeof(tmp));
    delay(50);
    for (int i = 0; i < 20; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        readDevSlot(tmp, sizeof(tmp));
        delay(50);
    }
    bool ok = sendCmd(CMD_SET_MCP_TOOLS, p.buf, p.len);
    Serial.printf(">>>> MCP tools sent ok=%d payload=%d bytes\n", ok, p.len);

    // Wait for K230 to consume
    for (int i = 0; i < 30; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        delay(50);
    }
    // Drain RSP
    int got = readDevSlot(tmp, sizeof(tmp));
    if (got > 0) parseFrame(tmp, (uint16_t)got);
    delay(50);
}

// ── Action execution ──

bool AiLLMControl::executeAction(const uint8_t* data, uint16_t len) {
    char action[64] = {0};
    for (int i = 0; i <= (int)len - 4; i++) {
        if (memcmp(&data[i], "name", 4) == 0) {
            int val_off = i + 4;
            if (val_off < len && data[val_off] == TYPE_STRING) {
                uint16_t vlen = (data[val_off + 1] << 8) | data[val_off + 2];
                int vs = val_off + 3;
                if (vs + vlen <= len) {
                    int copy = (vlen < 63) ? vlen : 63;
                    memcpy(action, &data[vs], copy);
                    action[copy] = '\0';
                    break;
                }
            }
        }
    }

    if (action[0] == '\0') {
        for (int i = 0; i <= (int)len - 3; i++) {
            if (data[i] != TYPE_STRING) continue;
            uint16_t vlen = ((uint16_t)data[i + 1] << 8) | data[i + 2];
            int vs = i + 3;
            if (vlen == 0 || vs + vlen > (int)len) continue;

            char candidate[64] = {0};
            int copy = (vlen < 63) ? vlen : 63;
            memcpy(candidate, &data[vs], copy);
            candidate[copy] = '\0';
            if (isActionText(candidate)) {
                memcpy(action, candidate, copy + 1);
                Serial.printf(">>>> Action fallback string: '%s'\n", action);
                break;
            }
        }
    }

    if (action[0] == '\0') {
        Serial.println(">>>> No action name found");
        return false;
    }

    Serial.printf(">>>> Action: '%s'\n", action);

    if (strstr(action, "look_down") || strstr(action, "低头")) {
        arm.move(200, 0, 200, -90, 0, _cur_claw, 1000);
        _cur_pitch = -90;
    } else if (strstr(action, "look_up") || strstr(action, "抬头")) {
        arm.move(200, 0, 200, 45, 0, _cur_claw, 1000);
        _cur_pitch = 45;
    } else if (strstr(action, "look_left") || strstr(action, "向左")) {
        arm.move(200, 150, 200, 0, 0, _cur_claw, 1000);
        _cur_y = 150;
    } else if (strstr(action, "look_right") || strstr(action, "向右")) {
        arm.move(200, -150, 200, 0, 0, _cur_claw, 1000);
        _cur_y = -150;
    } else if (strstr(action, "open_claw") || strstr(action, "张开")) {
        arm.move(_cur_x, _cur_y, _cur_z, _cur_pitch, 0, -60, 500);
        _cur_claw = -60;
    } else if (strstr(action, "close_claw") || strstr(action, "闭合")) {
        arm.move(_cur_x, _cur_y, _cur_z, _cur_pitch, 0, 20, 500);
        _cur_claw = 20;
    } else if (strstr(action, "go_up") || strstr(action, "升高")) {
        arm.move(200, 0, 300, 0, 0, _cur_claw, 1000);
        _cur_z = 300;
    } else if (strstr(action, "go_down") || strstr(action, "降低")) {
        arm.move(200, 0, 100, 0, 0, _cur_claw, 1000);
        _cur_z = 100;
    } else if (strstr(action, "reset") || strstr(action, "初始")) {
        arm.move(200, 0, 200, 0, 0, -60, 1000);
        _cur_x = 200; _cur_y = 0; _cur_z = 200; _cur_pitch = 0; _cur_claw = -60;
    } else if (strstr(action, "nod") || strstr(action, "点头")) {
        arm.move(200, 0, 200, -30, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, 0, 200, 0, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, 0, 200, -30, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, 0, 200, 0, 0, _cur_claw, 300);
    } else if (strstr(action, "shake") || strstr(action, "摇头")) {
        arm.move(200, 80, 200, 0, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, -80, 200, 0, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, 80, 200, 0, 0, _cur_claw, 300);
        delay(400);
        arm.move(200, 0, 200, 0, 0, _cur_claw, 300);
    } else {
        Serial.printf(">>>> Unknown action: '%s'\n", action);
        return false;
    }

    Serial.println(">>>> Action done!");
    return true;
}

bool AiLLMControl::executeGrabColor(const uint8_t* data, uint16_t len) {
    char color[32] = {0};
    for (int i = 0; i <= (int)len - 4; i++) {
        if (memcmp(&data[i], "name", 4) == 0) {
            int val_off = i + 4;
            if (val_off < len && data[val_off] == TYPE_STRING) {
                uint16_t vlen = (data[val_off + 1] << 8) | data[val_off + 2];
                int vs = val_off + 3;
                if (vs + vlen <= len) {
                    int copy = (vlen < 31) ? vlen : 31;
                    memcpy(color, &data[vs], copy);
                    color[copy] = '\0';
                    break;
                }
            }
        }
    }

    if (color[0] == '\0') {
        Serial.println(">>>> No color name found");
        return false;
    }

    Serial.printf(">>>> Grab color: '%s'\n", color);

    const char* cName = nullptr;
    if (strstr(color, "red") || strstr(color, "红")) cName = "red";
    else if (strstr(color, "green") || strstr(color, "绿")) cName = "green";
    else if (strstr(color, "blue") || strstr(color, "蓝")) cName = "blue";

    if (!cName) {
        Serial.printf(">>>> Unknown color: '%s'\n", color);
        return false;
    }

    colorTrackerRot.stop();
    colorTrackerRot.start(cName);
    _grab_state = LG_IDLE;
    _busy = false;
    Serial.printf(">>>> Color tracking started: %s\n", cName);
    return true;
}

bool AiLLMControl::executeTrackColor(const uint8_t* data, uint16_t len) {
    char color[64] = {0};
    for (int i = 0; i <= (int)len - 5; i++) {
        if (memcmp(&data[i], "color", 5) == 0) {
            int val_off = i + 5;
            if (val_off < len && data[val_off] == TYPE_STRING) {
                uint16_t vlen = (data[val_off + 1] << 8) | data[val_off + 2];
                int vs = val_off + 3;
                if (vs + vlen <= len) {
                    int copy = (vlen < 63) ? vlen : 63;
                    memcpy(color, &data[vs], copy);
                    color[copy] = '\0';
                    break;
                }
            }
        }
    }

    if (color[0] == '\0') {
        Serial.println(">>>> track_color: no color param");
        return false;
    }

    const char* cName = nullptr;
    if (strstr(color, "red") || strstr(color, "红")) cName = "red";
    else if (strstr(color, "green") || strstr(color, "绿")) cName = "green";
    else if (strstr(color, "blue") || strstr(color, "蓝")) cName = "blue";

    if (!cName) {
        Serial.printf(">>>> track_color: unknown '%s'\n", color);
        return false;
    }

    colorTrackerRot.stop();
    colorTrackerRot.start(cName);
    _busy = false;
    Serial.printf(">>>> Color tracking started: %s\n", cName);
    return true;
}

// ── Heartbeat handling (v2 format) ──

void AiLLMControl::handleHeartbeat(const uint8_t* payload, uint16_t plen) {
    if (plen < 2) return;
    uint8_t mode = payload[0];
    uint8_t status = payload[1];
    _last_hb_mode = mode;
    _last_hb_status = status;

    static uint8_t last_status = 0;
    static uint8_t last_mode = 0xFF;
    if (status != last_status || mode != last_mode) {
        Serial.printf(">>>> HB mode=%d status=0x%02X\n", mode, status);
        last_status = status;
        last_mode = mode;
    }
}

void AiLLMControl::debugDumpBytes(const char* label, const uint8_t* data, uint16_t len, uint16_t limit) {
    // Serial.printf(">>>> %s len=%d", label, len);
    if (len > limit) Serial.printf(" dump=%d", limit);
    Serial.println();

    uint16_t n = min(len, limit);
    for (uint16_t i = 0; i < n; i++) {
        if ((i % 16) == 0) Serial.printf(">>>>   %04X: ", i);
        // Serial.printf("%02X ", data[i]);
        if ((i % 16) == 15 || i + 1 == n) Serial.println();
    }
    if (len > limit) Serial.println(">>>>   ...");
}

void AiLLMControl::debugDumpDataPackStrings(const uint8_t* data, uint16_t len) {
    // Serial.println(">>>> data_pack strings:");
    bool found = false;
    for (uint16_t i = 0; i + 3 <= len; i++) {
        if (data[i] != TYPE_STRING) continue;
        uint16_t slen = ((uint16_t)data[i + 1] << 8) | data[i + 2];
        if (slen == 0 || i + 3 + slen > len) continue;

        // Serial.printf(">>>>   @%d string(%d): ", i, slen);
        uint16_t n = min(slen, (uint16_t)120);
        for (uint16_t j = 0; j < n; j++) {
            char c = (char)data[i + 3 + j];
            if ((uint8_t)c >= 0x20 && (uint8_t)c <= 0x7E) Serial.print(c);
            else Serial.printf("\\x%02X", (uint8_t)c);
        }
        if (slen > n) Serial.print("...");
        Serial.println();
        found = true;
    }
    if (!found) Serial.println(">>>>   <none>");
}

bool AiLLMControl::sendMCPResult(const char* tool, bool ok, const char* result) {
    DataPacker p;
    p.addString(result);

    uint8_t tmp[MAX_FRAME_SIZE];
    readDevSlot(tmp, sizeof(tmp));
    delay(10);
    for (int i = 0; i < 10; i++) {
        SlotMetaV2 hm = readSlotMeta(HOST_SLOT_META_OFFSET);
        if (hm.state == SLOT_EMPTY) break;
        readDevSlot(tmp, sizeof(tmp));
        delay(20);
    }

    bool sent = sendCmd(CMD_RESULT_RETURN, p.buf, p.len);
    // Serial.printf(">>>> MCP result sent ok=%d tool=%s success=%d\n", sent, tool, ok);
    return sent;
}

void AiLLMControl::handleResultRSP(const uint8_t* payload, uint16_t plen) {
    // Protocol v2.4 RSP payload:
    // [err_code:u8][err_module:u8][err_subcode:u16_be][extra:data_pack...]
    if (plen < 4) {
        // Serial.println(">>>> RSP RESULT too short");
        return;
    }

    uint8_t err_code = payload[0];
    uint8_t err_module = payload[1];
    uint16_t err_subcode = ((uint16_t)payload[2] << 8) | payload[3];
    const uint8_t* result_data = &payload[4];
    int result_len = plen - 4;

    // Serial.printf(">>>> MCP result err=0x%02X len=%d\n", err_code, result_len);

    if (err_code != 0x00 || result_len <= 0) return;

    bool is_move = false;
    bool is_grab = false;
    bool is_track = false;
    for (int i = 0; i <= result_len - 8; i++) {
        if (memcmp(&result_data[i], "move_arm", 8) == 0) { is_move = true; break; }
    }
    for (int i = 0; i <= result_len - 10; i++) {
        if (memcmp(&result_data[i], "grab_color", 10) == 0) { is_grab = true; break; }
    }
    for (int i = 0; i <= result_len - 11; i++) {
        if (memcmp(&result_data[i], "track_color", 11) == 0) { is_track = true; break; }
    }

    if (is_move) {
        Serial.println(">>>> MCP move_arm call!");
        bool ok = executeAction(result_data, result_len);
        sendMCPResult("move_arm", ok, ok ? "执行成功" : "执行失败");
    } else if (is_grab) {
        Serial.println(">>>> MCP grab_color call!");
        bool ok = executeGrabColor(result_data, result_len);
        sendMCPResult("grab_color", ok, ok ? "执行成功" : "执行失败");
    } else if (is_track) {
        Serial.println(">>>> MCP track_color call!");
        bool ok = executeTrackColor(result_data, result_len);
        sendMCPResult("track_color", ok, ok ? "开始追踪" : "执行失败");
    }
}

// ── Frame parsing ──

void AiLLMControl::resetRxFragment() {
    _rx_frag_active = false;
    _rx_frag_type = 0;
    _rx_frag_func = 0;
    _rx_frag_txn = 0;
    _rx_frag_expected_seq = 0;
    _rx_frag_len = 0;
}

void AiLLMControl::parseFrame(const uint8_t* frame, uint16_t len) {
    if (len < 8) {
        Serial.printf(">>>> RX short frame len=%d\n", len);
        debugDumpBytes("RX short raw", frame, len, 64);
        return;
    }
    if (frame[0] != FRAME_H0 || frame[1] != FRAME_H1) {
        Serial.printf(">>>> RX bad header len=%d h=%02X %02X\n", len, frame[0], frame[1]);
        debugDumpBytes("RX bad raw", frame, len, 64);
        return;
    }
    uint16_t plen = (frame[2] << 8) | frame[3];
    if (8 + plen > len) {
        Serial.printf(">>>> RX length mismatch plen=%d frame_len=%d\n", plen, len);
        debugDumpBytes("RX mismatch raw", frame, len, 96);
        return;
    }
    uint8_t xor_val = 0;
    for (uint16_t i = 2; i < 7 + plen; i++) xor_val ^= frame[i];
    if (xor_val != frame[7 + plen]) {
        Serial.printf(">>>> RX xor fail calc=%02X got=%02X plen=%d\n", xor_val, frame[7 + plen], plen);
        debugDumpBytes("RX xor raw", frame, len, 96);
        return;
    }
    uint8_t ctrl = frame[4];
    uint8_t ftype = ctrl & 0xC0;
    bool continuation = (ctrl & CTRL_CONT) != 0;
    uint8_t seq = ctrl & CTRL_SEQ_MASK;
    uint8_t func = frame[5];
    uint8_t txn = frame[6];
    const uint8_t* payload = &frame[7];

    bool verboseFrame = !(ftype == CTRL_TYPE_RPT && func == RPT_HEARTBEAT);

    if (continuation || _rx_frag_active) {
        if (!_rx_frag_active) {
            _rx_frag_active = true;
            _rx_frag_type = ftype;
            _rx_frag_func = func;
            _rx_frag_txn = txn;
            _rx_frag_expected_seq = seq;
            _rx_frag_len = 0;
        } else if (ftype != _rx_frag_type || func != _rx_frag_func ||
                   txn != _rx_frag_txn || seq != _rx_frag_expected_seq) {
            resetRxFragment();
            return;
        }

        if ((uint32_t)_rx_frag_len + plen > sizeof(_rx_frag_buf)) {
            resetRxFragment();
            return;
        }

        memcpy(&_rx_frag_buf[_rx_frag_len], payload, plen);
        _rx_frag_len += plen;
        _rx_frag_expected_seq = (seq + 1) & CTRL_SEQ_MASK;

        if (continuation) return;

        payload = _rx_frag_buf; 
        plen = _rx_frag_len;
        ftype = _rx_frag_type;
        func = _rx_frag_func;
        resetRxFragment();
    }

    if (ftype == CTRL_TYPE_RPT) {
        if (func == RPT_HEARTBEAT) handleHeartbeat(payload, plen);
        else if (func == CMD_RESULT_RETURN && plen >= 4) {
            handleResultRSP(payload, plen);
        }
    } else if (ftype == CTRL_TYPE_RSP) {
        uint8_t err_code = (plen >= 1) ? payload[0] : 0xFF;
        Serial.printf(">>>> RSP func=0x%02X err=0x%02X plen=%d\n", func, err_code, plen);
        if (func == CMD_RESULT_RETURN && plen >= 4) {
            handleResultRSP(payload, plen);
        }
    }
}

// ── Main update loop ──

void AiLLMControl::update() {
    // Drive grab state machine even when _busy is false
    if (_grab_state != LG_IDLE) {
        updateGrab();
    }

    if (!_busy && _auto_start_pending) {
        if (_start_attempts >= 5) {
            _auto_start_pending = false;
            return;
        }
        unsigned long now = millis();
        if (now - _last_start_attempt_time > 2000) {
            _last_start_attempt_time = now;
            _start_attempts++;
            Serial.printf(">>>> MCP auto start attempt %d/5\n", _start_attempts);
            start();
        }
    }

    if (!_busy) return;
    unsigned long now = millis();
    if (now - _last_i2c_time < 20) return;
    _last_i2c_time = now;

    uint8_t buf[MAX_FRAME_SIZE];
    int got = readDevSlot(buf, sizeof(buf));
    if (got > 0) {
        parseFrame(buf, (uint16_t)got);
    }

}

void AiLLMControl::updateGrab() {
    unsigned long now = millis();
    unsigned long elapsed = now - _grab_state_time;

    switch (_grab_state) {
        case LG_DONE:
            if (elapsed > 1500) {
                Serial.println(">>>> GRAB: done, resuming MCP");
                _grab_state = LG_IDLE;
                start();
            }
            break;

        default:
            break;
    }
}
