#ifndef COLOR_TRACKER_ROT_H
#define COLOR_TRACKER_ROT_H

#include <Arduino.h>

enum ColorTrackState {
  CT_STATE_INIT,
  CT_STATE_SET_MODE,
  CT_STATE_WAIT_READY,
  CT_STATE_CONFIG_PARAMS,
  CT_STATE_ENABLE_RUN,
  CT_STATE_RUNNING
};

class ColorTrackerRot {
public:
    float kp = 0.07f;
    float kd = 0.003f;

    void begin();
    void start(const char* colorName);
    void stop();
    void update();

    bool isCentered() const { return _center_locked; }
    float getTargetX() const { return _f_tx; }
    float getTargetY() const { return _f_ty; }

private:
    enum SlotStateV2 : uint8_t {
        SLOT_EMPTY   = 0,
        SLOT_WRITING = 1,
        SLOT_READY   = 2
    };

    struct SlotMetaV2 {
        uint8_t  state;
        uint8_t  reserved0;
        uint16_t generation;
        uint16_t frame_len;
        uint8_t  frame_xor;
        uint8_t  reserved1;
    };

    static constexpr uint8_t  K230_ADDR = 0x5F;
    static constexpr uint16_t MAILBOX_SIZE = 4096;
    static constexpr uint16_t MAILBOX_HEADER_SIZE = 32;
    static constexpr uint16_t HOST_SLOT_META_OFFSET = 16;
    static constexpr uint16_t DEV_SLOT_META_OFFSET  = 24;
    static constexpr uint16_t MAX_FRAME_SIZE = 256;
    static constexpr uint8_t  FRAME_H0 = 0xAA;
    static constexpr uint8_t  FRAME_H1 = 0x55;

    bool _busy = false;
    bool _center_locked = false;
    uint8_t _txn = 0;
    uint16_t _host_gen = 0;
    uint16_t _dev_gen  = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;
    String _target_color;

    float _last_ex = 0;
    float _last_ey = 0;
    float _f_tx = 200;
    float _f_ty = 0;
    float _f_tz = 200;

    ColorTrackState _currentState = CT_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_i2c_time = 0;
    unsigned long _ignore_data_until = 0;
    bool _cmd_sent = false;

    // Mailbox v2 helpers
    void     i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len);
    void     i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len);
    bool     initMailbox();
    SlotMetaV2 readSlotMeta(uint16_t offset);
    void     writeSlotMeta(uint16_t offset, const SlotMetaV2& m);
    bool     writeHostSlot(const uint8_t* frame, uint16_t len);
    int      readDevSlot(uint8_t* buf, uint16_t bufSize);

    // Protocol v2 frame
    uint8_t  nextTxn();
    uint16_t buildFrame(uint8_t* buf, uint8_t func, const uint8_t* payload, uint16_t plen, uint8_t txn = 0);
    bool     sendCmd(uint8_t func, const uint8_t* payload = nullptr, uint16_t plen = 0);

    // Frame parsing & state machine
    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleHeartbeat(const uint8_t* payload, uint16_t plen);
    void handleColor(const uint8_t* payload, uint16_t plen);
    void handleCenter(const uint8_t* payload, uint16_t plen);
    void runStateMachine(uint8_t mode, uint8_t status);
    void forceConfig();
    bool safeSend(uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label);
};

extern ColorTrackerRot colorTrackerRot;

#endif
