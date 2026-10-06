#ifndef GARBAGE_GRABBER_H
#define GARBAGE_GRABBER_H

#include <Arduino.h>

enum GarbageState {
  GB_STATE_INIT,
  GB_STATE_SET_MODE,
  GB_STATE_WAIT_READY,
  GB_STATE_CONFIG_PARAMS,
  GB_STATE_SEARCH,
  GB_STATE_DOWN,
  GB_STATE_GRAB,
  GB_STATE_LIFT,
  GB_STATE_PLACE,
  GB_STATE_RELEASE,
  GB_STATE_RESET
};

class GarbageGrabber {
public:
    float kp = 0.10f;
    float kd = 0.02f;

    uint8_t conf_threshold = 70;

    void begin();
    void start();
    void stop();
    void update();

    void setOffsets(float x_off, float y_off, float z_grab);
    void setThreshold(uint8_t thresh);

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
    bool _grab_comp_locked = false;
    uint8_t _txn = 0;
    uint16_t _host_gen = 0;
    uint16_t _dev_gen  = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;

    float _last_ex = 0, _last_ey = 0, _last_ew = 0;
    float _grab_comp_ex = 0, _grab_comp_ey = 0;
    float _filtered_cx = 160.0f;
    float _filtered_cy = 120.0f;
    float _f_tx = 200;
    float _f_ty = 0;
    float _f_tz = 200;

    float X_COMP = 15.0f;
    float Y_COMP = 50.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;

    const float P_Y = 150.0f;
    const float P_Z = 150.0f;

    const float S_X = 200.0f;
    const float S_Y = 0.0f;
    const float S_Z = 200.0f;

    const float C_OPEN = -60.0f;
    const float C_CLOSE = -25.0f;

    const float S_P = -90.0f;

    GarbageState _currentState = GB_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    unsigned long _ignore_data_until = 0;
    unsigned long _last_i2c_time = 0;
    int _stableCount = 0;
    unsigned long _stable_start_time = 0;
    float _grab_x = 200;
    float _grab_y = 0;
    bool _cmd_sent = false;
    bool _clawState = false;
    bool _grabOpenHoldSent = false;
    float place_y = 0.0f;

    int _target_id = -1;

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

    // Frame parsing
    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleBbox(const uint8_t* payload, uint16_t plen);
    void handleQuad(const uint8_t* payload, uint16_t plen);
    void forceConfig();
    void calcCompensatedGrabTarget(float& x, float& y) const;
};

extern GarbageGrabber garbageGrabber;

#endif
