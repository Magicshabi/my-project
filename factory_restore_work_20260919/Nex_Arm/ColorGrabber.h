#ifndef COLOR_GRABBER_H
#define COLOR_GRABBER_H

#include <Arduino.h>

enum ColorGrabState {
    CG_STATE_INIT,
    CG_STATE_SET_MODE,
    CG_STATE_WAIT_READY,
    CG_STATE_CONFIG,
    CG_STATE_SEARCH,
    CG_STATE_WRIST,
    CG_STATE_DOWN,
    CG_STATE_GRAB,
    CG_STATE_LIFT,
    CG_STATE_PLACE,
    CG_STATE_RELEASE,
    CG_STATE_RESET
};

class ColorGrabber {
public:
    float kp = 0.08f;
    float kd = 0.01f;

    void begin();
    void start(const char* colors[], uint8_t count);
    void stop();
    void update();
    void setOffsets(float x_off, float y_off, float z_grab);
    bool isBusy() const { return _busy; }

private:
    enum SlotStateV2 : uint8_t { SLOT_EMPTY=0, SLOT_WRITING=1, SLOT_READY=2 };
    struct SlotMetaV2 {
        uint8_t state; uint8_t reserved0;
        uint16_t generation; uint16_t frame_len;
        uint8_t frame_xor; uint8_t reserved1;
    };

    static constexpr uint8_t  K230_ADDR = 0x5F;
    static constexpr uint16_t MAILBOX_SIZE = 4096;
    static constexpr uint16_t MAILBOX_HEADER_SIZE = 32;
    static constexpr uint16_t HOST_SLOT_META_OFFSET = 16;
    static constexpr uint16_t DEV_SLOT_META_OFFSET  = 24;
    static constexpr uint16_t MAX_FRAME_SIZE = 256;
    static constexpr uint8_t  FRAME_H0 = 0xAA;
    static constexpr uint8_t  FRAME_H1 = 0x55;

    static constexpr uint8_t  APP_SINGLE_COLOR = 21;
    static constexpr float    BLOCK_H = 30.0f;

    // Place position (configurable)
    float PLACE_X = 200.0f;
    float PLACE_Y = -200.0f;
    float PLACE_Z = 60.0f;

    // Search position
    static constexpr float S_X = 200.0f;
    static constexpr float S_Y = 0.0f;
    static constexpr float S_Z = 200.0f;
    static constexpr float S_P = -90.0f;
    static constexpr float C_OPEN = -60.0f;
    static constexpr float C_CLOSE = 20.0f;
    static constexpr float LIFT_Z = 220.0f;

    bool _busy = false;
    uint8_t _txn = 0;
    uint16_t _host_gen = 0, _dev_gen = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;

    ColorGrabState _currentState = CG_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_i2c_time = 0;
    unsigned long _ignore_data_until = 0;
    unsigned long _stable_start_time = 0;
    bool _cmd_sent = false;

    // Tracking
    float _f_tx = S_X, _f_ty = S_Y, _f_tz = S_Z;
    float _last_ex = 0, _last_ey = 0;
    bool  _center_locked = false;
    int   _stableCount = 0;
    float _grab_x = S_X, _grab_y = S_Y;

    // Color sequence
    String _color_list[8];
    uint8_t _color_count = 0;
    uint8_t _color_index = 0;
    uint8_t _stack_count = 0;
    String _current_color;

    // Offsets (from calibration)
    float X_COMP = 0, Y_COMP = 0, GRAB_Z = 60.0f;

    // Mailbox v2
    void     i2cWrite16(uint16_t memAddr, const uint8_t* data, uint16_t len);
    void     i2cRead16(uint16_t memAddr, uint8_t* data, uint16_t len);
    bool     initMailbox();
    SlotMetaV2 readSlotMeta(uint16_t offset);
    void     writeSlotMeta(uint16_t offset, const SlotMetaV2& m);
    bool     writeHostSlot(const uint8_t* frame, uint16_t len);
    int      readDevSlot(uint8_t* buf, uint16_t bufSize);

    uint8_t  nextTxn();
    uint16_t buildFrame(uint8_t* buf, uint8_t func, const uint8_t* payload, uint16_t plen, uint8_t txn = 0);
    bool     sendCmd(uint8_t func, const uint8_t* payload = nullptr, uint16_t plen = 0);
    bool     safeSend(uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label);

    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleHeartbeat(const uint8_t* payload, uint16_t plen);
    void handleColor(const uint8_t* payload, uint16_t plen);
    void runStateMachine(uint8_t mode, uint8_t status);
    void sendColorTarget();
};

extern ColorGrabber colorGrabber;

#endif
