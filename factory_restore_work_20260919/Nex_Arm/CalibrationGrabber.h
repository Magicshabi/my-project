#ifndef CALIBRATION_GRABBER_H
#define CALIBRATION_GRABBER_H

#include <Arduino.h>

enum CalibState {
  CB_STATE_INIT,
  CB_STATE_SET_MODE,
  CB_STATE_WAIT_READY,
  CB_STATE_CONFIG_PARAMS,
  CB_STATE_SEARCH,
  CB_STATE_DOWN,
  CB_STATE_GRAB,
  CB_STATE_LIFT,
  CB_STATE_HOLD,
  CB_STATE_RELEASE,
  CB_STATE_RESET
};

class CalibrationGrabber {
public:
    float kp = 0.03f;
    float kd = 0.001f;

    void begin();
    void start();
    void grab();
    void resetArm();
    void stop();
    void update();
    void setOffsets(float x_off, float y_off, float z_grab);

private:
    // Mailbox v2
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
    uint8_t _txn = 0;
    uint16_t _host_gen = 0;
    uint16_t _dev_gen  = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;

    float _last_ex = 0;
    float _last_ey = 0;
    float _last_ew = 0;
    float _f_tx = 0;
    float _f_ty = 200;
    float _f_tz = 200;
    int   _stableCount = 0;
    bool  _center_locked = false;
    unsigned long _last_move_time = 0;

    float _x_offset = 0.0f;
    float _y_offset = 0.0f;
    float _z_grab   = 60.0f;

    float X_COMP = 0.0f;
    float Y_COMP = 0.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;

    const float C_OPEN = -60.0f;
    const float C_CLOSE = 20.0f;

    const float S_P = -90.0f;
    const float S_X = 200.0f;
    const float S_Y = 0.0f;
    const float S_Z = 200.0f;

    CalibState _currentState = CB_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    unsigned long _ignore_data_until = 0;
    unsigned long _last_print_time = 0;
    unsigned long _last_i2c_time = 0;
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
    bool     safeSend(uint8_t cmd, const uint8_t* payload, uint16_t plen, const char* label);

    void forceConfig();
    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleHeartbeat(const uint8_t* payload, uint16_t plen);
    void handleBbox(const uint8_t* payload, uint16_t plen);
    void handleQuad(const uint8_t* payload, uint16_t plen);
    void runStateMachine(uint8_t mode, uint8_t status);
    void updateGrabStateMachine();
};

extern CalibrationGrabber calibrationGrabber;

#endif
