#ifndef GESTURE_TRACKER_H
#define GESTURE_TRACKER_H

#include <Arduino.h>

enum GestureType {
    GESTURE_NONE = 0,
    GESTURE_GUN,
    GESTURE_OTHER,
    GESTURE_YEAH,
    GESTURE_FIVE,
    GESTURE_UNKNOWN
};

enum GT_State {
    GT_IDLE,
    GT_SET_MODE,
    GT_WAIT_READY,
    GT_CONFIG_PARAMS,
    GT_ENABLE_RUN,
    GT_RUNNING
};

class GestureTracker {
public:
    void begin();
    void start();
    void stop();
    void update();

    GestureType getLastGesture() const { return _lastGesture; }
    const char* getLastGestureStr() const { return _lastGestureStr; }

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

    static constexpr uint8_t APP_HAND_RECOGNITION = 10;

    bool _busy = false;
    uint8_t _txn = 0;
    uint16_t _host_gen = 0;
    uint16_t _dev_gen  = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;

    const float S_X = 200.0f;
    const float S_Y = 0.0f;
    const float S_Z = 200.0f;
    const float S_P = 0.0f;

    GestureType _lastGesture = GESTURE_NONE;
    char _lastGestureStr[32] = {0};
    GestureType _pendingGesture = GESTURE_NONE;
    uint8_t _pendingGestureCount = 0;

    GT_State _currentState = GT_IDLE;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
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

    // Frame parsing & gesture
    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleBbox(const uint8_t* payload, uint16_t plen);
    void handleHandKP(const uint8_t* payload, uint16_t plen);
    void handleDetectStr(const uint8_t* payload, uint16_t plen);
    void forceConfig();
    void handleGestureResult(const char* gesture_str);
    GestureType parseGestureString(const char* str, int len);
};

extern GestureTracker gestureTracker;

#endif
