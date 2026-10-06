#ifndef AILLM_CONTROL_H
#define AILLM_CONTROL_H

#include <Arduino.h>

enum MCP_GrabState : uint8_t {
    LG_IDLE = 0,
    LG_TRACKING,
    LG_DONE
};

class AiLLMControl {
public:
    void begin();
    void start();
    void stop();
    void update();

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
    static constexpr uint16_t MAX_FRAME_SIZE = 2048;
    static constexpr uint8_t  FRAME_H0 = 0xAA;
    static constexpr uint8_t  FRAME_H1 = 0x55;
    static constexpr uint8_t  CTRL_CONT = 0x20;
    static constexpr uint8_t  CTRL_SEQ_MASK = 0x1F;

    static constexpr uint8_t CMD_SET_MCP_TOOLS  = 0x6C;
    static constexpr uint8_t CMD_RESULT_RETURN  = 0x6D;
    static constexpr uint8_t RPT_HEARTBEAT      = 0x70;
    static constexpr uint16_t DEBUG_DUMP_LIMIT  = 256;

    float _cur_x = 200.0f;
    float _cur_y = 0.0f;
    float _cur_z = 200.0f;
    float _cur_pitch = 0.0f;
    float _cur_roll = 0.0f;
    float _cur_claw = -60.0f;

    bool _busy = false;
    bool _auto_start_pending = false;
    uint8_t _start_attempts = 0;
    uint8_t _txn = 0;
    uint8_t _seq = 0;
    uint16_t _host_gen = 0;
    uint16_t _dev_gen  = 0;
    uint16_t _slot_size = 0;
    uint16_t _host_slot_data_offset = 0;
    uint16_t _dev_slot_data_offset  = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_i2c_time = 0;
    unsigned long _last_start_attempt_time = 0;
    uint8_t _last_hb_mode = 0xFF;
    uint8_t _last_hb_status = 0;

    // Grab state machine
    MCP_GrabState _grab_state = LG_IDLE;
    unsigned long _grab_state_time = 0;
    void updateGrab();

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
    uint8_t  nextSeq();
    uint16_t buildFrame(uint8_t* buf, uint8_t func, const uint8_t* payload, uint16_t plen, uint8_t txn, uint8_t seq, bool continuation);
    bool     sendCmd(uint8_t func, const uint8_t* payload = nullptr, uint16_t plen = 0);

    // Frame parsing & MCP
    void parseFrame(const uint8_t* frame, uint16_t len);
    void handleHeartbeat(const uint8_t* payload, uint16_t plen);
    void handleResultRSP(const uint8_t* payload, uint16_t plen);
    void setupMCPTools();
    bool sendMCPResult(const char* tool, bool ok, const char* result);
    bool executeAction(const uint8_t* data, uint16_t len);
    bool executeGrabColor(const uint8_t* data, uint16_t len);
    bool executeTrackColor(const uint8_t* data, uint16_t len);
    void debugDumpBytes(const char* label, const uint8_t* data, uint16_t len, uint16_t limit = DEBUG_DUMP_LIMIT);
    void debugDumpDataPackStrings(const uint8_t* data, uint16_t len);

    // RX fragment reassembly
    bool _rx_frag_active = false;
    uint8_t _rx_frag_type = 0;
    uint8_t _rx_frag_func = 0;
    uint8_t _rx_frag_txn = 0;
    uint8_t _rx_frag_expected_seq = 0;
    uint16_t _rx_frag_len = 0;
    uint8_t _rx_frag_buf[MAX_FRAME_SIZE];
    void resetRxFragment();
};

extern AiLLMControl aiLLMControl;

#endif
