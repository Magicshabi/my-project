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
    static constexpr uint8_t PLACE_SLOTS = 8;

    float kp = 0.08f;
    float kd = 0.01f;

    void begin();
    void start(const char* colors[], uint8_t count);
    /** 带放置槽位的序列抓取：slots[i] = 第 i 个颜色放到哪个槽位(0..7)。
     *  约定：槽位 0 留给任务一 的 T0；槽位 1/2/3 留给任务二 序列区 1/2/3 号位。 */
    void startSeq(const char* colors[], const uint8_t slots[], uint8_t count);
    void stop();
    void update();
    void setOffsets(float x_off, float y_off, float z_grab);
    /** 夹取阶段 Z 修正（与 setOffsets 的 z_grab 相加，负值表示末端更低） */
    void setGrabZAdjust(float delta_z) { _grab_z_adjust = delta_z; }
    float getGrabZAdjust() const { return _grab_z_adjust; }

    // ── 放置槽位坐标表（可掉电保存）──
    void     setPlaceSlot(uint8_t slot, float x, float y, float z);
    bool     getPlaceSlot(uint8_t slot, float* x, float* y, float* z) const;
    float*   placeTableRaw() { return &_place_tab[0][0]; }
    uint16_t placeTableBytes() const { return (uint16_t)sizeof(_place_tab); }
    uint8_t  getPlaceSlotOf(uint8_t colorIdx) const;

    // ── 抓取源坐标表 + 视觉失败兜底 ──
    // 底图把“颜色→格位”印死了（电池A=红 B=黄 C=蓝 D=绿），所以每个颜色都有固定抓取点。
    // 示教的坐标含义 = **夹爪夹住电池时末端应该在的 XY**（即抓取点，不是“相机居中的位置”）。
    static constexpr uint8_t PICK_COLORS = 4;      // red, green, blue, yellow
    enum PickMode : uint8_t {
        PICK_MODE_AUTO   = 0,   // 视觉优先；视觉超时后自动降级到示教坐标（默认）
        PICK_MODE_COORD  = 1,   // 强制走示教坐标，完全不依赖 K230
        PICK_MODE_VISION = 2    // 强制只用视觉，不降级
    };

    static int8_t colorIndexFromName(const char* name);   // red=0 green=1 blue=2 yellow=3，未知=-1

    void    setPickMode(uint8_t m) { _pick_mode = (m > (uint8_t)PICK_MODE_VISION) ? (uint8_t)PICK_MODE_AUTO : m; }
    uint8_t getPickMode() const { return _pick_mode; }
    bool    setPickSlot(const char* colorName, float x, float y, float z);
    bool    getPickSlot(uint8_t idx, float* x, float* y, float* z) const;
    bool    isPickValid(uint8_t idx) const { return idx < PICK_COLORS && _pick_valid[idx] != 0; }
    float*   pickTableRaw()   { return &_pick_tab[0][0]; }
    uint16_t pickTableBytes() const { return (uint16_t)sizeof(_pick_tab); }
    uint8_t* pickValidRaw()   { return _pick_valid; }
    uint16_t pickValidBytes() const { return (uint16_t)sizeof(_pick_valid); }
    bool    isCoordMode() const { return _coord_mode; }

    /** 视觉连续无检测超过这个毫秒数就降级到坐标（默认 8000） */
    unsigned long visionFallbackMs = 8000;

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
    static constexpr float    BLOCK_H = 30.0f;   // 兼容保留；放置已不再叠高度

    // ── 放置槽位表 ──
    // 标定前 8 个槽位默认全部等于厂商 demo 的单一放置点 (200,-200,60)，
    // 这样未标定时行为与原固件一致，标定一个改一个。
    static constexpr float PLACE_DEF_X = 200.0f;
    static constexpr float PLACE_DEF_Y = -200.0f;
    static constexpr float PLACE_DEF_Z = 60.0f;
    float   _place_tab[PLACE_SLOTS][3];   // [slot][0..2] = x, y, z
    uint8_t _place_slot_of[8];            // 第 i 个颜色使用哪个槽位
    float   _cur_place_x = 200.0f;   // 本轮实际使用的放置点
    float   _cur_place_y = -200.0f;
    float   _cur_place_z = 60.0f;

    // ── 抓取源坐标表 ──
    float   _pick_tab[PICK_COLORS][3];    // [颜色][0..2] = 抓取点 x, y, z
    uint8_t _pick_valid[PICK_COLORS];     // 是否已示教过；未示教则不启用兜底
    uint8_t _pick_mode = PICK_MODE_AUTO;
    bool    _coord_mode = false;          // 本轮是否走了坐标抓取
    float   _coord_grab_z = 60.0f;        // 坐标模式使用的抓取高度
    unsigned long _last_seen_time = 0;    // 上次看到目标颜色的时刻

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
    float _active_grab_x = S_X;
    float _active_grab_y = S_Y;
    float _active_grab_z = 60.0f;

    // Color sequence
    String _color_list[8];
    uint8_t _color_count = 0;
    uint8_t _color_index = 0;
    uint8_t _stack_count = 0;
    String _current_color;

    // Offsets (from calibration)
    float X_COMP = 0, Y_COMP = 0, GRAB_Z = 60.0f;
    /** 仅用于 CG_STATE_DOWN / CG_STATE_GRAB 的 Z 叠加量（默认略负 = 夹得更低） */
    float _grab_z_adjust = -12.0f;

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
