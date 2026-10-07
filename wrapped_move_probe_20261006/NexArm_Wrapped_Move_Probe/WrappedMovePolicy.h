#ifndef WRAPPED_MOVE_POLICY_H
#define WRAPPED_MOVE_POLICY_H
#include <stdint.h>
#include <stddef.h>

// r5 policy: only wrapped (id=0xFF) frames may leave the ESP32 UART.
// No raw servo frames (id 1..6, cmd 3) are permitted in any phase.
// Motion is a single incremental wrapped move dz=+30mm over 3000ms and,
// only after upward motion was observed, one return move dz=-30mm.
enum WMPermit : uint8_t { WM_NONE = 0, WM_UP_ONCE = 2, WM_DOWN_RETURN = 3 };

inline bool wrapped_tx_allowed(uint8_t id, uint8_t cmd, const uint8_t* a, size_t n, WMPermit permit) {
    // Zero-arg wrapped queries are always allowed: 65 = joint positions, 11 = current coords.
    if (id == 255 && (cmd == 65 || cmd == 11) && n == 0) return true;
    // Everything else must be exactly one permitted CMD_ARM_MOVE_INC (cmd 50) frame.
    if (id != 255 || cmd != 50 || !a || n != 14) return false;
    if (permit != WM_UP_ONCE && permit != WM_DOWN_RETURN) return false;
    // Payload layout: dX, dY, dZ, dPitch, dRoll, dClaw, Time  (int16 LE each)
    // Fixed: dX=dY=dPitch=dRoll=dClaw=0, Time=3000ms; only dZ differs by permit.
    int16_t dz = (int16_t)((uint16_t)a[4] | ((uint16_t)a[5] << 8));
    if (permit == WM_UP_ONCE && dz != 30) return false;
    if (permit == WM_DOWN_RETURN && dz != -30) return false;
    for (unsigned i = 0; i < 14; ++i) {
        if (i == 4 || i == 5) continue;
        uint8_t expect = 0;
        if (i == 12) expect = 0xB8;  // 3000 = 0x0BB8 low byte
        if (i == 13) expect = 0x0B;  // high byte
        if (a[i] != expect) return false;
    }
    return true;
}

inline bool wrapped_move_policy_test() {
    uint8_t up[14]   = {0,0, 0,0, 30,0,   0,0, 0,0, 0,0, 0xB8,0x0B};
    uint8_t down[14] = {0,0, 0,0, 0xE2,0xFF, 0,0, 0,0, 0,0, 0xB8,0x0B};
    // Queries: any id other than 255 rejected; arg-carrying query rejected.
    for (unsigned id = 0; id < 256; ++id) {
        if (wrapped_tx_allowed((uint8_t)id, 65, nullptr, 0, WM_NONE) != (id == 255)) return false;
        if (wrapped_tx_allowed((uint8_t)id, 11, nullptr, 0, WM_NONE) != (id == 255)) return false;
    }
    if (wrapped_tx_allowed(255, 65, up, 1, WM_NONE)) return false;
    // Only cmd 50 with the right permit passes; every other id/cmd rejected.
    for (unsigned id = 0; id < 256; ++id)
        for (unsigned cmd = 0; cmd < 256; ++cmd) {
            if (cmd == 65 || cmd == 11) continue;  // zero-arg form checked above; with args must fail
            bool want_up = (id == 255 && cmd == 50);
            if (wrapped_tx_allowed((uint8_t)id, (uint8_t)cmd, up, 14, WM_UP_ONCE) != want_up) return false;
            if (wrapped_tx_allowed((uint8_t)id, (uint8_t)cmd, down, 14, WM_DOWN_RETURN) != want_up) return false;
        }
    // Permit/payload mismatches must fail.
    if (wrapped_tx_allowed(255, 50, up, 14, WM_DOWN_RETURN)) return false;
    if (wrapped_tx_allowed(255, 50, down, 14, WM_UP_ONCE)) return false;
    if (wrapped_tx_allowed(255, 50, up, 14, WM_NONE)) return false;
    if (wrapped_tx_allowed(255, 50, up, 13, WM_UP_ONCE)) return false;
    if (wrapped_tx_allowed(255, 50, nullptr, 14, WM_UP_ONCE)) return false;
    // Any single-byte corruption of a valid frame must be rejected.
    for (unsigned i = 0; i < 14; ++i) {
        up[i] ^= 1;
        if (wrapped_tx_allowed(255, 50, up, 14, WM_UP_ONCE)) return false;
        up[i] ^= 1;
    }
    // dz magnitude other than exactly ±30 must be rejected.
    up[4] = 31;
    if (wrapped_tx_allowed(255, 50, up, 14, WM_UP_ONCE)) return false;
    up[4] = 29;
    if (wrapped_tx_allowed(255, 50, up, 14, WM_UP_ONCE)) return false;
    up[4] = 5;  // r4 value must no longer pass
    if (wrapped_tx_allowed(255, 50, up, 14, WM_UP_ONCE)) return false;
    return true;
}
#endif
