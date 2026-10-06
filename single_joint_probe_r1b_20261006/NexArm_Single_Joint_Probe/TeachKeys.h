#ifndef TEACH_KEYS_H
#define TEACH_KEYS_H
#include <stdint.h>

// Board confirmed in the field: GPIO0/K1 active LOW, GPIO2/K2 active HIGH.
inline uint8_t teach_pressed_mask(uint8_t gpio0, uint8_t gpio2) {
    return (gpio0 == 0 ? 1 : 0) | (gpio2 != 0 ? 2 : 0);
}

// Electrical levels are normalized before reaching this state machine.
// Emit a short-click only on release. Chords and long holds have no action.
class TeachKeys {
    uint8_t raw = 0, stable = 0, pending = 0;
    bool armed = false, blocked = false;
    uint32_t changed = 0, pressed = 0;
public:
    uint32_t release_sequence = 0, release_duration_ms = 0;
    uint8_t released_key = 0, release_reason = 0;
    bool is_armed() const { return armed; }
    bool is_blocked() const { return blocked; }
    uint8_t stable_mask() const { return stable; }
    uint8_t pending_mask() const { return pending; }
    uint8_t update(uint8_t mask, uint32_t now) {
        if (mask != raw) { raw = mask; changed = now; }
        if (raw == 3 || (pending && raw && raw != pending)) blocked = true;
        if (static_cast<uint32_t>(now - changed) < 35) return 0;
        if (!armed) {
            if (!raw) { armed = true; stable = 0; blocked = false; }
            return 0;
        }
        if (stable == raw) return 0;
        stable = raw;
        if (stable) {
            if (!pending) { pending = stable; pressed = now; }
            return 0;
        }
        ++release_sequence;
        released_key = pending;
        release_duration_ms = now - pressed;
        // 0=click, 1=chord/overlap, 2=long hold, 3=no pending press.
        release_reason = blocked ? 1 : !pending ? 3 : release_duration_ms >= 1500 ? 2 : 0;
        uint8_t event = (!blocked && pending && now - pressed < 1500) ? pending : 0;
        pending = 0; blocked = false;
        return event;
    }
};

inline bool teach_keys_self_test() {
    TeachKeys k;
    if (k.update(1, 0) || k.update(1, 100) || k.update(0, 110) || k.update(0, 150)) return false;
    if (k.update(1, 200) || k.update(1, 240) || k.update(0, 300) || k.update(0, 320)) return false;
    if (k.update(0, 340) != 1 || k.update(0, 400)) return false;
    if (k.update(1, 500) || k.update(1, 540) || k.update(3, 600) || k.update(3, 640) ||
        k.update(2, 700) || k.update(2, 740) || k.update(0, 800) || k.update(0, 840)) return false;
    if (k.update(2, 900) || k.update(2, 940) || k.update(0, 2600) || k.update(0, 2640)) return false;
    if (k.update(2, 2700) || k.update(2, 2740) || k.update(0, 2800)) return false;
    if (k.update(0, 2840) != 2) return false;
    // Reproduce the reported order: K2 (Save first), then a normal K1 tap.
    if (k.update(1, 2900) || k.update(1, 2940) || k.update(0, 3100)) return false;
    return k.update(0, 3140) == 1 && k.release_reason == 0 &&
           k.released_key == 1 && k.release_duration_ms == 200;
}
inline bool teach_pin_levels_self_test() {
    if (teach_pressed_mask(1, 0) != 0 || teach_pressed_mask(0, 0) != 1 ||
        teach_pressed_mask(1, 1) != 2 || teach_pressed_mask(0, 1) != 3) return false;
    TeachKeys k;
    auto step = [&k](uint8_t g0, uint8_t g2, uint32_t ms) {
        return k.update(teach_pressed_mask(g0, g2), ms);
    };
    // Release both; K2 (Save first), then K1 (save). Cover real GPIO levels.
    if (step(1, 0, 0) || step(1, 0, 40) || !k.is_armed()) return false;
    if (step(1, 1, 100) || step(1, 1, 140) || step(1, 0, 600)) return false;
    if (step(1, 0, 640) != 2 || step(1, 0, 680)) return false;
    if (step(0, 0, 700) || step(0, 0, 740) || step(1, 0, 1200)) return false;
    if (step(1, 0, 1240) != 1 || k.release_reason != 0) return false;
    // A true overlap still must never produce a save event.
    if (step(1, 1, 1300) || step(1, 1, 1340) || step(0, 1, 1400) ||
        step(0, 1, 1440) || step(0, 0, 1500) || step(0, 0, 1540) ||
        step(1, 0, 1600) || step(1, 0, 1640)) return false;
    return k.release_reason == 1;
}
#endif
