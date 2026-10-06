#ifndef TEACH_KEYS_H
#define TEACH_KEYS_H
#include <stdint.h>

// Active-low pins are converted to a pressed-bit mask by the sketch.
// Emit a short-click only on release. Chords and long holds have no action.
class TeachKeys {
    uint8_t raw = 0, stable = 0, pending = 0;
    bool armed = false, blocked = false;
    uint32_t changed = 0, pressed = 0;
public:
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
    return k.update(0, 2840) == 2;
}
#endif
