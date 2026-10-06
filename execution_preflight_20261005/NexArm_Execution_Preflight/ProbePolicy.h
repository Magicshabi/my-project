#ifndef PROBE_POLICY_H
#define PROBE_POLICY_H
#include <stdint.h>
#include <stddef.h>

// Read-only local HX_30HM layout candidates; these are not confirmed hardware
// register semantics until the returned identity and values are reviewed.
static constexpr uint8_t PROBE_ID = 6;
static constexpr uint8_t READ_REGS[4] = {3, 33, 40, 56};
static constexpr uint8_t READ_SIZES[4] = {3, 1, 8, 2};
inline bool allowed_probe_tx(uint8_t id, uint8_t cmd, const uint8_t* a, size_t n) {
    if (id == 0xFF && cmd == 65 && n == 0) return true;
    if (id != PROBE_ID || cmd != 2 || !a || n != 2) return false;
    for (uint8_t i = 0; i < 4; ++i)
        if (a[0] == READ_REGS[i] && a[1] == READ_SIZES[i]) return true;
    return false;
}
inline bool probe_policy_self_test() {
    if (!allowed_probe_tx(255, 65, nullptr, 0) || allowed_probe_tx(6, 2, nullptr, 2)) return false;
    for (unsigned id = 0; id < 256; ++id) {
        for (unsigned cmd = 0; cmd < 256; ++cmd) {
            const uint8_t data[] = {40, 8};
            if (allowed_probe_tx(id, cmd, data, 2) != (id == 6 && cmd == 2)) return false;
        }
    }
    for (unsigned reg = 0; reg < 256; ++reg) {
        for (unsigned size = 0; size < 256; ++size) {
            const uint8_t data[] = {static_cast<uint8_t>(reg), static_cast<uint8_t>(size)};
            bool expected = false;
            for (uint8_t i = 0; i < 4; ++i) expected |= reg == READ_REGS[i] && size == READ_SIZES[i];
            if (allowed_probe_tx(6, 2, data, 2) != expected) return false;
        }
    }
    const uint8_t data[] = {40, 8, 1};
    return !allowed_probe_tx(6, 2, data, 1) && !allowed_probe_tx(6, 2, data, 3);
}
#endif
