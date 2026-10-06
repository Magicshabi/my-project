#ifndef REAL_JOINT_FEEDBACK_H
#define REAL_JOINT_FEEDBACK_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Factory SDK command 65: six (actual pulse:i16, angle_x10:i16) pairs.
inline bool decode_real_joint_positions(const uint8_t* data, size_t size, int16_t out[6]) {
    if (!data || !out || size != 24) return false;
    int16_t checked[6];
    for (uint8_t i = 0; i < 6; ++i) {
        checked[i] = static_cast<int16_t>(static_cast<uint16_t>(data[4*i]) |
                                        (static_cast<uint16_t>(data[4*i+1]) << 8));
        if (checked[i] < 0 || checked[i] > 4095) return false;
    }
    memcpy(out, checked, sizeof(checked));
    return true;
}

inline bool real_joint_feedback_self_test() {
    const int16_t expected[6] = {0, 511, 1024, 2048, 3000, 4095};
    uint8_t data[24] = {};
    for (uint8_t i = 0; i < 6; ++i) {
        data[4*i] = expected[i] & 255;
        data[4*i+1] = expected[i] >> 8;
        data[4*i+2] = 0xEA; data[4*i+3] = 0xFF;
    }
    int16_t result[6] = {};
    if (!decode_real_joint_positions(data, sizeof(data), result) ||
        memcmp(result, expected, sizeof(result))) return false;
    if (decode_real_joint_positions(data, 12, result) ||
        decode_real_joint_positions(data, 23, result) ||
        decode_real_joint_positions(data, 25, result)) return false;
    data[20] = 0; data[21] = 0x10;
    if (decode_real_joint_positions(data, sizeof(data), result)) return false;
    if (memcmp(result, expected, sizeof(result))) return false;
    data[20] = 0xFF; data[21] = 0xFF;
    return !decode_real_joint_positions(data, sizeof(data), result);
}
#endif
