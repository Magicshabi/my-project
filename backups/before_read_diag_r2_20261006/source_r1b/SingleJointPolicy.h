#ifndef SINGLE_JOINT_POLICY_H
#define SINGLE_JOINT_POLICY_H
#include "HandleTeachRecord.h"
static constexpr int16_t OPEN_STEP = 8;
static constexpr uint8_t TEST_ID = 6;

// Taught gripper ranges only constrain this experiment; they are not factory
// limits. Larger pulses meant closing in both user-taught grip/release pairs.
inline bool make_open_probe(const HandleTeachRecord& r, const int16_t current[6],
                            int16_t& goal, uint8_t (&args)[8]) {
    if (!current || !valid_handle_record(r) || r.valid_mask != 255) return false;
    if (r.positions[2][5] - r.positions[1][5] < 32 ||
        r.positions[5][5] - r.positions[6][5] < 32) return false;
    for (uint8_t i = 0; i < 6; ++i) if (current[i] < 0 || current[i] > 4095) return false;
    int16_t low = 4095, high = 0;
    for (uint8_t s = 0; s < 8; ++s) {
        if (r.positions[s][5] < low) low = r.positions[s][5];
        if (r.positions[s][5] > high) high = r.positions[s][5];
    }
    if (current[5] - OPEN_STEP < low + 16 || current[5] > high - 16) return false;
    goal = current[5] - OPEN_STEP;
    // Local factory write_pos_ex: ACC at 41, then position, PWM=0, speed=30.
    const uint8_t packet[] = {41, 5, static_cast<uint8_t>(goal), static_cast<uint8_t>(goal >> 8), 0, 0, 30, 0};
    memcpy(args, packet, 8);
    return true;
}
inline bool allowed_single_tx(uint8_t id, uint8_t cmd, const uint8_t* args, size_t n,
                              bool authorized, int16_t goal) {
    if (id == 255 && cmd == 65 && n == 0) return true;
    return authorized && goal >= 0 && goal <= 4095 && id == TEST_ID && cmd == 3 &&
        args && n == 8 && args[0] == 41 && args[1] == 5 &&
        args[2] == (goal & 255) && args[3] == (goal >> 8) &&
        args[4] == 0 && args[5] == 0 && args[6] == 30 && args[7] == 0;
}
inline bool single_joint_policy_test() {
    HandleTeachRecord r = new_handle_record(); r.valid_mask = 255;
    for (uint8_t s = 0; s < 8; ++s)
        for (uint8_t j = 0; j < 6; ++j) r.positions[s][j] = 1400;
    r.positions[2][5] = r.positions[5][5] = 1900;
    r.positions[6][5] = 1100;
    r.checksum = handle_record_checksum(r);
    int16_t current[] = {2000,2000,2000,2000,2000,1400};
    int16_t goal = -1; uint8_t args[8] = {};
    if (!make_open_probe(r, current, goal, args) || goal != 1392) return false;
    for (unsigned id = 0; id < 256; ++id)
        for (unsigned cmd = 0; cmd < 256; ++cmd)
            if (allowed_single_tx(id, cmd, args, 8, true, goal) != (id == 6 && cmd == 3)) return false;
    if (allowed_single_tx(6,3,args,8,false,goal) || allowed_single_tx(6,3,args,8,true,goal+1) ||
        allowed_single_tx(6,3,args,7,true,goal) || allowed_single_tx(6,3,nullptr,8,true,goal)) return false;
    for (unsigned i = 0; i < 8; ++i) {
        args[i] ^= 1;
        if (allowed_single_tx(6,3,args,8,true,goal)) return false;
        args[i] ^= 1;
    }
    current[5] = 1123;
    if (make_open_probe(r,current,goal,args)) return false;
    current[5] = 1124;
    if (!make_open_probe(r,current,goal,args)) return false;
    r.positions[2][5] = 1200; r.checksum = handle_record_checksum(r);
    return !make_open_probe(r,current,goal,args);
}
#endif
