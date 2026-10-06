#ifndef TEACH_CAPTURE_H
#define TEACH_CAPTURE_H
#include "HandleTeachRecord.h"

// Five separate command-65 replies are required. This validates stability of
// AT32 feedback, not servo power, torque, mechanical limits or path clearance.
class TeachCapture {
public:
    uint8_t count = 0;
    bool failed = false;
    int16_t low[6] = {}, high[6] = {};
    int32_t sum[6] = {};
    void reset() { *this = TeachCapture(); }
    bool add(const int16_t values[6]) {
        if (failed || count >= 5) return false;
        for (uint8_t j = 0; j < 6; ++j) {
            if (values[j] < 0 || values[j] > 4095) { failed = true; return false; }
            if (count == 0) low[j] = high[j] = values[j];
            if (values[j] < low[j]) low[j] = values[j];
            if (values[j] > high[j]) high[j] = values[j];
            if (high[j] - low[j] > 8) { failed = true; return false; }
        }
        for (uint8_t j = 0; j < 6; ++j) sum[j] += values[j];
        ++count;
        return true;
    }
    bool result(int16_t out[6]) const {
        if (failed || count != 5) return false;
        for (uint8_t j = 0; j < 6; ++j) out[j] = sum[j] / count;
        return true;
    }
};

inline uint8_t first_missing_slot(const HandleTeachRecord& record) {
    for (uint8_t s = 0; s < 8; ++s)
        if (!(record.valid_mask & (1u << s))) return s;
    return 8;
}

inline bool make_teach_candidate(const HandleTeachRecord& previous, uint8_t slot,
                                 const int16_t values[6], HandleTeachRecord& out) {
    if (!valid_handle_record(previous) || slot >= 8 ||
        slot > first_missing_slot(previous)) return false;
    HandleTeachRecord candidate = previous;
    candidate.valid_mask = static_cast<uint8_t>((1u << (slot + 1)) - 1u);
    memcpy(candidate.positions[slot], values, sizeof(candidate.positions[slot]));
    for (uint8_t s = slot + 1; s < 8; ++s)
        memset(candidate.positions[s], 0, sizeof(candidate.positions[s]));
    candidate.checksum = handle_record_checksum(candidate);
    if (!valid_handle_record(candidate)) return false;
    out = candidate;
    return true;
}

inline bool teach_capture_self_test() {
    TeachCapture c;
    int16_t p[6] = {1000, 1500, 2000, 2500, 3000, 3500}, result[6];
    for (uint8_t i = 0; i < 4; ++i) if (!c.add(p)) return false;
    if (c.result(result) || !c.add(p) || !c.result(result) || memcmp(p, result, sizeof(p))) return false;
    c.reset(); c.add(p); p[3] += 9;
    if (c.add(p) || c.result(result)) return false;
    c.reset(); p[3] = 4096;
    if (c.add(p)) return false;
    p[3] = 2500;
    HandleTeachRecord r = new_handle_record(), out;
    if (make_teach_candidate(r, 1, p, out)) return false;
    for (uint8_t s = 0; s < 8; ++s) {
        if (!make_teach_candidate(r, s, p, out)) return false;
        r = out;
    }
    if (first_missing_slot(r) != 8 || !make_teach_candidate(r, 2, p, out)) return false;
    if (out.valid_mask != 7 || first_missing_slot(out) != 3) return false;
    for (uint8_t s = 3; s < 8; ++s)
        for (uint8_t j = 0; j < 6; ++j) if (out.positions[s][j]) return false;
    return true;
}
#endif
