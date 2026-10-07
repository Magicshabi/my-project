#ifndef HANDLE_TEACH_RECORD_H
#define HANDLE_TEACH_RECORD_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Encoder positions retain all six joints, including wrist and gripper.
// These are taught endpoints, not proof of a collision-free replay trajectory.
struct HandleTeachRecord {
    uint32_t magic;
    uint16_t version;
    uint8_t valid_mask;
    uint8_t reserved;
    int16_t positions[8][6];
    uint32_t checksum;
};
static_assert(sizeof(HandleTeachRecord) == 108, "Unexpected teach record layout");

inline uint32_t handle_record_checksum(const HandleTeachRecord& record) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&record);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < offsetof(HandleTeachRecord, checksum); ++i) {
        crc ^= p[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}

inline HandleTeachRecord new_handle_record() {
    HandleTeachRecord r = {};
    r.magic = 0x484E4431u;
    r.version = 1;
    r.checksum = handle_record_checksum(r);
    return r;
}

inline bool valid_handle_record(const HandleTeachRecord& r) {
    if (r.magic != 0x484E4431u || r.version != 1 || r.reserved != 0 ||
        r.checksum != handle_record_checksum(r)) return false;
    // Only sequentially taught prefixes are accepted; gaps are not runnable.
    if ((static_cast<uint16_t>(r.valid_mask) &
         (static_cast<uint16_t>(r.valid_mask) + 1u)) != 0) return false;
    for (uint8_t slot = 0; slot < 8; ++slot) {
        if (!(r.valid_mask & (1u << slot))) continue;
        for (uint8_t joint = 0; joint < 6; ++joint)
            if (r.positions[slot][joint] < 0 || r.positions[slot][joint] > 4095)
                return false;
    }
    return true;
}

inline bool handle_record_self_test() {
    HandleTeachRecord r = new_handle_record();
    if (!valid_handle_record(r)) return false;
    r.valid_mask = 0xFF;
    for (uint8_t s = 0; s < 8; ++s)
        for (uint8_t j = 0; j < 6; ++j) r.positions[s][j] = 2048;
    r.checksum = handle_record_checksum(r);
    if (!valid_handle_record(r)) return false;
    r.positions[7][5] ^= 1;
    if (valid_handle_record(r)) return false; // corrupt persisted bytes
    r.positions[7][5] = 4096;
    r.checksum = handle_record_checksum(r);
    if (valid_handle_record(r)) return false; // valid CRC, invalid encoder
    r.positions[7][5] = 2048;
    r.valid_mask = 5;
    r.checksum = handle_record_checksum(r);
    if (valid_handle_record(r)) return false; // skipped teaching stage
    r.valid_mask = 0xFF;
    r.version = 2;
    r.checksum = handle_record_checksum(r);
    return !valid_handle_record(r); // unsupported future format
}
#endif
