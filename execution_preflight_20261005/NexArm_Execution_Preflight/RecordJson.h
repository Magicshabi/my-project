#ifndef RECORD_JSON_H
#define RECORD_JSON_H
#include "HandleTeachRecord.h"
#include <stdio.h>

static constexpr size_t RECORD_HEX_SIZE = sizeof(HandleTeachRecord) * 2 + 1;
static constexpr size_t RECORD_JSON_SIZE = 512;

inline size_t format_record_json(const HandleTeachRecord& record, const char* version,
                                const char* ns, bool storage_fault, char* out, size_t capacity) {
    char hex[RECORD_HEX_SIZE];
    static const char digits[] = "0123456789abcdef";
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
    for (size_t i = 0; i < sizeof(record); ++i) {
        hex[2*i] = digits[bytes[i] >> 4];
        hex[2*i+1] = digits[bytes[i] & 15];
    }
    hex[RECORD_HEX_SIZE - 1] = '\0';
    int n = snprintf(out, capacity,
        "{\"type\":\"teach_record\",\"firmware\":\"%s\",\"namespace\":\"%s\",\"valid_mask\":%u,\"storage_fault\":%s,\"record_hex\":\"%s\",\"playback_enabled\":false}\n",
        version, ns, record.valid_mask, storage_fault ? "true" : "false", hex);
    if (n < 0 || static_cast<size_t>(n) >= capacity) {
        if (capacity) out[0] = '\0';
        return 0;
    }
    return static_cast<size_t>(n);
}
#endif
