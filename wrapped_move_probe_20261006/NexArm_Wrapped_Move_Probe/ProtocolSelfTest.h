#ifndef PROTOCOL_SELF_TEST_H
#define PROTOCOL_SELF_TEST_H
#include "CommProtocol.h"
#include "RecordJson.h"

inline bool protocol_boundary_self_test() {
    CommProtocol_t tx, rx;
    tx.begin(); rx.begin();
    uint8_t payload[MAX_ARGS_SIZE] = {};
    for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = (i * 17) & 255;
    bool matched = false;
    uint16_t expected = 0;
    rx.register_success_callback([&](PacketTypeDef* p) {
        matched = p->elements.id == 0x5A && p->elements.cmd == 65 &&
            p->elements.length == expected + WIRE_LENGTH_OVERHEAD &&
            memcmp(p->elements.args, payload, expected) == 0;
    });
    const uint16_t lengths[] = {0, 24, 108, 248, 249, 250};
    for (uint16_t size : lengths) {
        expected = size; matched = false;
        uint16_t n = tx.tx_packet_complete(0x5A, 65, payload, size);
        if (n != size + FRAME_OVERHEAD) return false;
        // Fragment into single bytes, including the 256-byte maximum frame.
        for (uint16_t i = 0; i < n; ++i)
            rx.parsing(reinterpret_cast<uint8_t*>(&tx.tx_packet) + i, 1);
        if (!matched || rx.error_state != ERROR_NULL) return false;
    }
    if (tx.tx_packet_complete(1, 65, payload, 251) || tx.tx_packet_complete(1, 65, nullptr, 1)) return false;
    uint16_t n = tx.tx_packet_complete(0x5A, 65, payload, 24);
    uint8_t* frame = reinterpret_cast<uint8_t*>(&tx.tx_packet);
    frame[n - 1] ^= 1;
    rx.parsing(frame, n);
    if (rx.error_state != ERROR_CHECKSUM || rx.stats.checksum_errors != 1) return false;
    uint8_t invalid_length[] = {255, 255, 90, 253};
    rx.parsing(invalid_length, sizeof(invalid_length));
    if (rx.error_state != ERROR_FRAME_LEN || rx.stats.consecutive_errors != 2) return false;
    n = tx.tx_packet_complete(0x5A, 65, payload, 24);
    rx.parsing(frame, 10);
    rx.discard_partial_frame();
    if (rx.stats.partial_timeouts != 1) return false;
    matched = false; expected = 24;
    rx.parsing(frame, n);
    if (!matched || rx.error_state != ERROR_NULL || rx.stats.consecutive_errors != 0) return false;
    char line[RECORD_JSON_SIZE];
    HandleTeachRecord r = new_handle_record();
    size_t used = format_record_json(r, "nexarm-teach-record-r7", "handle_free", false, line, sizeof(line));
    if (!used || line[used - 1] != '\n' || line[used] != '\0') return false;
    char tiny[8];
    return format_record_json(r, "r7", "handle_free", false, tiny, sizeof(tiny)) == 0 && tiny[0] == '\0';
}
#endif
