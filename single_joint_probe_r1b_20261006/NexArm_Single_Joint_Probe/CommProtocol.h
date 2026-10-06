#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>
#include <functional>

#define FRAME_HEADER_1 0xFF
#define FRAME_HEADER_2 0xFF
// length on wire counts command + payload + checksum, not the whole frame.
static constexpr uint16_t MAX_ARGS_SIZE = 250;
static constexpr uint16_t WIRE_LENGTH_OVERHEAD = 2;
static constexpr uint16_t FRAME_OVERHEAD = 6;
static constexpr uint16_t MAX_WIRE_LENGTH = MAX_ARGS_SIZE + WIRE_LENGTH_OVERHEAD;
static constexpr uint16_t MAX_FRAME_SIZE = MAX_ARGS_SIZE + FRAME_OVERHEAD;
static_assert(MAX_WIRE_LENGTH <= UINT8_MAX, "Wire length field overflow");

#pragma pack(1)
typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    union {
        struct {
            uint8_t id;
            uint8_t length;
            uint8_t cmd;
            uint8_t args[MAX_ARGS_SIZE + 1]; // payload followed by checksum
        } elements;
        uint8_t data_raw[MAX_ARGS_SIZE + 4]; // id, length, command, args, checksum
    };
} PacketTypeDef;
#pragma pack()
static_assert(sizeof(PacketTypeDef) == MAX_FRAME_SIZE, "Packet capacity mismatch");

enum ParsingState {
    PARSING_HEADER_1 = 0,
    PARSING_HEADER_2,
    PARSING_ID,
    PARSING_CMD,
    PARSING_DATA_LENGTH,
    PARSING_ARGS,
    PARSING_CHECKSUM
};

enum ErrorState {
    ERROR_NULL = 0,
    ERROR_FRAME_HEADER,
    ERROR_FRAME_LEN,
    ERROR_CHECKSUM,
    ERROR_PARTIAL_TIMEOUT
};

struct ProtocolStats {
    uint32_t bytes = 0, valid_frames = 0, errors = 0;
    uint32_t length_errors = 0, checksum_errors = 0, partial_timeouts = 0;
    uint32_t noise_bytes = 0, consecutive_errors = 0;
};

using ProtocolSuccessCallback = std::function<void(PacketTypeDef* self)>;
using ProtocolErrorCallback = std::function<void(void)>;

class CommProtocol_t {
public:
    ErrorState error_state = ERROR_NULL; // retained until next valid frame
    ProtocolStats stats;
    PacketTypeDef tx_packet;
    PacketTypeDef rx_packet;

    void begin(void);
    void parsing(uint8_t *data, uint16_t len);
    uint16_t tx_packet_complete(uint8_t id, uint8_t cmd, const uint8_t* data, size_t data_len);
    bool has_partial_frame() const { return parsing_state != PARSING_HEADER_1; }
    void discard_partial_frame();
    void register_success_callback(ProtocolSuccessCallback cb);
    void register_error_callback(ProtocolErrorCallback cb);

private:
    ParsingState parsing_state = PARSING_HEADER_1;
    uint16_t arg_count = 0;
    ProtocolSuccessCallback successCallback;
    ProtocolErrorCallback errorCallback;
    void note_error(ErrorState error);
};

#endif
