#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

#include "Arduino.h"
#include <stdint.h>
#include <functional>

#define FRAME_HEADER_1 0xFF
#define FRAME_HEADER_2 0xFF
#define MAX_ARGS_SIZE 250

#pragma pack(1)
typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    union {
        struct {
            uint8_t id;
            uint8_t length;
            uint8_t cmd;
            uint8_t args[MAX_ARGS_SIZE];
        } elements;
        uint8_t data_raw[MAX_ARGS_SIZE + 3];
    };
} PacketTypeDef;
#pragma pack()

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
    ERROR_CHEAKSUM
};

using ProtocolSuccessCallback = std::function<void(PacketTypeDef* self)>;
using ProtocolErrorCallback = std::function<void(void)>;

class CommProtocol_t {
public:
    ErrorState error_state;
    PacketTypeDef tx_packet;
    PacketTypeDef rx_packet;

    void begin(void);
    void parsing(uint8_t *data, uint16_t len);
    uint8_t tx_packet_complete(uint8_t id, uint8_t cmd, uint8_t* data, uint8_t data_len);
    void register_success_callback(ProtocolSuccessCallback cb);
    void register_error_callback(ProtocolErrorCallback cb);

private:
    ParsingState parsing_state; 
    ProtocolSuccessCallback successCallback;
    ProtocolErrorCallback errorCallback;
};

#endif