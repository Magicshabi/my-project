#include "CommProtocol.h"

static uint8_t checksum_crc8(const uint8_t *data, uint8_t len) {
    uint16_t temp = 0;
    for (int i = 0; i < len; ++i) {
        temp += data[i];
    }
    return (uint8_t)(~temp);
}

void CommProtocol_t::begin() {
    error_state = ERROR_NULL;
    parsing_state = PARSING_HEADER_1;
    successCallback = nullptr;
    errorCallback = nullptr;
}

void CommProtocol_t::register_success_callback(ProtocolSuccessCallback cb) {
    successCallback = cb;
}

void CommProtocol_t::register_error_callback(ProtocolErrorCallback cb) {
    errorCallback = cb;
}

uint8_t CommProtocol_t::tx_packet_complete(uint8_t id, uint8_t cmd, uint8_t* data, uint8_t data_len) {
    uint8_t frame_len = 6 + data_len;
    tx_packet.header_1 = FRAME_HEADER_1;
    tx_packet.header_2 = FRAME_HEADER_2;
    tx_packet.elements.id = id;
    tx_packet.elements.length = 2 + data_len;
    tx_packet.elements.cmd = cmd;
    for(uint8_t i = 0; i < data_len; i++) {
        tx_packet.elements.args[i] = data[i];
    }
    tx_packet.elements.args[data_len] = checksum_crc8(tx_packet.data_raw, tx_packet.elements.length + 1);
    return frame_len;
}

void CommProtocol_t::parsing(uint8_t *data, uint16_t len) {
    static uint8_t arg_count = 0;
    error_state = ERROR_NULL;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t current_byte = data[i];
        switch(parsing_state) {
            case PARSING_HEADER_1:
                if(current_byte == FRAME_HEADER_1) parsing_state = PARSING_HEADER_2;
                break;
            case PARSING_HEADER_2:
                if(current_byte == FRAME_HEADER_2) parsing_state = PARSING_ID;
                else parsing_state = PARSING_HEADER_1;
                break;
            case PARSING_ID: 
                rx_packet.elements.id = current_byte;
                parsing_state = PARSING_DATA_LENGTH;
                break;
            case PARSING_DATA_LENGTH:
                rx_packet.elements.length = current_byte;
                if(current_byte >= 2 && current_byte <= MAX_ARGS_SIZE) parsing_state = PARSING_CMD;
                else parsing_state = PARSING_HEADER_1;
                break;
            case PARSING_CMD:
                rx_packet.elements.cmd = current_byte;
                arg_count = 0;
                if(rx_packet.elements.length == 2) parsing_state = PARSING_CHECKSUM;
                else parsing_state = PARSING_ARGS;
                break;
            case PARSING_ARGS:
                rx_packet.elements.args[arg_count++] = current_byte;
                if(arg_count == rx_packet.elements.length - 2) parsing_state = PARSING_CHECKSUM;
                break;
            case PARSING_CHECKSUM: 
                uint8_t check = checksum_crc8(rx_packet.data_raw, rx_packet.elements.length + 1);
                if(check == current_byte) {
                    /* 必须在阻塞型 successCallback（如动作组播放）返回前复位状态，
                     * 否则回调内 rec_handler 再次进入 parsing 时仍停留在 PARSING_CHECKSUM，
                     * 会把后续 STOP 帧首字节误当作校验和，导致停止无效。 */
                    parsing_state = PARSING_HEADER_1;
                    if(successCallback) successCallback(&rx_packet);
                } else {
                    if(errorCallback) errorCallback();
                    parsing_state = PARSING_HEADER_1;
                }
                break;
        }
    }
}