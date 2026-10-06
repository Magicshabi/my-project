#include "CommProtocol.h"

static uint8_t checksum_crc8(const uint8_t *data, size_t len) {
    uint16_t temp = 0;
    for (size_t i = 0; i < len; ++i) {
        temp += data[i];
    }
    return (uint8_t)(~temp);
}

void CommProtocol_t::begin() {
    error_state = ERROR_NULL;
    stats = ProtocolStats();
    parsing_state = PARSING_HEADER_1;
    arg_count = 0;
    successCallback = nullptr;
    errorCallback = nullptr;
}

void CommProtocol_t::register_success_callback(ProtocolSuccessCallback cb) {
    successCallback = cb;
}

void CommProtocol_t::register_error_callback(ProtocolErrorCallback cb) {
    errorCallback = cb;
}

uint16_t CommProtocol_t::tx_packet_complete(uint8_t id, uint8_t cmd, const uint8_t* data, size_t data_len) {
    if (data_len > MAX_ARGS_SIZE || (data_len && !data)) return 0;
    uint16_t frame_len = FRAME_OVERHEAD + data_len;
    tx_packet.header_1 = FRAME_HEADER_1;
    tx_packet.header_2 = FRAME_HEADER_2;
    tx_packet.elements.id = id;
    tx_packet.elements.length = WIRE_LENGTH_OVERHEAD + data_len;
    tx_packet.elements.cmd = cmd;
    for(size_t i = 0; i < data_len; i++) {
        tx_packet.elements.args[i] = data[i];
    }
    tx_packet.elements.args[data_len] = checksum_crc8(tx_packet.data_raw, tx_packet.elements.length + 1);
    return frame_len;
}

void CommProtocol_t::note_error(ErrorState error) {
    error_state = error;
    ++stats.errors;
    ++stats.consecutive_errors;
    if (error == ERROR_FRAME_LEN) ++stats.length_errors;
    if (error == ERROR_CHECKSUM) ++stats.checksum_errors;
    if (error == ERROR_PARTIAL_TIMEOUT) ++stats.partial_timeouts;
    if (errorCallback) errorCallback();
}

void CommProtocol_t::discard_partial_frame() {
    if (!has_partial_frame()) return;
    parsing_state = PARSING_HEADER_1;
    arg_count = 0;
    note_error(ERROR_PARTIAL_TIMEOUT);
}

void CommProtocol_t::parsing(uint8_t *data, uint16_t len) {
    if (!data) return;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t current_byte = data[i];
        ++stats.bytes;
        switch(parsing_state) {
            case PARSING_HEADER_1:
                if(current_byte == FRAME_HEADER_1) parsing_state = PARSING_HEADER_2;
                else ++stats.noise_bytes;
                break;
            case PARSING_HEADER_2:
                if(current_byte == FRAME_HEADER_2) parsing_state = PARSING_ID;
                else { parsing_state = PARSING_HEADER_1; stats.noise_bytes += 2; }
                break;
            case PARSING_ID: 
                rx_packet.elements.id = current_byte;
                parsing_state = PARSING_DATA_LENGTH;
                break;
            case PARSING_DATA_LENGTH:
                rx_packet.elements.length = current_byte;
                if(current_byte >= WIRE_LENGTH_OVERHEAD && current_byte <= MAX_WIRE_LENGTH) parsing_state = PARSING_CMD;
                else { parsing_state = PARSING_HEADER_1; note_error(ERROR_FRAME_LEN); }
                break;
            case PARSING_CMD:
                rx_packet.elements.cmd = current_byte;
                arg_count = 0;
                if(rx_packet.elements.length == WIRE_LENGTH_OVERHEAD) parsing_state = PARSING_CHECKSUM;
                else parsing_state = PARSING_ARGS;
                break;
            case PARSING_ARGS:
                if (arg_count >= MAX_ARGS_SIZE ||
                    arg_count >= rx_packet.elements.length - WIRE_LENGTH_OVERHEAD) {
                    arg_count = 0;
                    parsing_state = PARSING_HEADER_1;
                    note_error(ERROR_FRAME_LEN);
                    break;
                }
                rx_packet.elements.args[arg_count++] = current_byte;
                if(arg_count == rx_packet.elements.length - WIRE_LENGTH_OVERHEAD) parsing_state = PARSING_CHECKSUM;
                break;
            case PARSING_CHECKSUM: 
                uint8_t check = checksum_crc8(rx_packet.data_raw, rx_packet.elements.length + 1);
                if(check == current_byte) {
                    /* 必须在阻塞型 successCallback（如动作组播放）返回前复位状态，
                     * 否则回调内 rec_handler 再次进入 parsing 时仍停留在 PARSING_CHECKSUM，
                     * 会把后续 STOP 帧首字节误当作校验和，导致停止无效。 */
                    parsing_state = PARSING_HEADER_1;
                    error_state = ERROR_NULL;
                    ++stats.valid_frames;
                    stats.consecutive_errors = 0;
                    // A callback may parse another frame; preserve this frame for its caller.
                    PacketTypeDef completed_packet = rx_packet;
                    if(successCallback) successCallback(&completed_packet);
                } else {
                    parsing_state = PARSING_HEADER_1;
                    note_error(ERROR_CHECKSUM);
                }
                break;
        }
    }
}
