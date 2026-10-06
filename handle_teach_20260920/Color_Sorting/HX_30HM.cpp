#include "HX_30HM.h"
#include "system_task_handle.h"
#include <cstdint>

#define MASK_HOST(x, bit) 	((x) < 0 ? (-x) | (1U) << (bit): (x))
#define LIMIT(x, min, max) (((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x))

SerialServo_t servo;

void SerialServo_t::begin(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin) {
	this->uart = &uart;
	this->uart->setTxBufferSize(2048);
	this->uart->setRxBufferSize(2048);
	this->uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
}

uint8_t SerialServo_t::data_check(const uint8_t buf[], uint8_t len)
{
    uint16_t temp = 0;
    for (int i = 2; i < len; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

uint8_t SerialServo_t::tx_frame_write(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
	uint8_t frame_len =  6 + data_len;
  	uint8_t packet[frame_len];
  
	packet[0] = FRAME_HEADER_1;
	packet[1] = FRAME_HEADER_2;
	packet[2] = id;
	packet[3] = 2 + data_len;
	packet[4] = cmd;
	
	for(uint8_t i = 0; i < data_len; i++) {
		packet[5 + i] = data[i];
	}
	
	packet[frame_len - 1] = data_check((const uint8_t*)packet, frame_len - 1);
	return uart->write(packet, frame_len);
}

void SerialServo_t::write_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos)
{
    uint8_t data[8]; 
    uint16_t _pos, _spd;

    acc = LIMIT(acc, 0, 254);
    speed = LIMIT(speed, -3400, 3400);
    pos = LIMIT(pos, -30719, 30719);

    _pos = (uint16_t)MASK_HOST(pos, 15);
    _spd = (uint16_t)MASK_HOST(speed, 15);

    data[0] = 41;                      // 起始地址 REG_ACC
    data[1] = acc;                     // 41: 加速度
    data[2] = (uint8_t)(_pos & 0xFF);  // 42: 位置低位
    data[3] = (uint8_t)(_pos >> 8);    // 43: 位置高位
    data[4] = 0;                       // 44: PWM低
    data[5] = 0;                       // 45: PWM高
    data[6] = (uint8_t)(_spd & 0xFF);  // 46: 速度低位
    data[7] = (uint8_t)(_spd >> 8);    // 47: 速度高位

    tx_frame_write(id, 3, data, 8); 
}
uint8_t SerialServo_t::sync_write_pos_speed(uint8_t *ids, int16_t *positions, int16_t speed, uint8_t num) {
    uint8_t args_len = 2 + (num * 8); // Addr + DataLen + (ID + 7 Bytes)*num
    uint8_t buf[args_len];

    buf[0] = 41; 
    buf[1] = 7;  

    for (int i = 0; i < num; i++) {
        uint16_t u16_pos = (uint16_t)MASK_HOST(positions[i], 15);
        uint16_t u16_spd = (uint16_t)MASK_HOST(speed, 15);

        uint8_t base = 2 + i * 8;
        buf[base]     = ids[i];
        buf[base + 1] = global_acc;    // ACC: 使用全局加速度（上位机可设置）
        buf[base + 2] = u16_pos & 0xFF;  // POS_L
        buf[base + 3] = u16_pos >> 8;    // POS_H
        buf[base + 4] = 0;               // PWM_L
        buf[base + 5] = 0;               // PWM_H
        buf[base + 6] = u16_spd & 0xFF;  // SPD_L
        buf[base + 7] = u16_spd >> 8;    // SPD_H
    }
    return tx_frame_write(0xFE, 131, buf, args_len);
}

uint8_t SerialServo_t::sync_write_pos_speed_ex(uint8_t *ids, int16_t *positions, int16_t *speeds, uint8_t *accs, uint8_t num) {
    uint8_t args_len = 2 + (num * 8);
    uint8_t buf[args_len];

    buf[0] = 41;
    buf[1] = 7;

    for (int i = 0; i < num; i++) {
        uint16_t u16_pos = (uint16_t)MASK_HOST(positions[i], 15);
        uint16_t u16_spd = (uint16_t)MASK_HOST(speeds[i], 15);
        uint8_t acc = LIMIT(accs[i], 0, 254);

        uint8_t base = 2 + i * 8;
        buf[base]     = ids[i];
        buf[base + 1] = acc;
        buf[base + 2] = u16_pos & 0xFF;
        buf[base + 3] = u16_pos >> 8;
        buf[base + 4] = 0;
        buf[base + 5] = 0;
        buf[base + 6] = u16_spd & 0xFF;
        buf[base + 7] = u16_spd >> 8;
    }
    return tx_frame_write(0xFE, 131, buf, args_len);
}

void SerialServo_t::write_id(uint8_t old_id, uint8_t new_id) {
    uint8_t data[2] = {REG_ID, new_id};
    tx_frame_write(old_id, CMD_WRITE, data, 2);
}

void SerialServo_t::write_mode(uint8_t id, uint8_t mode) {
    uint8_t data[2] = {REG_MODE, mode};
    tx_frame_write(id, CMD_WRITE, data, 2);
}
// uint8_t SerialServo_t::sync_write_pos_speed(uint8_t *ids, int16_t *positions, int16_t speed, uint8_t num) {
//     uint8_t args_len = 2 + (num * 8); 
//     uint8_t buf[args_len];

//     buf[0] = 41; 
//     buf[1] = 7;  

//     for (int i = 0; i < num; i++) {
//         uint16_t u16_pos = (uint16_t)MASK_HOST(positions[i], 15);
//         uint16_t u16_spd = (uint16_t)MASK_HOST(speed, 15);
        
//         uint8_t base = 2 + i * 8;
//         buf[base]     = ids[i];
        
//         buf[base + 1] = 50;             
        
//         buf[base + 2] = u16_pos & 0xFF;  // 位置低位
//         buf[base + 3] = u16_pos >> 8;    // 位置高位
//         buf[base + 4] = 0;               // PWM 低位 
//         buf[base + 5] = 0;               // PWM 高位 
//         buf[base + 6] = u16_spd & 0xFF;  // 速度低位
//         buf[base + 7] = u16_spd >> 8;    // 速度高位
//     }
//     return tx_frame_write(0xFE, 131, buf, args_len);
// }
