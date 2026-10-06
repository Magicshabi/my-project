#ifndef HX_30HM_H
#define HX_30HM_H

#include "Arduino.h"
#include "HardwareSerial.h"
#include "CommProtocol.h"
#include "HX_30HM_Def.h"

#define SERVO_RX_PIN        17
#define SERVO_TX_PIN        16
#define BROADCAST_ID        0xFE

#define REG_GOAL_POSITION_L		42

class SerialServo_t {
public:
    HardwareSerial* uart; 

    void begin(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin = SERVO_TX_PIN, uint8_t rx_pin = SERVO_RX_PIN);
    void write_id(uint8_t old_id, uint8_t new_id);
    void write_mode(uint8_t id, uint8_t mode);
    uint8_t tx_frame_write(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t data_len);
    void write_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos);
    uint8_t sync_write_pos(uint8_t *ids, int16_t *positions, uint8_t num);
    uint8_t sync_write_pos_speed(uint8_t *ids, int16_t *positions, int16_t speed, uint8_t num);
    /** 同步写：每路独立速度与加速度（动作组用，一帧同时下发） */
    uint8_t sync_write_pos_speed_ex(uint8_t *ids, int16_t *positions, int16_t *speeds, uint8_t *accs, uint8_t num);
private:
    uint8_t data_check(const uint8_t buf[], uint8_t len);
};

extern SerialServo_t servo;

#endif