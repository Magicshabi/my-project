#ifndef _MOTOR_I2C_H_
#define _MOTOR_I2C_H_

#include <Arduino.h>
#include <Wire.h>

#define MOTOR_I2C_ADDR              0x34

#define ADC_BAT_ADDR                0x00
#define MOTOR_TYPE_ADDR             0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_PWM_ADDR        0x1F
#define MOTOR_FIXED_SPEED_ADDR      0x33
#define MOTOR_ENCODER_TOTAL_ADDR    0x3C

enum MotorType {
    MOTOR_WITHOUT_ENCODER = 0,
    MOTOR_TT              = 1,
    MOTOR_N20             = 2,
    MOTOR_JGB37_520       = 3 
};

class Motor_I2C {
public:
    void begin(TwoWire &wire, uint8_t sda_pin, uint8_t scl_pin, MotorType type = MOTOR_JGB37_520, uint8_t polarity = 0);

    void set_speed(int8_t m1, int8_t m2, int8_t m3, int8_t m4);
    
    void set_single_speed(uint8_t index, int8_t speed);

    void stop();
    
    void read_encoder(int32_t *enc);
    void read_vol(uint16_t *vol);

private:
    TwoWire *i2c_bus;
    void write_bytes(uint8_t reg, uint8_t *buf, uint8_t len);
    void read_bytes(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif