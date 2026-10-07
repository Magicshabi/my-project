#include "Motor_I2C.h"
static int8_t motor_shadow_speeds[4] = {0, 0, 0, 0};
void Motor_I2C::begin(TwoWire &wire, uint8_t sda_pin, uint8_t scl_pin, MotorType type, uint8_t polarity)
{
    i2c_bus = &wire;
    i2c_bus->begin(sda_pin, scl_pin, 100000); 
    delay(50); 

    uint8_t type_val = (uint8_t)type;
    write_bytes(MOTOR_TYPE_ADDR, &type_val, 1);
    delay(5);
    write_bytes(MOTOR_ENCODER_POLARITY_ADDR, &polarity, 1);
    delay(5);
}

void Motor_I2C::set_speed(int8_t m1, int8_t m2, int8_t m3, int8_t m4)
{
    motor_shadow_speeds[0] = m1;
    motor_shadow_speeds[1] = m2;
    motor_shadow_speeds[2] = m3;
    motor_shadow_speeds[3] = m4;
    write_bytes(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)motor_shadow_speeds, 4);
}
void Motor_I2C::set_single_speed(uint8_t index, int8_t speed)
{
    if (index < 1 || index > 4) return;
    
    motor_shadow_speeds[index - 1] = speed;
    
    write_bytes(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)motor_shadow_speeds, 4);
}

void Motor_I2C::stop()
{
    memset(motor_shadow_speeds, 0, 4);
    write_bytes(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)motor_shadow_speeds, 4);
}
void Motor_I2C::read_encoder(int32_t *enc)
{
    read_bytes(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)enc, 16);
}

void Motor_I2C::read_vol(uint16_t *vol)
{
    uint8_t buf[2];
    read_bytes(ADC_BAT_ADDR, buf, 2);
    *vol = buf[0] + (buf[1] << 8);
}

void Motor_I2C::write_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_bus->beginTransmission(MOTOR_I2C_ADDR);
    i2c_bus->write(reg);
    for(uint8_t i = 0; i < len; i++) {
        i2c_bus->write(buf[i]);
    }
    i2c_bus->endTransmission();
}

void Motor_I2C::read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_bus->beginTransmission(MOTOR_I2C_ADDR);
    i2c_bus->write(reg);
    if(i2c_bus->endTransmission() != 0) return;

    i2c_bus->requestFrom((uint8_t)MOTOR_I2C_ADDR, len);
    for(uint8_t i = 0; i < len && i2c_bus->available(); i++) {
        buf[i] = i2c_bus->read();
    }
}