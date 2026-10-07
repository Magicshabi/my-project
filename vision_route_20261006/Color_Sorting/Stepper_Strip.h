#ifndef _STEPPER_STRIP_H_
#define _STEPPER_STRIP_H_

#include <Arduino.h>
#include <Wire.h>

#define STEPPER_ADDR                0x35
#define REG_MODE                    0x15  // 细分模式寄存器
#define REG_RESET                   0x16  // 复位/归零寄存器
#define REG_STEPS                   0x18  // 步数寄存器 (int32)
#define REG_SPEED                   0x1C  // 速度/时间寄存器 (uint32)

#define SUB_NONE                    0x00
#define SUB_2                       0x01
#define SUB_4                       0x02
#define SUB_8                       0x03
#define SUB_16                      0x07

class Stepper_Strip {
public:
    void begin(TwoWire *wire);
    void reset();
    void set_speed(uint32_t time_us);
    void move(int32_t steps);
    void set_subdivision(uint8_t div_code); 
    uint8_t read_reset_status(); // 读取是否复位完成

private:
    TwoWire *i2c_bus;
    void write_bytes(uint8_t reg, uint8_t *data, uint8_t len);
    void read_bytes(uint8_t reg, uint8_t *data, uint8_t len);
};

#endif