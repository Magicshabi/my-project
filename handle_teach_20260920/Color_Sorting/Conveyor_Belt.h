#ifndef _CONVEYOR_BELT_H_
#define _CONVEYOR_BELT_H_

#include <Arduino.h>
#include <Wire.h>

#define CONVEYOR_ADDR 0x37

class Conveyor_Belt {
public:
    void begin(TwoWire *wire);
    void set_speed(int8_t speed);

private:
    TwoWire *i2c_bus;
    void write_reg(uint8_t reg, uint8_t data);
};

#endif