#include "Conveyor_Belt.h"

void Conveyor_Belt::begin(TwoWire *wire)
{
    this->i2c_bus = wire;
}

void Conveyor_Belt::set_speed(int8_t speed)
{
    write_reg(0x00, (uint8_t)speed);
}

void Conveyor_Belt::write_reg(uint8_t reg, uint8_t data)
{
    if(i2c_bus == nullptr) return;
    i2c_bus->beginTransmission(CONVEYOR_ADDR);
    i2c_bus->write(reg);
    i2c_bus->write(data);
    i2c_bus->endTransmission();
}