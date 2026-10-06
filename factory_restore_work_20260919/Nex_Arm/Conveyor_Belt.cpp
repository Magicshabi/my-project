#include "Conveyor_Belt.h"

void Conveyor_Belt::begin(TwoWire *wire)
{
    this->i2c_bus = wire;
    protocol = CONVEYOR_PROTOCOL_NONE;

    if (probe_addr(CONVEYOR_NEW_ADDR)) {
        protocol = CONVEYOR_PROTOCOL_NEW;
        write_reg(CONVEYOR_NEW_ADDR, CONVEYOR_REG_COMMAND, CONVEYOR_CMD_SET_SPEED);
    } else if (probe_addr(CONVEYOR_OLD_ADDR)) {
        protocol = CONVEYOR_PROTOCOL_OLD;
    }
}

void Conveyor_Belt::set_speed(int8_t speed)
{
    if(i2c_bus == nullptr || protocol == CONVEYOR_PROTOCOL_NONE) return;

    if (protocol == CONVEYOR_PROTOCOL_NEW) {
        float rpm = (float)speed;
        if(rpm > 162.0f) rpm = 162.0f;
        if(rpm < -162.0f) rpm = -162.0f;

        write_reg(CONVEYOR_NEW_ADDR, CONVEYOR_REG_COMMAND, CONVEYOR_CMD_SET_SPEED);
        write_bytes(CONVEYOR_NEW_ADDR, CONVEYOR_REG_TARGET_SPEED_RPM, (const uint8_t*)&rpm, sizeof(rpm));
    } else if (protocol == CONVEYOR_PROTOCOL_OLD) {
        write_reg(CONVEYOR_OLD_ADDR, 0x00, (uint8_t)speed);
    }
}

bool Conveyor_Belt::probe_addr(uint8_t addr)
{
    if(i2c_bus == nullptr) return false;
    i2c_bus->beginTransmission(addr);
    return i2c_bus->endTransmission() == 0;
}

void Conveyor_Belt::write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
    if(i2c_bus == nullptr) return;
    i2c_bus->beginTransmission(addr);
    i2c_bus->write(reg);
    i2c_bus->write(data);
    i2c_bus->endTransmission();
}

void Conveyor_Belt::write_bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    if(i2c_bus == nullptr) return;
    i2c_bus->beginTransmission(addr);
    i2c_bus->write(reg);
    for(uint8_t i = 0; i < len; i++) {
        i2c_bus->write(data[i]);
    }
    i2c_bus->endTransmission();
}
