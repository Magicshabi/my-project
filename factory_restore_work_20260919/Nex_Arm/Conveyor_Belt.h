#ifndef _CONVEYOR_BELT_H_
#define _CONVEYOR_BELT_H_

#include <Arduino.h>
#include <Wire.h>

#define CONVEYOR_NEW_ADDR 0x50
#define CONVEYOR_OLD_ADDR 0x37

#define CONVEYOR_REG_COMMAND          0x10
#define CONVEYOR_REG_TARGET_SPEED_RPM 0x20

#define CONVEYOR_CMD_SET_SPEED        0x01

enum ConveyorProtocol {
    CONVEYOR_PROTOCOL_NONE = 0,
    CONVEYOR_PROTOCOL_NEW,
    CONVEYOR_PROTOCOL_OLD
};

class Conveyor_Belt {
public:
    void begin(TwoWire *wire);
    void set_speed(int8_t speed);

private:
    TwoWire *i2c_bus = nullptr;
    ConveyorProtocol protocol = CONVEYOR_PROTOCOL_NONE;

    bool probe_addr(uint8_t addr);
    void write_reg(uint8_t addr, uint8_t reg, uint8_t data);
    void write_bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len);
};

#endif
