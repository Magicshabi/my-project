#include "Stepper_Strip.h"

void Stepper_Strip::begin(TwoWire *wire) {
    this->i2c_bus = wire;
    if(read_reset_status() == 0) {
        reset();
    }
}

void Stepper_Strip::reset() {
    uint8_t val = 1;
    write_bytes(REG_RESET, &val, 1);
}

void Stepper_Strip::set_speed(uint32_t time_us) {
    write_bytes(REG_SPEED, (uint8_t*)&time_us, 4);
}

void Stepper_Strip::move(int32_t steps) {
    write_bytes(REG_STEPS, (uint8_t*)&steps, 4);
}

void Stepper_Strip::set_subdivision(uint8_t div_code) {
    write_bytes(REG_MODE, &div_code, 1);
}

uint8_t Stepper_Strip::read_reset_status() {
    uint8_t val = 0;
    read_bytes(REG_RESET, &val, 1);
    return val;
}

void Stepper_Strip::write_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
    if(!i2c_bus) return;
    i2c_bus->beginTransmission(STEPPER_ADDR);
    i2c_bus->write(reg);
    for(uint8_t i=0; i<len; i++) i2c_bus->write(data[i]);
    i2c_bus->endTransmission();
}

void Stepper_Strip::read_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
    if(!i2c_bus) return;
    i2c_bus->beginTransmission(STEPPER_ADDR);
    i2c_bus->write(reg);
    if(i2c_bus->endTransmission() != 0) return;
    
    i2c_bus->requestFrom((uint8_t)STEPPER_ADDR, len);
    for(uint8_t i=0; i<len && i2c_bus->available(); i++) {
        data[i] = i2c_bus->read();
    }
}