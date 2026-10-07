#ifndef _TANK_CHASSIS_H_
#define _TANK_CHASSIS_H_

#include <Arduino.h>
#include "Motor_I2C.h"

class Tank_Chassis {
public:
    Motor_I2C* motor_driver;

    void begin(Motor_I2C* driver) {
        this->motor_driver = driver;
    }

    void run(int16_t speed, int16_t turn) {
        int16_t speed_l = speed + turn;
        int16_t speed_r = speed - turn;

        if (speed_l > 100) speed_l = 100;
        if (speed_l < -100) speed_l = -100;
        if (speed_r > 100) speed_r = 100;
        if (speed_r < -100) speed_r = -100;

        if(motor_driver) {
            motor_driver->set_speed((int8_t)speed_l, (int8_t)speed_r, 0, 0);
        }
    }

    void stop() {
        if(motor_driver) {
            motor_driver->stop();
        }
    }
};

#endif