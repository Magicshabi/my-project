#ifndef _MECANUM_CHASSIS_H_
#define _MECANUM_CHASSIS_H_

#include <Arduino.h>
#include <math.h>
#include "Motor_I2C.h"

class Mecanum_Chassis {
public:
    Motor_I2C* motor_driver;

    void begin(Motor_I2C* driver) {
        this->motor_driver = driver;
    }

    /**
     * @brief 麦轮运动学解算 (笛卡尔坐标系)
     * @param vx  前后速度 (-100 ~ 100)，正数为前
     * @param vy  左右平移 (-100 ~ 100)，正数为左
     * @param vz  旋转速度 (-100 ~ 100)，正数为逆时针旋转
     */
    void set_velocity(int16_t vx, int16_t vy, int16_t vz) {
        // 麦轮逆运动学公式 (A=Vx, B=Vy, C=Vz)

        // Motor1 (LF) = Vx - Vy - Vz
        // Motor2 (RF) = Vx + Vy + Vz 
        // Motor3 (LB) = Vx + Vy - Vz
        // Motor4 (RB) = Vx - Vy + Vz
        
        int16_t m1 = vx - vy - vz; // 左前
        int16_t m2 = vx + vy + vz; // 右前 
        int16_t m3 = vx + vy - vz; // 左后
        int16_t m4 = vx - vy + vz; // 右后

        int16_t max_val = max(abs(m1), max(abs(m2), max(abs(m3), abs(m4))));
        
        if (max_val > 100) {
            m1 = m1 * 100 / max_val;
            m2 = m2 * 100 / max_val;
            m3 = m3 * 100 / max_val;
            m4 = m4 * 100 / max_val;
        }
        //  output 是 [-m1, -m2, m3, m4]
        if(motor_driver) {

            motor_driver->set_speed((int8_t)m1, (int8_t)m2, (int8_t)m3, (int8_t)m4);
        }
    }
};

#endif