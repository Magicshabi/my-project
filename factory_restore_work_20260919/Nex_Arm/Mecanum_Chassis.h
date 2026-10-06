#ifndef _MECANUM_CHASSIS_H_
#define _MECANUM_CHASSIS_H_

#include <Arduino.h>
#include <math.h>
#include "Motor_I2C.h"

class Mecanum_Chassis {
public:
    Motor_I2C* motor_driver = nullptr;

    void begin(Motor_I2C* driver) {
        this->motor_driver = driver;
    }

    void configure(float wheel_diameter_mm,
                   float wheel_base_mm,
                   float track_width_mm,
                   const uint8_t motor_dir[4],
                   int8_t max_speed_pct) {
        // Invalid values fall back to reference defaults.
        wheel_diameter_mm_ = (wheel_diameter_mm > 1.0f) ? wheel_diameter_mm : kRefWheelDiameterMm;
        wheel_base_mm_ = (wheel_base_mm > 1.0f) ? wheel_base_mm : kRefWheelBaseMm;
        track_width_mm_ = (track_width_mm > 1.0f) ? track_width_mm : kRefTrackWidthMm;
        max_speed_pct_ = (int8_t)constrain((int)max_speed_pct, 0, 100);

        if (motor_dir) {
            for (uint8_t i = 0; i < 4; i++) {
                motor_dir_[i] = motor_dir[i] ? 1 : 0;
            }
        }
    }

    /**
     * @brief 麦轮运动学逆解（车体坐标系）
     * @param vx  前后速度 (-100 ~ 100)，正数为前
     * @param vy  左右平移 (-100 ~ 100)，正数为左
     * @param vz  旋转速度 (-100 ~ 100)，正数为逆时针
     */
    void set_velocity(int16_t vx, int16_t vy, int16_t vz) {
        // Use chassis geometry and wheel size to scale translation and rotation.
        float wheel_r = wheel_diameter_mm_ * 0.5f;
        if (wheel_r < 1.0f) wheel_r = kRefWheelDiameterMm * 0.5f;

        float ref_r = kRefWheelDiameterMm * 0.5f;
        float rot_arm = (wheel_base_mm_ + track_width_mm_) * 0.5f;
        if (rot_arm < 1.0f) rot_arm = (kRefWheelBaseMm + kRefTrackWidthMm) * 0.5f;
        float ref_rot_arm = (kRefWheelBaseMm + kRefTrackWidthMm) * 0.5f;

        float linear_scale = ref_r / wheel_r;
        float yaw_scale = (rot_arm / wheel_r) / (ref_rot_arm / ref_r);

        float fx = (float)vx * linear_scale;
        float fy = (float)vy * linear_scale;
        float fz = (float)vz * yaw_scale;

        // Actual wiring: M1=right front, M2=right rear, M3=left front, M4=left rear.
        float rf = fx + fy + fz;
        float rb = fx - fy + fz;
        float lf = fx - fy - fz;
        float lb = fx + fy - fz;

        float m1f = -rf; // M1: right front
        float m2f =  rb; // M2: right rear
        float m3f =  lf; // M3: left front
        float m4f = -lb; // M4: left rear

        float max_val = fmaxf(fabsf(m1f), fmaxf(fabsf(m2f), fmaxf(fabsf(m3f), fabsf(m4f))));
        if (max_val > 100.0f) {
            float norm = 100.0f / max_val;
            m1f *= norm;
            m2f *= norm;
            m3f *= norm;
            m4f *= norm;
        }

        float max_scale = (float)max_speed_pct_ / 100.0f;
        m1f *= max_scale;
        m2f *= max_scale;
        m3f *= max_scale;
        m4f *= max_scale;

        if (motor_dir_[0]) m1f = -m1f;
        if (motor_dir_[1]) m2f = -m2f;
        if (motor_dir_[2]) m3f = -m3f;
        if (motor_dir_[3]) m4f = -m4f;

        int16_t m1 = (int16_t)constrain((int)lroundf(m1f), -100, 100);
        int16_t m2 = (int16_t)constrain((int)lroundf(m2f), -100, 100);
        int16_t m3 = (int16_t)constrain((int)lroundf(m3f), -100, 100);
        int16_t m4 = (int16_t)constrain((int)lroundf(m4f), -100, 100);

        static uint32_t last_print_ms = 0;
        static int16_t last_vx = 32767, last_vy = 32767, last_vz = 32767;
        static int16_t last_m1 = 32767, last_m2 = 32767, last_m3 = 32767, last_m4 = 32767;
        uint32_t now = millis();
        bool changed = (vx != last_vx) || (vy != last_vy) || (vz != last_vz) ||
                       (m1 != last_m1) || (m2 != last_m2) || (m3 != last_m3) || (m4 != last_m4);
        if (changed || (now - last_print_ms >= 300)) {
            last_print_ms = now;
            last_vx = vx; last_vy = vy; last_vz = vz;
            last_m1 = m1; last_m2 = m2; last_m3 = m3; last_m4 = m4;
            Serial.printf("[MEC] vx=%d vy=%d vz=%d | kL=%.3f kW=%.3f max=%d | rf=%.1f rb=%.1f lf=%.1f lb=%.1f | M1=%d M2=%d M3=%d M4=%d\n",
                          (int)vx, (int)vy, (int)vz,
                          (double)linear_scale, (double)yaw_scale, (int)max_speed_pct_,
                          (double)rf, (double)rb, (double)lf, (double)lb,
                          (int)m1, (int)m2, (int)m3, (int)m4);
        }
        if(motor_driver) {
            motor_driver->set_speed((int8_t)m1, (int8_t)m2, (int8_t)m3, (int8_t)m4);
        }
    }

private:
    static constexpr float kRefWheelDiameterMm = 25.0f;
    static constexpr float kRefWheelBaseMm = 190.0f;
    static constexpr float kRefTrackWidthMm = 95.0f;

    float wheel_diameter_mm_ = kRefWheelDiameterMm;
    float wheel_base_mm_ = kRefWheelBaseMm;
    float track_width_mm_ = kRefTrackWidthMm;
    uint8_t motor_dir_[4] = {0, 0, 0, 0};
    int8_t max_speed_pct_ = 100;
};

#endif
