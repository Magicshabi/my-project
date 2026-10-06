// #ifndef __ROBOT_ARM_H__
// #define __ROBOT_ARM_H__

// #include "Nex_Arm_Board.h"
// #include "HX_30HM.h" 
// #include "Global.h"

// // 机械臂状态结构体
// typedef struct {
//     float x;
//     float y;
//     float z;
//     float pitch;
//     float roll;
//     float claw; 
//     uint32_t duration;
//     float joint_angle[4];
// } ArmPose_t;

// class Robot_Arm_t 
// { 
// public:
//     HW_Board board; 
//     ArmPose_t current_pose;

//     void begin(void);
    
//     void move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration_ms);
//     // void move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration_ms, bool is_track_mode = false);
//     void move_inc(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint32_t duration_ms);
//     void move_servo(uint8_t id, int16_t angle, uint16_t time_ms);
    
//     void set_claw(float angle);
//     void set_roll(float angle);
//     void request_ik_calc(float x, float y, float z, float pitch);

//     void update_status(void);
    
//     // 测试程序
//     void test(void);
//     void set_servo_id(uint8_t old_id, uint8_t new_id);
//     void set_servo_mode(uint8_t id, uint8_t mode);
//     void reset_all(uint32_t duration_ms);

// private:
//     void send_packet(uint8_t cmd, uint8_t* data, uint8_t len);
// };

// extern Robot_Arm_t arm;

// #endif
#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__

#include "Nex_Arm_Board.h"
#include "HX_30HM.h" 
#include "Global.h"

// 机械臂状态结构体
typedef struct {
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float claw; 
    uint32_t duration;
    float joint_angle[4];
} ArmPose_t;

class Robot_Arm_t 
{ 
public:
    HW_Board board; 
    ArmPose_t current_pose;

    float limit_x_min = 100.0f;
    float limit_x_max = 400.0f;
    float limit_y_min = -450.0f;
    float limit_y_max = 450.0f;
    float limit_z_min = 20.0f;
    float limit_z_max = 500.0f;

    void begin(void);
    
    void set_torque(bool enable); 
    
    void move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration_ms, bool calc_only = false);
    void request_fk_calc(float j1, float j2, float j3, float j4, float roll, float claw);
    
    // void move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration_ms);
    
    void move_inc(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint32_t duration_ms);
    
    void move_servo(uint8_t id, int16_t angle, uint16_t time_ms);
    
    void set_claw(float angle);
    void set_roll(float angle);
    void request_ik_calc(float x, float y, float z, float pitch);

    void update_status(void);
    
    void test(void);
    void set_servo_id(uint8_t old_id, uint8_t new_id);
    void set_servo_mode(uint8_t id, uint8_t mode);
    void reset_all(uint32_t duration_ms);

    /* 舵机参数读写 */
    void read_servo_overload(uint8_t servo_id);
    void write_servo_overload(uint8_t servo_id, uint8_t torque, uint8_t time_val);
    void read_servo_baud(uint8_t servo_id);
    void write_servo_baud(uint8_t servo_id, uint8_t baud);
    void read_servo_max_torque(uint8_t servo_id);
    void write_servo_max_torque(uint8_t servo_id, uint16_t torque);
    void read_servo_angle_limit(uint8_t servo_id);
    void write_servo_angle_limit(uint8_t servo_id, int16_t min_angle, int16_t max_angle);

    /* 坐标限位下发/读取 */
    void send_coord_limits(void);
    void request_coord_limits(void);


private:
    void send_packet(uint8_t cmd, uint8_t* data, uint8_t len);
};

extern Robot_Arm_t arm;

#endif