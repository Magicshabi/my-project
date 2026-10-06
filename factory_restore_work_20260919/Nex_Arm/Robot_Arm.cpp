// // #include "Robot_Arm.h"

// // Robot_Arm_t arm;

// // void Robot_Arm_t::begin(void)
// // {
// //     board.begin();
    
// //     this->current_pose.x = 0.0f;
// //     this->current_pose.y = 200.0f;
// //     this->current_pose.z = 200.0f;
// //     this->current_pose.pitch = 0.0f;
// //     this->current_pose.roll = 0.0f;
// //     this->current_pose.claw = 0.0f;
// // }


// // void Robot_Arm_t::send_packet(uint8_t cmd, uint8_t* data, uint8_t len)
// // {
// //     servo.tx_frame_write(0xFF, cmd, data, len);
// // }

// // void Robot_Arm_t::move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration)
// // {
// //     uint8_t buf[12];
    
// //     int16_t i_pitch = (int16_t)(pitch * 10.0f); 
// //     int16_t i_x = (int16_t)x;
// //     int16_t i_y = (int16_t)y;
// //     int16_t i_z = (int16_t)z;
// //     int16_t i_roll = (int16_t)roll; 
    
// //     uint16_t i_time = (uint16_t)duration;
    
// //     buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
// //     buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
// //     buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
// //     buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);
// //     buf[8] = GET_LOW_BYTE(i_roll);  buf[9] = GET_HIGH_BYTE(i_roll);
// //     buf[10] = GET_LOW_BYTE(i_time); buf[11] = GET_HIGH_BYTE(i_time);
    
// //     send_packet(CMD_COORDINATE_SET, buf, 12);

// //     this->current_pose.x = x;
// //     this->current_pose.y = y;
// //     this->current_pose.z = z;
// //     this->current_pose.pitch = pitch;
// //     this->current_pose.roll = roll;
// // }


// // void Robot_Arm_t::update_status(void)
// // {
// //     send_packet(CMD_GET_CUR_COORDS, NULL, 0);
// // }
// // void Robot_Arm_t::move_inc(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint32_t duration_ms)
// // {
// //     uint8_t buf[12];
    
// //     int16_t i_dx = (int16_t)dx;
// //     int16_t i_dy = (int16_t)dy;
// //     int16_t i_dz = (int16_t)dz;
// //     int16_t i_dp = (int16_t)(dpitch * 10.0f); 
// //     int16_t i_dr = (int16_t)droll;
    
// //     // 【修改4】：时间改为无符号 16 位
// //     uint16_t i_time = (uint16_t)duration_ms;
    
// //     buf[0] = GET_LOW_BYTE(i_dx); buf[1] = GET_HIGH_BYTE(i_dx);
// //     buf[2] = GET_LOW_BYTE(i_dy); buf[3] = GET_HIGH_BYTE(i_dy);
// //     buf[4] = GET_LOW_BYTE(i_dz); buf[5] = GET_HIGH_BYTE(i_dz);
// //     buf[6] = GET_LOW_BYTE(i_dp); buf[7] = GET_HIGH_BYTE(i_dp);
// //     buf[8] = GET_LOW_BYTE(i_dr); buf[9] = GET_HIGH_BYTE(i_dr);
// //     buf[10] = GET_LOW_BYTE(i_time); buf[11] = GET_HIGH_BYTE(i_time);
    
// //     send_packet(CMD_ARM_MOVE_INC, buf, 12);

// //     this->current_pose.x += dx;
// //     this->current_pose.y += dy;
// //     this->current_pose.z += dz;
// //     this->current_pose.pitch += dpitch;
// //     this->current_pose.roll += droll;
// // }
// // void Robot_Arm_t::move_servo(uint8_t id, int16_t angle, uint16_t time_ms)
// // {
    
// //     int16_t pos = angle; 
    
// //     int16_t speed = 1000; 
// //     if(time_ms > 0) speed = 20000 / time_ms; 

    
// //     servo.write_pos_ex(id, 0, speed, pos);
// //     update_status();
// // }
// // void Robot_Arm_t::request_ik_calc(float x, float y, float z, float pitch)
// // {
// //     uint8_t buf[8];
// //     int16_t i_pitch = (int16_t)pitch;
// //     int16_t i_x = (int16_t)x;
// //     int16_t i_y = (int16_t)y;
// //     int16_t i_z = (int16_t)z;
    
// //     buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
// //     buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
// //     buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
// //     buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);

// //     send_packet(CMD_IKINE_RESULT_GET, buf, 8);
// // }

// // void Robot_Arm_t::test(void)
// // {
// //     // move(0.0f, 200.0f, 250.0f, 0.0f, 0.0f, 0.0f, 2000);
// // }
// #include "Robot_Arm.h"

// Robot_Arm_t arm;

// void Robot_Arm_t::begin(void)
// {
//     board.begin();
    
//     this->current_pose.x = 0.0f;
//     this->current_pose.y = 200.0f;
//     this->current_pose.z = 200.0f;
//     this->current_pose.pitch = 0.0f;
//     this->current_pose.roll = 0.0f;
//     this->current_pose.claw = 0.0f;
// }

// void Robot_Arm_t::send_packet(uint8_t cmd, uint8_t* data, uint8_t len)
// {
//     servo.tx_frame_write(0xFF, cmd, data, len);
// }

// void Robot_Arm_t::move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration)
// {
//     uint8_t buf[14]; // 从12扩大到14
    
//     int16_t i_pitch = (int16_t)(pitch * 10.0f); 
//     int16_t i_x = (int16_t)x;
//     int16_t i_y = (int16_t)y;
//     int16_t i_z = (int16_t)z;
//     int16_t i_roll = (int16_t)roll; 
//     int16_t i_claw = (int16_t)claw; // 【新增】
//     uint16_t i_time = (uint16_t)duration;
    
//     buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
//     buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
//     buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
//     buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);
//     buf[8] = GET_LOW_BYTE(i_roll);  buf[9] = GET_HIGH_BYTE(i_roll);
//     buf[10] = GET_LOW_BYTE(i_claw); buf[11] = GET_HIGH_BYTE(i_claw); // 【新增】
//     buf[12] = GET_LOW_BYTE(i_time); buf[13] = GET_HIGH_BYTE(i_time);
    
//     send_packet(CMD_COORDINATE_SET, buf, 14); // 长度改为14

//     this->current_pose.x = x;
//     this->current_pose.y = y;
//     this->current_pose.z = z;
//     this->current_pose.pitch = pitch;
//     this->current_pose.roll = roll;
//     this->current_pose.claw = claw; // 记录状态
// }

// void Robot_Arm_t::update_status(void)
// {
//     send_packet(CMD_GET_CUR_COORDS, NULL, 0);
// }

// void Robot_Arm_t::move_inc(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint32_t duration_ms)
// {
//     uint8_t buf[12];
    
//     int16_t i_dx = (int16_t)dx;
//     int16_t i_dy = (int16_t)dy;
//     int16_t i_dz = (int16_t)dz;
//     int16_t i_dp = (int16_t)(dpitch * 10.0f); 
//     int16_t i_dr = (int16_t)droll;
//     uint16_t i_time = (uint16_t)duration_ms;
    
//     buf[0] = GET_LOW_BYTE(i_dx); buf[1] = GET_HIGH_BYTE(i_dx);
//     buf[2] = GET_LOW_BYTE(i_dy); buf[3] = GET_HIGH_BYTE(i_dy);
//     buf[4] = GET_LOW_BYTE(i_dz); buf[5] = GET_HIGH_BYTE(i_dz);
//     buf[6] = GET_LOW_BYTE(i_dp); buf[7] = GET_HIGH_BYTE(i_dp);
//     buf[8] = GET_LOW_BYTE(i_dr); buf[9] = GET_HIGH_BYTE(i_dr);
//     buf[10] = GET_LOW_BYTE(i_time); buf[11] = GET_HIGH_BYTE(i_time);
    
//     send_packet(CMD_ARM_MOVE_INC, buf, 12);

//     this->current_pose.x += dx;
//     this->current_pose.y += dy;
//     this->current_pose.z += dz;
//     this->current_pose.pitch += dpitch;
//     this->current_pose.roll += droll;
// }

// void Robot_Arm_t::move_servo(uint8_t id, int16_t angle, uint16_t time_ms)
// {
//     int16_t pos = angle; 
//     int16_t speed = 1000; 
//     if(time_ms > 0) speed = 20000 / time_ms; 
//     servo.write_pos_ex(id, 0, speed, pos);
//     update_status();
// }

// void Robot_Arm_t::request_ik_calc(float x, float y, float z, float pitch)
// {
//     uint8_t buf[8];
//     int16_t i_pitch = (int16_t)pitch;
//     int16_t i_x = (int16_t)x;
//     int16_t i_y = (int16_t)y;
//     int16_t i_z = (int16_t)z;
    
//     buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
//     buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
//     buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
//     buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);

//     send_packet(CMD_IKINE_RESULT_GET, buf, 8);
// }

// void Robot_Arm_t::test(void)
// {
// }

// void Robot_Arm_t::set_servo_id(uint8_t old_id, uint8_t new_id) {
//     servo.write_id(old_id, new_id);
// }

// void Robot_Arm_t::set_servo_mode(uint8_t id, uint8_t mode) {
//     servo.write_mode(id, mode);
// }

// void Robot_Arm_t::reset_all(uint32_t duration_ms) {
//     move(0.0f, 200.0f, 200.0f, 0.0f, 0.0f, 0.0f, duration_ms);
// }
#include "Robot_Arm.h"
#include <math.h>
Robot_Arm_t arm;

void Robot_Arm_t::begin(void)
{
    board.begin();
    memset(&current_pose, 0, sizeof(ArmPose_t));
    // this->current_pose.x = 0.0f;
    // this->current_pose.y = 200.0f;
    // this->current_pose.z = 200.0f;
    // this->current_pose.pitch = 0.0f;
    // this->current_pose.roll = 0.0f;
    // this->current_pose.claw = 0.0f; // 初始化夹爪
    
}

void Robot_Arm_t::send_packet(uint8_t cmd, uint8_t* data, uint8_t len)
{
    servo.tx_frame_write(0xFF, cmd, data, len);
}

// // 【修改】绝对运动：打包 14 字节
// void Robot_Arm_t::move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration)
// {
//     uint8_t buf[14]; 
    
//     int16_t i_pitch = (int16_t)(pitch * 10.0f); 
//     int16_t i_x = (int16_t)x;
//     int16_t i_y = (int16_t)y;
//     int16_t i_z = (int16_t)z;
//     int16_t i_roll = (int16_t)roll; 
//     int16_t i_claw = (int16_t)claw; // 夹爪
//     uint16_t i_time = (uint16_t)duration;
    
//     // 协议顺序：Pitch, X, Y, Z, Roll, Claw, Time
//     buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
//     buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
//     buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
//     buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);
//     buf[8] = GET_LOW_BYTE(i_roll);  buf[9] = GET_HIGH_BYTE(i_roll);
//     buf[10] = GET_LOW_BYTE(i_claw); buf[11] = GET_HIGH_BYTE(i_claw); 
//     buf[12] = GET_LOW_BYTE(i_time); buf[13] = GET_HIGH_BYTE(i_time);
    
//     send_packet(CMD_COORDINATE_SET, buf, 14);

//     // 更新本地状态
//     this->current_pose.x = x;
//     this->current_pose.y = y;
//     this->current_pose.z = z;
//     this->current_pose.pitch = pitch;
//     this->current_pose.roll = roll;
//     this->current_pose.claw = claw;
// }

void Robot_Arm_t::move(float x, float y, float z, float pitch, float roll, float claw, uint32_t duration, bool calc_only)
{
    uint8_t buf[14]; 
    
    int16_t i_pitch = (int16_t)(pitch * 10.0f); 
    int16_t i_x = (int16_t)x;
    int16_t i_y = (int16_t)y;
    int16_t i_z = (int16_t)z;
    int16_t i_roll = (int16_t)roll; 
    int16_t i_claw = (int16_t)claw; 
    uint16_t i_time = (uint16_t)duration;
    
    buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
    buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
    buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
    buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);
    buf[8] = GET_LOW_BYTE(i_roll);  buf[9] = GET_HIGH_BYTE(i_roll);
    buf[10] = GET_LOW_BYTE(i_claw); buf[11] = GET_HIGH_BYTE(i_claw); 
    buf[12] = GET_LOW_BYTE(i_time); buf[13] = GET_HIGH_BYTE(i_time);
    
    if (calc_only) {
        send_packet(CMD_IKINE_RESULT_GET, buf, 14);
    } else {
        send_packet(CMD_COORDINATE_SET, buf, 14);

        this->current_pose.x = x;
        this->current_pose.y = y;
        this->current_pose.z = z;
        this->current_pose.pitch = pitch;
        this->current_pose.roll = roll;
        this->current_pose.claw = claw;
    }
}

void Robot_Arm_t::request_fk_calc(float j1, float j2, float j3, float j4, float roll, float claw)
{
    uint8_t buf[12];
    int16_t p1 = (int16_t)(j1 * 10.0f);
    int16_t p2 = (int16_t)(j2 * 10.0f);
    int16_t p3 = (int16_t)(j3 * 10.0f);
    int16_t p4 = (int16_t)(j4 * 10.0f);
    int16_t r  = (int16_t)roll;
    int16_t c  = (int16_t)claw;

    buf[0] = GET_LOW_BYTE(p1); buf[1] = GET_HIGH_BYTE(p1);
    buf[2] = GET_LOW_BYTE(p2); buf[3] = GET_HIGH_BYTE(p2);
    buf[4] = GET_LOW_BYTE(p3); buf[5] = GET_HIGH_BYTE(p3);
    buf[6] = GET_LOW_BYTE(p4); buf[7] = GET_HIGH_BYTE(p4);
    buf[8] = GET_LOW_BYTE(r);  buf[9] = GET_HIGH_BYTE(r);
    buf[10] = GET_LOW_BYTE(c); buf[11] = GET_HIGH_BYTE(c);

    send_packet(CMD_FKINE_RESULT_GET, buf, 12);
}
// 【修改】增量运动：打包 14 字节
void Robot_Arm_t::move_inc(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint32_t duration_ms)
{
    uint8_t buf[14];
    
    int16_t i_dx = (int16_t)roundf(dx);
    int16_t i_dy = (int16_t)roundf(dy);
    int16_t i_dz = (int16_t)roundf(dz);
    int16_t i_dp = (int16_t)roundf(dpitch * 10.0f); 
    int16_t i_dr = (int16_t)roundf(droll);
    int16_t i_dc = (int16_t)roundf(dclaw); // 夹爪增量
    uint16_t i_time = (uint16_t)duration_ms;
    
    // 协议顺序：dX, dY, dZ, dPitch, dRoll, dClaw, Time
    buf[0] = GET_LOW_BYTE(i_dx); buf[1] = GET_HIGH_BYTE(i_dx);
    buf[2] = GET_LOW_BYTE(i_dy); buf[3] = GET_HIGH_BYTE(i_dy);
    buf[4] = GET_LOW_BYTE(i_dz); buf[5] = GET_HIGH_BYTE(i_dz);
    buf[6] = GET_LOW_BYTE(i_dp); buf[7] = GET_HIGH_BYTE(i_dp);
    buf[8] = GET_LOW_BYTE(i_dr); buf[9] = GET_HIGH_BYTE(i_dr);
    buf[10] = GET_LOW_BYTE(i_dc); buf[11] = GET_HIGH_BYTE(i_dc); 
    buf[12] = GET_LOW_BYTE(i_time); buf[13] = GET_HIGH_BYTE(i_time);
    
    send_packet(CMD_ARM_MOVE_INC, buf, 14);

    // 更新本地状态
    this->current_pose.x += dx;
    this->current_pose.y += dy;
    this->current_pose.z += dz;
    this->current_pose.pitch += dpitch;
    this->current_pose.roll += droll;
    this->current_pose.claw += dclaw;
}

void Robot_Arm_t::update_status(void) {
    send_packet(CMD_GET_CUR_COORDS, NULL, 0);
}

void Robot_Arm_t::move_servo(uint8_t id, int16_t angle, uint16_t time_ms) {
    int16_t pos = angle; 
    int16_t speed = 1000; 
    if(time_ms > 0) speed = 20000 / time_ms; 
    servo.write_pos_ex(id, 0, speed, pos);
    update_status();
}

void Robot_Arm_t::request_ik_calc(float x, float y, float z, float pitch) {
    uint8_t buf[8];
    int16_t i_pitch = (int16_t)pitch;
    int16_t i_x = (int16_t)x;
    int16_t i_y = (int16_t)y;
    int16_t i_z = (int16_t)z;
    buf[0] = GET_LOW_BYTE(i_pitch); buf[1] = GET_HIGH_BYTE(i_pitch);
    buf[2] = GET_LOW_BYTE(i_x);     buf[3] = GET_HIGH_BYTE(i_x);
    buf[4] = GET_LOW_BYTE(i_y);     buf[5] = GET_HIGH_BYTE(i_y);
    buf[6] = GET_LOW_BYTE(i_z);     buf[7] = GET_HIGH_BYTE(i_z);
    send_packet(CMD_IKINE_RESULT_GET, buf, 8);
}

void Robot_Arm_t::set_servo_id(uint8_t old_id, uint8_t new_id) {
    servo.write_id(old_id, new_id);
}

void Robot_Arm_t::set_servo_mode(uint8_t id, uint8_t mode) {
    servo.write_mode(id, mode);
}

void Robot_Arm_t::reset_all(uint32_t duration_ms) {
    move(200.0f, 0.0f, 200.0f, 0.0f, 0.0f, 0.0f, duration_ms);
}
void Robot_Arm_t::set_torque(bool enable)
{
    uint8_t data[2] = {REG_TORQUE_ENABLE, (uint8_t)(enable ? 1 : 0)};
    servo.tx_frame_write(BROADCAST_ID, CMD_WRITE, data, 2);
}

/* ---------- 舵机参数读写 ---------- */
void Robot_Arm_t::read_servo_overload(uint8_t servo_id) {
    uint8_t buf[1] = { servo_id };
    send_packet(CMD_SERVO_READ_OVERLOAD, buf, 1);
}

void Robot_Arm_t::write_servo_overload(uint8_t servo_id, uint8_t torque, uint8_t time_val) {
    uint8_t buf[3] = { servo_id, torque, time_val };
    send_packet(CMD_SERVO_WRITE_OVERLOAD, buf, 3);
}

void Robot_Arm_t::read_servo_baud(uint8_t servo_id) {
    uint8_t buf[1] = { servo_id };
    send_packet(CMD_SERVO_READ_BAUD, buf, 1);
}

void Robot_Arm_t::write_servo_baud(uint8_t servo_id, uint8_t baud) {
    uint8_t buf[2] = { servo_id, baud };
    send_packet(CMD_SERVO_WRITE_BAUD, buf, 2);
}

void Robot_Arm_t::read_servo_max_torque(uint8_t servo_id) {
    uint8_t buf[1] = { servo_id };
    send_packet(CMD_SERVO_READ_MAX_TORQUE, buf, 1);
}

void Robot_Arm_t::write_servo_max_torque(uint8_t servo_id, uint16_t torque) {
    uint8_t buf[3];
    buf[0] = servo_id;
    buf[1] = (uint8_t)(torque & 0xFF);
    buf[2] = (uint8_t)((torque >> 8) & 0xFF);
    send_packet(CMD_SERVO_WRITE_MAX_TORQUE, buf, 3);
}

void Robot_Arm_t::read_servo_angle_limit(uint8_t servo_id) {
    uint8_t buf[1] = { servo_id };
    send_packet(CMD_SERVO_READ_ANGLE_LIMIT, buf, 1);
}

void Robot_Arm_t::write_servo_angle_limit(uint8_t servo_id, int16_t min_angle, int16_t max_angle) {
    uint8_t buf[3];
    buf[0] = servo_id;
    buf[1] = (uint8_t)(min_angle & 0xFF);
    buf[2] = (uint8_t)(max_angle & 0xFF);
    send_packet(CMD_SERVO_WRITE_ANGLE_LIMIT, buf, 3);
}

/* ---------- 坐标限位 ---------- */
void Robot_Arm_t::send_coord_limits(void) {
    int16_t buf[6];
    buf[0] = (int16_t)limit_x_min;
    buf[1] = (int16_t)limit_x_max;
    buf[2] = (int16_t)limit_y_min;
    buf[3] = (int16_t)limit_y_max;
    buf[4] = (int16_t)limit_z_min;
    buf[5] = (int16_t)limit_z_max;
    send_packet(CMD_SET_COORD_LIMITS, (uint8_t *)buf, 12);
}

void Robot_Arm_t::request_coord_limits(void) {
    send_packet(CMD_GET_COORD_LIMITS, NULL, 0);
}


