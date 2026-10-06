#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdint.h>
#include <stddef.h>

enum BluetoothMode {
    BT_MODE_BLE = 0,
    BT_MODE_PS3 = 1
};

#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define CMD_FIRMWARE_VERSION_CHECK  1
#define CMD_CHECK_BAT_LEVEL_CHECK   2

#define CMD_ACTION_GROUP_RUN        3
#define CMD_ACTION_GROUP_STOP       4
#define CMD_ACTION_GROUP_DOWNLOAD   5
#define CMD_FKINE_RESULT_GET        6
#define CMD_IKINE_RESULT_GET        7
#define CMD_COORDINATE_SET          8
#define CMD_BUZZER_SET              9
#define CMD_OLED_SET                10
#define CMD_GET_CUR_COORDS          11
#define CMD_OLED_ICON               12
#define CMD_SET_SINGLE_MOTOR        13
#define CMD_STOP_ALL_MOTOR          14
#define CMD_SET_MOTOR_SPEED         15

#define CMD_CONVEYOR_SET      16
#define CMD_STEPPER_RESET     17
#define CMD_STEPPER_DIV       18
#define CMD_STEPPER_RUN       19

#define CMD_ACTION_GROUP_ERASE      23
#define CMD_BUTTON_EVENT      22
#define CMD_SET_ESPNOW_CHANNEL      30
#define CMD_SET_GLOBAL_ACC          31
#define CMD_ESPNOW_SYNC_CTRL        33

#define CMD_MECANUM_CONTROL         34
#define CMD_TANK_CONTROL            35

#define CMD_SET_PEER_MAC            36

#define CMD_COLOR_TRACK             40
#define CMD_FACE_TRACK              41
#define CMD_SELF_LEARN_TRACK        42

#define CMD_APRILTAG_TRACK          43
#define CMD_APRILTAG_GRAB           44
#define CMD_APRILTAG_SET_OFFSET     45
#define CMD_COLOR_GRAB              46
#define CMD_LLM_CONTROL             47

#define CMD_GARBAGE_GRAB            48
#define CMD_CALIBRATION             49

#define CMD_ARM_MOVE_INC            50
#define CMD_ARM_SERVO_SINGLE        51

#define CMD_SET_SERVO_ID            52
#define CMD_SET_SERVO_MODE          53
#define CMD_ARM_RESET               54
#define CMD_READ_ALL_SERVOS         55
#define CMD_SET_MOVE_ACC            56

#define CMD_SET_POS_OFFSET          57
#define CMD_GET_POS_OFFSET          58
#define CMD_SET_PID_PARAM           59
#define CMD_GET_PID_PARAM           60
#define CMD_SET_TORQUE      61
#define CMD_SET_BT_MODE     62
#define CMD_SET_KINEMATICS_PARAM    63
#define CMD_GET_KINEMATICS_PARAM    64

#define CMD_GET_REAL_JOINT_ANGLES   65
#define CMD_GET_REAL_TCP_POSE       66
#define CMD_ARM_HOME                67

#define CMD_LEROBOT_MODE            68

#define CMD_PC_SYNC_TEACH           69
#define CMD_SYNC_WRITE_SERVOS       70
#define CMD_SERVO_READ_OVERLOAD     71
#define CMD_SERVO_WRITE_OVERLOAD    72
#define CMD_SERVO_READ_BAUD         73
#define CMD_SERVO_WRITE_BAUD        74
#define CMD_SERVO_READ_MAX_TORQUE   75
#define CMD_SERVO_WRITE_MAX_TORQUE  76
#define CMD_SERVO_READ_ANGLE_LIMIT  77
#define CMD_SERVO_WRITE_ANGLE_LIMIT 78
#define CMD_SET_COORD_LIMITS        79
#define CMD_GET_COORD_LIMITS        80
#define CMD_SERVO_CALI_POS          88
#define CMD_GESTURE_TRACK           81
#define CMD_MOVE_INC                82
#define CMD_SET_PS3_MAC             83
#define CMD_FACTORY_RESET           84
#define CMD_SET_CHASSIS_CONFIG      85
#define CMD_GET_CHASSIS_CONFIG      86
#define CMD_SCAN_WIFI_CHANNELS      87

#define CMD_FIRMWARE_UPDATE         90
#define CMD_FW_START                91
#define CMD_FW_DATA                 92
#define CMD_FW_END                  93
#define CMD_FW_QUERY                94

#define CMD_ACTION_EDIT_ENTER       120
#define CMD_ACTION_EDIT_EXIT        121
#define CMD_ACTION_EDIT_START       122
#define CMD_ACTION_EDIT_STOP        123
#define CMD_ACTION_EDIT_PLAY        124
#define CMD_ACTION_EDIT_STOP_PLAY   125
#define CMD_ACTION_EDIT_CLEAR       126
#define CMD_ACTION_EDIT_STATUS      127

#define CMD_SYNC_TEACH_ENTER        128
#define CMD_SYNC_TEACH_EXIT         129
#define CMD_SYNC_TEACH_START        130
#define CMD_SYNC_TEACH_STOP         131
#define CMD_SYNC_TEACH_PLAY         132
#define CMD_SYNC_TEACH_STOP_PLAY    133
#define CMD_SYNC_TEACH_CLEAR        134
#define CMD_SYNC_TEACH_STATUS       135

enum ChassisType {
    CHASSIS_NONE    = 0,
    CHASSIS_MECANUM = 1,
    CHASSIS_TANK    = 2
};
#endif
