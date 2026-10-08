# -*- coding: utf-8 -*-
"""NexArm 命令号表与载荷编解码。

来源与可信度标注：
  [FW]  ESP32 固件 `Global.h`（出厂程序 / 视觉分支一致）
  [GUI] NexArm.exe 反编译出的 nexarm_qt.constants + 各页面 send_* 函数
  [SDK] 厂商主机 SDK ros_robot_controller_sdk.py（产品资料内附带）
本文件中的载荷布局，凡三处同时出现者视为已交叉验证。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum


class Cmd(IntEnum):
    # [FW][GUI][SDK] 三处一致
    FIRMWARE_VERSION_CHECK = 1
    CHECK_BAT_LEVEL_CHECK = 2
    ACTION_GROUP_RUN = 3
    ACTION_GROUP_STOP = 4
    ACTION_GROUP_DOWNLOAD = 5
    FKINE_RESULT_GET = 6
    IKINE_RESULT_GET = 7
    COORDINATE_SET = 8
    BUZZER_SET = 9
    OLED_SET = 10
    GET_CUR_COORDS = 11
    OLED_ICON = 12
    SET_SINGLE_MOTOR = 13
    STOP_ALL_MOTOR = 14
    SET_MOTOR_SPEED = 15
    CONVEYOR_SET = 16
    STEPPER_RESET = 17
    STEPPER_DIV = 18
    STEPPER_RUN = 19
    BUTTON_EVENT = 22
    ACTION_GROUP_ERASE = 23
    SET_ESPNOW_CHANNEL = 30
    SET_GLOBAL_ACC = 31
    ESPNOW_SYNC_CTRL = 33
    MECANUM_CONTROL = 34
    TANK_CONTROL = 35
    SET_PEER_MAC = 36
    COLOR_TRACK = 40
    FACE_TRACK = 41
    SELF_LEARN_TRACK = 42
    APRILTAG_TRACK = 43
    APRILTAG_GRAB = 44
    APRILTAG_SET_OFFSET = 45
    COLOR_GRAB = 46
    LLM_CONTROL = 47
    GARBAGE_GRAB = 48
    CALIBRATION = 49
    ARM_MOVE_INC = 50
    ARM_SERVO_SINGLE = 51
    SET_SERVO_ID = 52
    SET_SERVO_MODE = 53
    ARM_RESET = 54
    READ_ALL_SERVOS = 55
    SET_MOVE_ACC = 56
    SET_POS_OFFSET = 57
    GET_POS_OFFSET = 58
    SET_PID_PARAM = 59
    GET_PID_PARAM = 60
    SET_TORQUE = 61
    SET_BT_MODE = 62
    SET_KINEMATICS_PARAM = 63
    GET_KINEMATICS_PARAM = 64
    GET_REAL_JOINT_ANGLES = 65
    GET_REAL_TCP_POSE = 66
    ARM_HOME = 67              # [FW] 部分分支无实现，使用前先探测
    LEROBOT_MODE = 68
    PC_SYNC_TEACH = 69
    SYNC_WRITE_SERVOS = 70
    GESTURE_TRACK = 81
    MOVE_INC = 82              # [FW] 别名
    SET_PS3_MAC = 83
    FACTORY_RESET = 84
    SET_CHASSIS_CONFIG = 85
    GET_CHASSIS_CONFIG = 86
    SCAN_WIFI_CHANNELS = 87
    SERVO_CALI_POS = 88
    SET_INTERP_MODE = 89       # [GUI][SDK] 有定义；本机出厂固件未见实现，慎用
    FIRMWARE_UPDATE = 90
    FW_START = 91
    FW_DATA = 92
    FW_END = 93
    FW_QUERY = 94
    SET_COORD_LIMITS = 79
    GET_COORD_LIMITS = 80
    ACTION_EDIT_ENTER = 120
    ACTION_EDIT_EXIT = 121
    ACTION_EDIT_START = 122
    ACTION_EDIT_STOP = 123
    ACTION_EDIT_PLAY = 124
    ACTION_EDIT_PLAY_STOP = 125
    ACTION_EDIT_CLEAR = 126
    ACTION_EDIT_QUERY = 127
    SYNC_TEACH_ENTER = 128
    SYNC_TEACH_EXIT = 129
    SYNC_TEACH_REC_START = 130
    SYNC_TEACH_REC_STOP = 131
    SYNC_TEACH_PLAY = 132
    SYNC_TEACH_PLAY_STOP = 133
    SYNC_TEACH_CLEAR = 134
    SYNC_TEACH_QUERY = 135


# ============================================================================
# 载荷打包
# ============================================================================
#: 绝对值坐标。**字段顺序是 (pitch, x, y, z, roll, claw)**，不是 (x,y,z,...)。
#: 来源 [GUI] CoordTab.send_coordinate_move / [FW] AT32 期待顺序 / [SDK] set_arm_coords
COORD_FMT = "<hhhhhhH"          # pitch*10, x, y, z, roll, claw, time_ms(u16)
#: 增量坐标。**字段顺序是 (dx, dy, dz, dpitch, droll, dclaw)** —— 与绝对值不同！
#: 来源 [SDK] arm_move_inc（GUI 侧同为 <hhhhhhH，仅字段顺序按此）
INC_FMT = "<hhhhhhH"


def pack_coord(x: float, y: float, z: float, pitch: float, roll: float,
               claw: float, time_ms: int) -> bytes:
    """cmd 8 COORDINATE_SET 的 14 字节载荷（pitch 放大 10 倍）。"""
    return struct.pack(COORD_FMT, int(round(pitch * 10)), int(x), int(y), int(z),
                       int(roll), int(claw), int(time_ms) & 0xFFFF)


def pack_joint_counts(position: int, acceleration: int, speed: int) -> bytes:
    """EXE slider payload: reg41, acc, raw position, reserved zero, speed.

    Verified against the user's HEX export. Position is encoder counts, not
    degrees; speed is a servo protocol value, not motion duration or mm/s.
    """
    for name, value, low, high in (
        ('position', position, 0, 4095),
        ('acceleration', acceleration, 0, 254),
        ('speed', speed, 1, 65535),
    ):
        if type(value) is not int or not low <= value <= high:
            raise ValueError('%s must be an integer in [%d,%d]' % (name, low, high))
    return struct.pack('<BBHHH', 41, acceleration, position, 0, speed)


def pack_move_inc(dx: float, dy: float, dz: float, dpitch: float, droll: float,
                  dclaw: float, time_ms: int) -> bytes:
    """cmd 50 ARM_MOVE_INC 的 14 字节载荷（dpitch 放大 10 倍）。"""
    return struct.pack(INC_FMT, int(dx), int(dy), int(dz), int(round(dpitch * 10)),
                       int(droll), int(dclaw), int(time_ms) & 0xFFFF)


def pack_u16(value: int) -> bytes:
    """cmd 54 ARM_RESET 的载荷：uint16 时长(ms)。[FW] 小端解析。"""
    return struct.pack("<H", int(value))


def pack_u8(value: int) -> bytes:
    """单字节参数：SET_TORQUE(61)、SET_MOVE_ACC(56)、SET_GLOBAL_ACC(31) 等。"""
    return struct.pack("<B", int(value) & 0xFF)


def pack_servo_offset(servo_id: int, offset: int) -> bytes:
    """cmd 57 SET_POS_OFFSET：'<Bh' = (id, offset)。[GUI] ServoAdvancedTab.set_pos_offset"""
    return struct.pack("<Bh", int(servo_id), int(offset))


def pack_pid(servo_id: int, p: int, i: int, d: int, threshold: int) -> bytes:
    """cmd 59 SET_PID_PARAM：'<BBBBH'。[GUI] ServoAdvancedTab.set_pid_param"""
    return struct.pack("<BBBBH", int(servo_id), int(p), int(i), int(d), int(threshold))


def pack_coord_limits(x_min, x_max, y_min, y_max, z_min, z_max,
                      p_min, p_max, r_min, r_max, c_min, c_max) -> bytes:
    """cmd 79 SET_COORD_LIMITS：12 × int16。

    单位混用，务必注意：x/y/z 是 **mm**，**pitch 是 0.1°（×10）**，roll/claw 是**度**。
    [GUI] SystemTab.set_coord_limits：打包前对下标 6、7（pmin/pmax）乘以 10；
          update_coord_limits 显示时再除以 10，UI 输入框范围 Pitch ±90°。
          实测回包 [-980, 120, -90, 90, -100, 83] ↔ UI 显示 -98.0 / 12.0。
    本函数入参统一用**度**，内部完成 ×10。
    """
    return struct.pack("<hhhhhhhhhhhh",
                       int(x_min), int(x_max), int(y_min), int(y_max),
                       int(z_min), int(z_max),
                       int(round(p_min * 10)), int(round(p_max * 10)),
                       int(r_min), int(r_max), int(c_min), int(c_max))


def pack_kinematics(arm_kin) -> bytes:
    """cmd 63 SET_KINEMATICS_PARAM：9 × float32 = 36 B。[FW] memcpy(arm_kin, 36)"""
    if len(arm_kin) != 9:
        raise ValueError("arm_kin must have exactly 9 floats")
    return struct.pack("<9f", *[float(v) for v in arm_kin])


def pack_buzzer(on_ms: int, off_ms: int, repeat: int, freq: int) -> bytes:
    """cmd 9 BUZZER_SET：'<IIHH'。[GUI] SystemTab.set_buzzer / [SDK] set_buzzer"""
    return struct.pack("<IIHH", int(on_ms), int(off_ms), int(repeat), int(freq))


def pack_oled_text(line: int, text: str) -> bytes:
    """cmd 10 OLED_SET：[line, strlen, utf-8 bytes...]。[SDK] set_oled_text"""
    raw = text.encode("utf-8")[:20]
    return bytes([int(line), len(raw)]) + raw


def pack_stepper_run(steps: int) -> bytes:
    """cmd 19 STEPPER_RUN：int32。[GUI] PeripheralTab.send_stepper"""
    return struct.pack("<i", int(steps))


def pack_single_motor(motor_idx: int, speed: int) -> bytes:
    """cmd 13 SET_SINGLE_MOTOR：'<Bb' = (M1..M4, -100..100)。[SDK]"""
    return struct.pack("<Bb", int(motor_idx), int(speed))


def pack_motor_speeds(s1: int, s2: int, s3: int, s4: int) -> bytes:
    """cmd 15 SET_MOTOR_SPEED：'<bbbb'。[SDK]"""
    return struct.pack("<bbbb", int(s1), int(s2), int(s3), int(s4))


def pack_action_group_run(group_id: int) -> bytes:
    """cmd 3 ACTION_GROUP_RUN：[group_id]。[FW] args[0] 即组号。"""
    return bytes([int(group_id) & 0xFF])


def pack_action_group_download(group_id: int, total_frames: int, frame_index: int,
                              blob: bytes) -> bytes:
    """cmd 5 ACTION_GROUP_DOWNLOAD：[group_id, total_frames, frame_index, 数据...]

    [FW] Nex_Arm_Board::action_group_download：frame_index==1 时以 "w" 建文件，
    否则追加；数据原样写入 /ActionGroupN.rob。分片长度受 MAX_ARGS=250 限制。
    """
    return bytes([int(group_id) & 0xFF, int(total_frames) & 0xFF,
                  int(frame_index) & 0xFF]) + bytes(blob)


# ============================================================================
# 回包解析
# ============================================================================
@dataclass(frozen=True)
class Pose:
    """cmd 11 GET_CUR_COORDS 回包（AT32 上报，24 字节）。

    [FW] system_task_handle.cpp：args[0..11] 依次是小端 int16 的
    x, y, z, pitch(×10), roll, claw；args[12..23] 是 6 个舵机脉冲。
    """
    x: int
    y: int
    z: int
    pitch: float
    roll: int
    claw: int
    servos: tuple

    def __str__(self) -> str:
        return ("Pose(x=%d, y=%d, z=%d, pitch=%.1f, roll=%d, claw=%d, "
                "servos=%s)" % (self.x, self.y, self.z, self.pitch, self.roll,
                                self.claw, list(self.servos)))


def decode_current_coords(args: bytes):
    """cmd 11 → Pose。数据不足 24 字节返回 None。"""
    if len(args) < 24:
        return None
    vals = struct.unpack("<hhhhhh", args[:12])
    servos = struct.unpack("<hhhhhh", args[12:24])
    return Pose(x=vals[0], y=vals[1], z=vals[2], pitch=vals[3] / 10.0,
                roll=vals[4], claw=vals[5], servos=tuple(servos))


def decode_real_joint_angles(args: bytes):
    """cmd 65 → [(pulse, angle_deg) × 6]。[SDK] get_real_joint_angles"""
    if len(args) < 24:
        return None
    out = []
    for i in range(6):
        pulse, angle_x10 = struct.unpack("<hh", args[i * 4:i * 4 + 4])
        out.append((pulse, angle_x10 / 10.0))
    return out


def decode_real_tcp_pose(args: bytes):
    """cmd 66 → dict(x,y,z,yaw,pitch,roll,claw)。角度 ×10。[SDK] get_real_tcp_pose"""
    if len(args) < 14:
        return None
    v = struct.unpack("<hhhhhhh", args[:14])
    return {"x": v[0], "y": v[1], "z": v[2], "yaw": v[3] / 10.0,
            "pitch": v[4] / 10.0, "roll": v[5] / 10.0, "claw": v[6] / 10.0}


def decode_battery_mv(args: bytes):
    """cmd 2 → 毫伏。'<H'。[SDK] get_battery"""
    if len(args) < 2:
        return None
    return struct.unpack("<H", args[:2])[0]


def decode_fw_version(args: bytes):
    """cmd 1 → (esp版, at32版)。[GUI] handle_packet：前 3 字节 ESP，其后 AT32。"""
    if len(args) < 6:
        return None
    esp = tuple(args[0:3])
    at32 = tuple(args[3:6])
    return {"esp": esp, "at32": at32,
            "esp_str": "%d.%d.%d" % esp, "at32_str": "%d.%d.%d" % at32}


def decode_coord_limits(args: bytes):
    """cmd 80 → 12 个限位，**pitch 已由 0.1° 换算回度**，其余原样。

    即返回 (xmin,xmax,ymin,ymax,zmin,zmax,pmin°,pmax°,rmin,rmax,cmin,cmax)。
    """
    if len(args) < 24:
        return None
    v = list(struct.unpack("<hhhhhhhhhhhh", args[:24]))
    v[6] /= 10.0
    v[7] /= 10.0
    return v


def decode_kinematics(args: bytes):
    """cmd 64 → 9 个 float。"""
    if len(args) < 36:
        return None
    return list(struct.unpack("<9f", args[:36]))


def decode_pos_offset(args: bytes):
    """cmd 58 → (servo_id, offset)。

    [GUI] CommManager._decode_servo_offset：offset 为 int16，合法区间
    [-2047, 2047]，中位基准 2048。
    """
    if len(args) < 3:
        return None
    sid, raw = struct.unpack("<Bh", args[:3])
    if not -2047 <= raw <= 2047:
        return None
    return sid, raw
