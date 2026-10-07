# -*- coding: utf-8 -*-
"""nexarm —— NexArm 六轴机械臂主机侧运动控制库（纯 Python，仅依赖 pyserial）。

最小用法：
    from nexarm import ArmController, Cmd
    arm = ArmController()
    arm.connect_serial("COM9")
    arm.enable_motion(True)
    arm.home()
    print(arm.get_pose())
"""
from .arm import HOME_POSE, ArmController
from .commands import (Cmd, Pose, decode_battery_mv, decode_coord_limits,
                       decode_current_coords, decode_fw_version, decode_kinematics,
                       decode_pos_offset, decode_real_joint_angles,
                       decode_real_tcp_pose, pack_action_group_download,
                       pack_action_group_run, pack_buzzer, pack_coord,
                       pack_coord_limits, pack_kinematics, pack_move_inc,
                       pack_oled_text, pack_pid, pack_servo_offset, pack_u16,
                       pack_u8)
from .frames import AT32_SYS_ID, SYS_HEAD, Frame, FrameParser, build_frame, checksum_inv8
from .safety import DryRunPlan, MotionGate, SafetyError, WorkspaceLimits
from .transport import DEFAULT_BAUDRATE, Transport, TransportError

__all__ = [
    "ArmController", "HOME_POSE", "Cmd", "Pose", "Frame", "FrameParser",
    "build_frame", "checksum_inv8", "SYS_HEAD", "AT32_SYS_ID",
    "Transport", "TransportError", "DEFAULT_BAUDRATE",
    "WorkspaceLimits", "MotionGate", "SafetyError", "DryRunPlan",
    "pack_coord", "pack_move_inc", "pack_coord_limits", "pack_kinematics",
    "pack_buzzer", "pack_oled_text", "pack_pid", "pack_servo_offset",
    "pack_u8", "pack_u16", "pack_action_group_run", "pack_action_group_download",
    "decode_current_coords", "decode_real_joint_angles",
    "decode_real_tcp_pose", "decode_battery_mv", "decode_fw_version",
    "decode_coord_limits", "decode_kinematics", "decode_pos_offset",
]
__version__ = "0.1.0"
