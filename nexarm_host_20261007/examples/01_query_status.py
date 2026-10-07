# -*- coding: utf-8 -*-
"""示例 1：只读体检（零运动风险）。

    python examples/01_query_status.py COM9

先跑这个再谈运动：确认版本、电压、当前位姿、坐标限位、舵机偏差都读得回来。
"""
import sys

from nexarm import ArmController


def main(port: str) -> int:
    arm = ArmController()
    arm.connect_serial(port)

    ver = arm.get_version()
    print("固件版本:", ver)
    mv = arm.get_battery_mv()
    print("电池      :", "%.2f V" % (mv / 1000.0) if mv else "读取失败")

    pose = arm.get_pose()
    print("当前位姿  :", pose)

    tcp = arm.get_tcp_pose()
    print("TCP 位姿  :", tcp)

    joints = arm.get_real_joints()
    if joints:
        print("真实关节  :", " ".join("J%d=%.1f°(%d)" % (i + 1, a, p)
                                      for i, (p, a) in enumerate(joints)))

    limits = arm.get_coord_limits()
    if limits:
        print("坐标限位  :", limits, "  (已同步到 arm.limits)")

    kin = arm.get_kinematics()
    if kin:
        print("运动学参数:", ["%.2f" % v for v in kin])

    print("舵机偏差  :", arm.get_pose_offsets())
    print("链路统计  :", arm.stats())
    arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else "COM9"))
