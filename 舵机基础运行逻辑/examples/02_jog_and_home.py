# -*- coding: utf-8 -*-
"""示例 2：先干跑核对，再真动（默认不动，必须加 --go）。

    python examples/02_jog_and_home.py COM9            # 只打印计划，不发运动指令
    python examples/02_jog_and_home.py COM9 --go       # 真正下发

动作序列刻意做小：回中 -> 抬 Z 20mm -> 前进 30mm -> 回中。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import ArmController  # noqa: E402


def main(argv) -> int:
    port = argv[1] if len(argv) > 1 else "COM9"
    go = "--go" in argv

    if not go:
        # dry_run：不发任何运动帧，只把动作序列和越界检查跑一遍
        arm = ArmController(dry_run=True)
        arm.enable_motion(True)
        arm.home(time_ms=1000, wait=False)
        arm.jog(dz=20, time_ms=500)
        arm.jog(dx=30, time_ms=500)
        arm.home(time_ms=1000, wait=False)
        print("将要执行的动作序列：")
        print(arm.plan)
        print("\n（这是干跑。确认无误后加 --go 真正执行。）")
        return 0

    arm = ArmController()
    arm.connect_serial(port)
    arm.enable_motion(True)
    arm.set_acc(30)              # 起步加速度：0 最快，越大越平滑
    arm.set_torque(True)         # 力矩关闭时坐标指令不会让臂动

    print("初始位姿:", arm.get_pose())

    arm.home(time_ms=1200, wait=True)
    print("回中完成:", arm.get_pose())

    arm.jog(dz=20, time_ms=500, wait=True)
    print("抬 Z 后  :", arm.get_pose())

    arm.jog(dx=30, time_ms=500, wait=True)
    print("前进 后  :", arm.get_pose())

    arm.home(time_ms=1200, wait=True)
    print("回到中位 :", arm.get_pose())
    arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
