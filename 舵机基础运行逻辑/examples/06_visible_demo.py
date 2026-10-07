# -*- coding: utf-8 -*-
"""肉眼可辨的演示：夹爪开合 ×3 + 垂直大行程来回。

    python examples/06_visible_demo.py COM9 --go

为什么用这两个动作：
  夹爪在臂末端、开合约 60°，是最不可能被忽视的动作；
  垂直 90mm 行程慢速走，肉眼一定看得到。
如果这两个动作都看不见，而软件读数却在变 —— 那就是 P0 那种
"内部状态更新、舵机没写"的故障复发，必须停下排查供电/力矩/总线。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import ArmController  # noqa: E402


def main(argv) -> int:
    port = argv[1] if len(argv) > 1 else "COM9"
    if "--go" not in argv:
        print("需要 --go 才动。用法: python examples/06_visible_demo.py COM9 --go")
        return 1

    arm = ArmController()
    arm.connect_serial(port)
    arm.enable_motion(True)

    p = arm.get_pose()
    print("起始位姿:", p)
    base_z = p.z

    print("\n--- 第 0 组：蜂鸣器 ×3（听得到就说明帧被 ESP32 执行了）---")
    arm.buzzer(150, 150, 3, 2000)
    time.sleep(1.5)

    print("\n--- 第 1 组：夹爪开合 ×3（张 -60° / 合 0°）---")
    for i in range(3):
        print("  第 %d 次：张开" % (i + 1))
        arm.set_claw(-60.0, time_ms=700)
        time.sleep(0.3)
        print("  第 %d 次：闭合" % (i + 1))
        arm.set_claw(0.0, time_ms=700)
        time.sleep(0.3)

    print("\n--- 第 2 组：垂直抬起 90mm 再落下 ---")
    up_z = min(base_z + 90.0, 480.0)
    print("  抬升: z %.0f -> %.0f（2500ms，慢慢走）" % (base_z, up_z))
    arm.move_to(p.x, p.y, up_z, p.pitch, p.roll, 0.0, 2500)
    print("  到位:", arm.get_pose())
    time.sleep(1.0)
    print("  落下: z %.0f -> %.0f" % (up_z, base_z))
    arm.move_to(p.x, p.y, base_z, p.pitch, p.roll, 0.0, 2500)
    print("  到位:", arm.get_pose())

    print("\n--- 最终 ---")
    print("  位姿:", arm.get_pose())
    print("  真实关节:", arm.get_real_joints())
    print("  链路:", arm.stats())
    arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
