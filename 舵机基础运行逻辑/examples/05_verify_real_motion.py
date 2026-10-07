# -*- coding: utf-8 -*-
"""三通道对照：判断"数值变了"到底是舵机真转了，还是 AT32 只在改内部状态。

    python examples/05_verify_real_motion.py COM9          # 只读对照（不运动）
    python examples/05_verify_real_motion.py COM9 --go     # 做一次大而慢的移动再对照

三条独立通道：
  A  cmd 11  GET_CUR_COORDS      —— AT32 内部状态（P0「幻象轨迹」就是它变了而舵机没写）
  B  cmd 65  GET_REAL_JOINT_ANGLES —— 固件声称"基于舵机实际读回位置算的"
  C  舵机寄存器直读 reg 56       —— 透过 ESP32/AT32 直问 HX-30HM 本体（若固件支持）
判定：A 变而 B 不变 => 内部状态更新、舵机没动；A/B/C 同步变 => 真的动了。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import ArmController  # noqa: E402

TICKS_PER_DEG = 4096.0 / 360.0
TARGET = dict(x=250.0, y=0.0, z=260.0, pitch=-90.0, roll=0.0, claw=-60.0)


def deg(ticks):
    return ticks / TICKS_PER_DEG


def read_all(arm, tag):
    pose = arm.get_pose()
    joints = arm.get_real_joints()
    b = [j[0] for j in joints] if joints else None
    print("  [%s] A(cmd11) servos = %s" % (tag, list(pose.servos) if pose else None))
    print("  [%s]         pose   = x=%s y=%s z=%s pitch=%s" % (
        tag, pose.x if pose else "-", pose.y if pose else "-",
        pose.z if pose else "-", pose.pitch if pose else "-"))
    print("  [%s] B(cmd65) pulse  = %s" % (tag, b))
    return pose, b


def main(argv) -> int:
    port = argv[1] if len(argv) > 1 else "COM9"
    go = "--go" in argv

    arm = ArmController()
    arm.connect_serial(port)
    arm.enable_motion(True)

    print("=== 1) 静止基线：连读两次，确认读数稳定 ===")
    p0, b0 = read_all(arm, "base1")
    time.sleep(0.6)
    p1, b1 = read_all(arm, "base2")
    stable_a = (list(p0.servos) == list(p1.servos)) if (p0 and p1) else False
    stable_b = (b0 == b1)
    print("  静止稳定性: A=%s  B=%s（都为 True 才说明读数可信）" % (stable_a, stable_b))

    print("\n=== 2) 通道 C：舵机寄存器直读（可能不支持） ===")
    direct = arm.read_servo_positions()
    if direct:
        for sid, (raw, cand) in direct.items():
            print("  舵机 %d: args=%s  %s" % (sid, raw, cand))
    else:
        print("  无回包 —— 当前固件未把舵机透传数据推到 USB（预期内，不影响 A/B 判定）")

    if not go:
        print("\n（只读模式结束；加 --go 才会做大而慢的移动）")
        print("链路:", arm.stats())
        arm.close()
        return 0

    print("\n=== 3) 执行大而慢的移动：-> (250, 0, 260, pitch -90)，2500ms ===")
    print("  （相对当前位姿大约 +50mm X、+60mm Z，肉眼应明显可见）")
    arm.move_to(TARGET["x"], TARGET["y"], TARGET["z"], TARGET["pitch"],
                TARGET["roll"], TARGET["claw"], 2500, wait=False)
    for i in range(6):
        time.sleep(0.9)
        read_all(arm, "t+%.1fs" % (0.9 * (i + 1)))

    p_end, b_end = read_all(arm, "final")
    print("\n=== 4) 判定 ===")
    if p0 and b0 and p_end and b_end:
        da = [x - y for x, y in zip(list(p_end.servos), list(p0.servos))]
        db = [x - y for x, y in zip(b_end, b0)]
        print("  A(cmd11) 变化(ticks):", da, " = %.1f° 量级" % max(abs(v) for v in da))
        print("  B(cmd65) 变化(ticks):", db, " = %.1f° 量级" % max(abs(v) for v in db))
        if max(abs(v) for v in db) < 3:
            print("  => 结论：**A 变了但 B 几乎没变，舵机没有真正转动**（内部状态更新型故障）")
        else:
            print("  => 结论：**B 同步变化，舵机确实转动了**")
    print("  链路:", arm.stats())
    arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
