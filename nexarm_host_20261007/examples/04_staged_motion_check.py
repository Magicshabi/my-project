# -*- coding: utf-8 -*-
"""分级运动检查：把"第一次让臂动"拆成可回退的小步，每步都读位姿核对。

    python examples/04_staged_motion_check.py COM9          # 干跑，只打印
    python examples/04_staged_motion_check.py COM9 --go     # 真动

分级理由：
  stage1 只抬 10mm —— 唯一方向确定、绝不撞桌面的动作，用来判定"写路径通不通"；
  stage2 落回原位；stage3 才做中等幅度移动（目标用固件自己的工作姿态 pitch=-90，
        而不是 GUI 的 pitch=0，避免腕部一次摆 100°）；
  stage4 前后各 30mm 并归位。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import ArmController  # noqa: E402

STANCE = dict(x=200.0, y=0.0, z=200.0, pitch=-90.0, roll=0.0)  # 固件工作姿态


def show(tag, arm):
    pose = arm.get_pose()
    print("  %-14s %s" % (tag, pose))
    return pose


def main(argv) -> int:
    port = argv[1] if len(argv) > 1 else "COM9"
    go = "--go" in argv

    arm = ArmController(dry_run=not go)
    if go:
        arm.connect_serial(port)
    arm.enable_motion(True)

    print("=== stage 0 基线 ===")
    ver = None if arm.dry_run else arm.get_version()
    if ver:
        print("  版本:", ver["esp_str"], "/", ver["at32_str"])
    limits = None if arm.dry_run else arm.get_coord_limits()
    if limits:
        print("  限位(真机):", limits)
    base = None if arm.dry_run else arm.get_pose()
    claw = base.claw if base else -60.0
    print("  基线位姿:", base, " claw 将保持 =", claw)

    print("=== stage 1 抬升 10mm（判定写路径） ===")
    arm.jog(dz=10, time_ms=400)
    if go:
        time.sleep(1.2)
        p1 = show("after +Z10", arm)
        moved = abs(p1.z - base.z) if p1 else 0
        print("  Δz = %d mm  -> %s" % (moved, "写路径已通" if moved >= 3 else
                                      "没动！检查力矩"))
        if moved < 3:
            print("  停止后续动作，避免盲动。")
            arm.close()
            return 2
    else:
        print("  (干跑，未发送)")

    print("=== stage 2 落回原位 ===")
    arm.jog(dz=-10, time_ms=400)
    if go:
        time.sleep(1.2)
        show("after -Z10", arm)

    print("=== stage 3 移动到固件工作姿态 (200, 0, 200, pitch -90) ===")
    arm.move_to(STANCE["x"], STANCE["y"], STANCE["z"], STANCE["pitch"],
                STANCE["roll"], claw, 1800, wait=go)
    if go:
        show("at stance", arm)

    print("=== stage 4 前后各 30mm 并归位 ===")
    arm.jog(dx=30, time_ms=700, wait=go)
    if go:
        show("after +X30", arm)
    arm.jog(dx=-30, time_ms=700, wait=go)
    if go:
        final = show("after -X30", arm)
        print("  链路统计:", arm.stats())
        print("  与基线差异: dx=%+d dy=%+d dz=%+d"
              % (final.x - base.x, final.y - base.y, final.z - base.z))

    if not go:
        print("\n干跑计划：")
        print(arm.plan)
        print("\n（加 --go 才会真动）")

    arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
