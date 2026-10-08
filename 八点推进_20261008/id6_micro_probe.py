# -*- coding: utf-8 -*-
"""S2 判定实验：ID6（夹爪）微动，判定"指令能否真的驱动舵机"。

    python 八点推进_20261008/id6_micro_probe.py COM9 --go

为什么是 ID6 / 这个幅度：
  * ID6 是夹爪，空载时行程独立、不牵扯臂体，是最低风险的关节；
  * 幅度取 ±45 ticks（≈4°）—— Codex 此前用 ±5~8 ticks（0.4~0.7°），
    低于舵机死区/机械回差的可观测阈值，测不出任何结论（见 PROJECT_STATE.md 勘误 E5）；
  * 硬夹持在 [1100, 1900]：八点数据中 ID6 实测范围是 1097~1975。

判定纪律（见 PROJECT_STATE.md D1）：cmd11 与 cmd65 逐字节同源，
**不能用它们的数值变化当作"舵机动了"的证据** —— 所以本脚本只负责发帧与留档，
最终判定由现场目视给出。
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "舵机基础运行逻辑"))

from nexarm import ArmController, Cmd  # noqa: E402
from nexarm.frames import build_frame  # noqa: E402

SERVO_ID = 6
SAFE_LO, SAFE_HI = 1100, 1900        # 由八点实测范围 1097~1975 收窄
AMPLITUDE = 45                        # ticks ≈ 4.0°
MOVE_MS = 1000
CYCLES = 2


def line(tag, msg):
    print("[%s] %s" % (tag, msg))


def main(argv):
    port = argv[1] if len(argv) > 1 else "COM9"
    if "--go" not in argv:
        print("需要 --go。用法: python 八点推进_20261008/id6_micro_probe.py COM9 --go")
        return 1

    arm = ArmController()
    try:
        arm.connect_serial(port)
    except Exception as exc:
        line("ERR", "打开串口失败: %s" % exc)
        line("ERR", "若是被占用，请先关闭 NexArm.exe")
        return 2

    log = []
    try:
        line("S1", "=== 只读基线 ===")
        ver = arm.get_version()
        line("S1", "版本: %s" % ver)
        p1 = arm.get_pose()
        time.sleep(0.4)
        p2 = arm.get_pose()
        line("S1", "cmd11 第一次: %s" % p1)
        line("S1", "cmd11 第二次: %s" % p2)
        line("S1", "cmd11 两次一致: %s" % (p1 and p2 and list(p1.servos) == list(p2.servos)))
        j1 = arm.get_real_joints()
        time.sleep(0.3)
        j2 = arm.get_real_joints()
        b1 = [x[0] for x in j1] if j1 else None
        b2 = [x[0] for x in j2] if j2 else None
        line("S1", "cmd65 第一次: %s" % b1)
        line("S1", "cmd65 第二次: %s" % b2)
        line("S1", "cmd65 两次一致: %s" % (b1 == b2))
        line("S1", "链路: %s" % arm.stats())

        start = b1[SERVO_ID - 1] if b1 else None
        if start is None:
            line("ERR", "读不到 ID%d 当前位置，终止" % SERVO_ID)
            return 3
        line("S1", "ID%d 当前位置 = %d ticks (%.2f°)" % (SERVO_ID, start, start * 360.0 / 4096))
        if not SAFE_LO <= start <= SAFE_HI:
            line("ERR", "当前位置 %d 超出安全区间 [%d, %d]，终止" % (start, SAFE_LO, SAFE_HI))
            return 4

        line("S3-2", "=== 补一次力矩 CMD_SET_TORQUE(61)=1 ===")
        f = build_frame(int(Cmd.SET_TORQUE), bytes([1]))
        line("S3-2", "TX %s" % f.hex())
        log.append(("set_torque", f.hex()))
        arm.transport.send(int(Cmd.SET_TORQUE), bytes([1]))
        time.sleep(0.5)

        targets_open = max(SAFE_LO, start - AMPLITUDE)
        targets_close = min(SAFE_HI, start + AMPLITUDE)
        line("S2", "=== 微动 %d 个来回：%d -> %d -> %d（每次 %dms） ==="
             % (CYCLES, start, targets_open, targets_close, MOVE_MS))
        line("S2", "★ 请盯住夹爪 ★  现在开始")

        for i in range(CYCLES):
            for name, target in (("张开", targets_open), ("回位", targets_close)):
                frame = build_frame(int(Cmd.ARM_SERVO_SINGLE),
                                    __import__("struct").pack("<BhH", SERVO_ID,
                                                              target, MOVE_MS))
                line("S2", "第%d轮 %s -> ID%d 目标 %d, TX %s"
                     % (i + 1, name, SERVO_ID, target, frame.hex()))
                log.append(("cmd51", frame.hex(), target))
                arm.transport.send(int(Cmd.ARM_SERVO_SINGLE),
                                   __import__("struct").pack("<BhH", SERVO_ID,
                                                             target, MOVE_MS))
                time.sleep(MOVE_MS / 1000.0 + 1.0)
                now = arm.get_real_joints()
                if now:
                    pulse, ang = now[SERVO_ID - 1]
                    line("S2", "    读数: pulse=%d (%.2f°)" % (pulse, ang))
                else:
                    line("S2", "    读数: 无回包")

        line("S2", "=== 回到起始位置 %d ===" % start)
        import struct
        arm.transport.send(int(Cmd.ARM_SERVO_SINGLE),
                           struct.pack("<BhH", SERVO_ID, start, MOVE_MS))
        time.sleep(MOVE_MS / 1000.0 + 1.0)

        line("SUM", "结束位姿: %s" % arm.get_pose())
        line("SUM", "链路: %s" % arm.stats())
        line("SUM", "本次发送的帧（黄金样本用）:")
        for item in log:
            line("SUM", "   %s" % (item,))
        print()
        print("=" * 62)
        print("★ 请回答：夹爪在刚才的 8 秒里有没有明显张开/闭合的动作？")
        print("  回答决定下一步：")
        print("    动  -> 写黄金样本，执行链确认活着，再谈八点")
        print("    没动 -> 换 GUI 同款原始写帧（ID=6, CMD=3, [41,acc,pos,0,spd]）再试")
        print("=" * 62)
        return 0
    finally:
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
