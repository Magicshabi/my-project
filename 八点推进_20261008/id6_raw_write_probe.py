# -*- coding: utf-8 -*-
"""S3 分叉实验：改用 GUI 同款"原始舵机写帧"，并对比 ID 寻址方式。

    python 八点推进_20261008/id6_raw_write_probe.py COM9 --go

背景（从 NexArm.exe 反编译得到的 GUI 真实行为）：
  ServoTab.send_servo_cmd → comm_manager.send_packet(?, SERVO_CMD_WRITE, args)
  args = [SERVO_REG_ACC(41), acc, pos(int16LE), 0, 0, speed(int16LE)]   ← 8 字节
  厂商 SDK 的同类调用是 buf_write(servo_id, 3, data)，即 **ID = 舵机号**。
  但本工程固件里存在 `if (elements.id != 0xFF) return;` 的早退分支，
  因此 ID 到底是「舵机号」还是「0xFF」必须实测判定 —— 本脚本分两阶段各试一次。

与 CMD51 的区别：
  CMD51  = 上层封装（ID=0xFF, CMD=51, <B h H>），AT32 内部再转换；
  本脚本 = 直接写舵机寄存器（reg41 后的 acc/pos/PWM/speed），路径更接近 GUI 滑块。

判定仍为现场目视（D1：cmd11/cmd65 同源，读数不作为动作证据）。
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "舵机基础运行逻辑"))

from nexarm import ArmController  # noqa: E402
from nexarm.frames import build_frame  # noqa: E402

SERVO_ID = 6
SAFE_LO, SAFE_HI = 1100, 1900
AMPLITUDE = 45          # 默认 ±45 ticks（≈4°）；可用 --ticks N 覆盖
MOVE_MS = 1000
CYCLES = 2

SERVO_CMD_WRITE = 3      # 舵机帧 cmd：3 = 写寄存器
SERVO_REG_ACC = 41       # 0x29，加速度寄存器（其后跟 acc,pos,PWM,speed）
ACC = 100                # GUI 默认
SPEED = 2000             # GUI 默认


def parse_ticks(argv, default=AMPLITUDE):
    if "--ticks" in argv:
        return int(argv[argv.index("--ticks") + 1])
    return default


def parse_cycles(argv, default=CYCLES):
    if "--cycles" in argv:
        return int(argv[argv.index("--cycles") + 1])
    return default


def raw_args(pos, acc=ACC, speed=SPEED):
    """GUI/SDK 同款 8 字节： [41, acc, pos(int16LE), 0, 0, speed(int16LE)]"""
    return bytes([SERVO_REG_ACC, acc]) + struct.pack("<hHh", int(pos), 0, int(speed))


def main(argv):
    port = argv[1] if len(argv) > 1 else "COM9"
    if "--go" not in argv:
        print("需要 --go")
        return 1

    arm = ArmController()
    try:
        arm.connect_serial(port)
    except Exception as exc:
        print("[ERR] 打开串口失败: %s（若是被占用，请关闭 NexArm.exe）" % exc)
        return 2

    amp = parse_ticks(argv)
    cycles = parse_cycles(argv)

    try:
        print("[S1] 版本: %s" % arm.get_version())
        j = arm.get_real_joints()
        cur = j[SERVO_ID - 1][0] if j else None
        print("[S1] ID%d 当前位置 = %s" % (SERVO_ID, cur))
        if cur is None or not SAFE_LO <= cur <= SAFE_HI:
            print("[ERR] 位置 %s 超出安全区间 [%d,%d]，终止" % (cur, SAFE_LO, SAFE_HI))
            return 3

        lo = max(SAFE_LO, cur - amp)
        hi = min(SAFE_HI, cur + amp)
        print("[S3] 幅度 = ±%d ticks (≈%.1f°)，%d 个来回" % (amp, amp * 360.0 / 4096, cycles))

        for phase, target_id in (("A: ID=6（舵机号寻址）", SERVO_ID),
                                 ("B: ID=0xFF（广播寻址）", 0xFF)):
            print()
            print("=" * 62)
            print("★ %s —— 接下来 8 秒请盯住夹爪 ★" % phase)
            print("=" * 62)
            time.sleep(2.0)
            for i in range(cycles):
                for name, pos in (("张开", lo), ("回位", hi)):
                    frame = build_frame(SERVO_CMD_WRITE, raw_args(pos), target_id)
                    print("[S3] %s 第%d轮 %s -> ID%d 目标 %d, TX %s"
                          % (phase[0], i + 1, name, SERVO_ID, pos, frame.hex()))
                    arm.transport.send(SERVO_CMD_WRITE, raw_args(pos), target_id)
                    time.sleep(MOVE_MS / 1000.0 + 0.8)
                    j = arm.get_real_joints()
                    if j:
                        pulse, ang = j[SERVO_ID - 1]
                        print("[S3]      读数: pulse=%d (%.2f°)" % (pulse, ang))
                    else:
                        print("[S3]      读数: 无回包")
            frame = build_frame(SERVO_CMD_WRITE, raw_args(cur), target_id)
            print("[S3] 回位 -> 目标 %d, TX %s" % (cur, frame.hex()))
            arm.transport.send(SERVO_CMD_WRITE, raw_args(cur), target_id)
            time.sleep(MOVE_MS / 1000.0 + 0.8)
            print("★ %s 阶段结束，请记住这一段有没有动 ★" % phase[0])
            time.sleep(3.0)

        print()
        print("[SUM] 结束读数: %s" % arm.get_real_joints())
        print("[SUM] 链路: %s" % arm.stats())
        print()
        print("=" * 62)
        print("★ 请回答：A 阶段（ID=6）动了吗？B 阶段（ID=0xFF）动了吗？")
        print("=" * 62)
        return 0
    finally:
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
