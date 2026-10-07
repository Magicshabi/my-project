# -*- coding: utf-8 -*-
"""离线自检：不接硬件，验证帧编解码与载荷布局是否与厂商实现一致。

运行： python tests/test_codec.py
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import (Cmd, FrameParser, build_frame, checksum_inv8, decode_battery_mv,
                    decode_coord_limits, decode_current_coords, decode_pos_offset,
                    decode_real_tcp_pose, pack_coord, pack_coord_limits,
                    pack_kinematics, pack_move_inc)
from nexarm.commands import pack_buzzer
from nexarm.arm import ArmController
from nexarm.safety import SafetyError

FAILS = []


def check(name, got, want):
    ok = got == want
    print("%-58s %s" % (name, "OK" if ok else "FAIL got=%r want=%r" % (got, want)))
    if not ok:
        FAILS.append(name)


def expect_raises(name, exc, fn):
    try:
        fn()
    except exc:
        print("%-58s OK (raised %s)" % (name, exc.__name__))
        return
    print("%-58s FAIL (no %s)" % (name, exc.__name__))
    FAILS.append(name)


# 1) 帧组装：与厂商 SDK buf_write 逐字节一致
check("checksum_inv8([0xFF,0x02,0x02])", checksum_inv8(bytes([0xFF, 2, 2])), 0xFC)
check("build_frame(CMD=2) == SDK 构造", build_frame(2),
      bytes([0xFF, 0xFF, 0xFF, 0x02, 0x02, 0xFC]))
check("build_frame(CMD=11) 真实值", build_frame(11),
      bytes([0xFF, 0xFF, 0xFF, 0x02, 0x0B, 0xF3]))
check("build_frame(CMD=1) 真实值", build_frame(1),
      bytes([0xFF, 0xFF, 0xFF, 0x02, 0x01, 0xFD]))

# 2) 坐标载荷：'<hhhhhhH' = (pitch*10, x, y, z, roll, claw, time)
check("pack_coord(200,0,200,0,0,-60,1000)",
      pack_coord(200, 0, 200, 0, 0, -60, 1000),
      struct.pack("<hhhhhhH", 0, 200, 0, 200, 0, -60, 1000))
check("pack_coord pitch 放大 10 倍",
      pack_coord(200, 0, 200, 45.0, 0, 0, 1000)[:2], struct.pack("<h", 450))
check("pack_coord 长度 14 字节", len(pack_coord(1, 2, 3, 4, 5, 6, 7)), 14)
check("pack_coord 时间上限 u16 不溢出",
      pack_coord(1, 2, 3, 4, 5, 6, 65535)[-2:], struct.pack("<H", 65535))

# 3) 增量载荷：字段顺序 (dx,dy,dz,dpitch*10,droll,dclaw,time) —— 与绝对值不同
check("pack_move_inc(10,0,0,5,0,0,600)",
      pack_move_inc(10, 0, 0, 5, 0, 0, 600),
      struct.pack("<hhhhhhH", 10, 0, 0, 50, 0, 0, 600))

# 4) 其它载荷
check("pack_coord_limits 24 字节", len(pack_coord_limits(*range(12))), 24)
# 限位量纲：pitch 走 0.1°，其余原样（GUI set_coord_limits 对下标 6/7 ×10）
check("pack_coord_limits pitch ×10（入参是度）",
      pack_coord_limits(50, 450, -350, 350, 0, 500, -90, 12, -90, 90, -60, 30),
      struct.pack("<hhhhhhhhhhhh", 50, 450, -350, 350, 0, 500,
                  -900, 120, -90, 90, -60, 30))
check("decode_coord_limits pitch 还原为度",
      decode_coord_limits(struct.pack("<hhhhhhhhhhhh", 100, 400, -450, 450, 20,
                                       500, -980, 120, -90, 90, -100, 83))[6:8],
      [-98.0, 12.0])
check("pack_kinematics 36 字节", len(pack_kinematics([1.5] * 9)), 36)
check("pack_kinematics float32 校验",
      pack_kinematics([110.45] * 9)[:4], struct.pack("<f", 110.45))
check("pack_buzzer '<IIHH'", pack_buzzer(100, 100, 2, 2000),
      struct.pack("<IIHH", 100, 100, 2, 2000))
expect_raises("pack_kinematics 参数个数错误", ValueError,
              lambda: pack_kinematics([1.0] * 8))

# 5) 流式解析：普通垃圾 + 半帧 + 校验错误 都要能自愈
PARK = struct.pack("<hhhhhh", 2048, 2048, 2048, 2048, 2048, 2048)
parser = FrameParser()
stream = (b"\x00\x11\x22\x33"                      # 普通垃圾字节
          + build_frame(Cmd.CHECK_BAT_LEVEL_CHECK, b"\x34\x12")   # 合法：0x1234 mV
          + b"\xFF\xFF\xFF\x02\x0B\x00"            # 校验错误的帧
          + build_frame(Cmd.GET_CUR_COORDS,
                        struct.pack("<hhhhhh", 200, 0, 200, 0, 0, -60) + PARK))
frames = []
for i in range(len(stream)):                       # 故意逐字节喂
    frames.extend(parser.feed(stream[i:i + 1]))
check("逐字节流式解析出的帧数", len(frames), 2)
check("第 1 帧 cmd", frames[0].cmd, int(Cmd.CHECK_BAT_LEVEL_CHECK))
check("校验错误被计数", parser.checksum_errors, 1)
check("电池毫伏解析", decode_battery_mv(frames[0].args), 0x1234)
pose = decode_current_coords(frames[1].args)
check("位姿 x/z/claw", (pose.x, pose.z, pose.claw), (200, 200, -60))
check("位姿 servos", pose.servos, (2048, 2048, 2048, 2048, 2048, 2048))

# 5b) 协议固有歧义：紧贴真帧头的 0xFF 0xFF 会被当成本帧的 ID/LENGTH，吃掉一帧。
#     所以下位机绝不能在帧之间粘 0xFF 填充；调用方需容忍单帧丢失并重试。
p2 = FrameParser()
check("假帧头吃掉一帧（0xFF 填充是禁忌）",
      len(p2.feed(b"\xFF\xFF" + build_frame(Cmd.GET_CUR_COORDS, PARK))), 0)
check("随后自动重同步", len(p2.feed(build_frame(Cmd.GET_CUR_COORDS, PARK))), 1)

# 6) 回包解码
check("decode_pos_offset",
      decode_pos_offset(struct.pack("<Bh", 3, -42)), (3, -42))
check("decode_pos_offset 越界丢弃",
      decode_pos_offset(struct.pack("<Bh", 3, 30000)), None)
check("decode_real_tcp_pose pitch=12.5",
      decode_real_tcp_pose(struct.pack("<hhhhhhh", 200, 0, 200, 0, 125, 0, -600))["pitch"], 12.5)

# 7) 安全闸门：dry_run 下不发帧，但越界必须报错
arm = ArmController(dry_run=True)
expect_raises("未开闸门的 move_to", SafetyError,
              lambda: arm.move_to(200, 0, 200))
arm.enable_motion(True)
expect_raises("x=900 越界", SafetyError, lambda: arm.move_to(900, 0, 200))
expect_raises("z=0 越界", SafetyError, lambda: arm.move_to(200, 0, 0))
expect_raises("单次增量过大", SafetyError, lambda: arm.jog(dz=300))
arm.move_to(250, 0, 150, 0, 0, -60, 800, wait=False)
arm.jog(dz=20, time_ms=400)
check("dry_run 记录的动作条数", len(arm.plan), 2)
print("\n--- dry_run 计划 ---")
print(arm.plan)

print("\n%s（%d 项）" % ("全部通过" if not FAILS else "存在失败: " + ", ".join(FAILS),
                      len(FAILS)))
sys.exit(1 if FAILS else 0)
