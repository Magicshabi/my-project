# -*- coding: utf-8 -*-
"""ArmController —— 面向"能直接用"的运动控制门面。

调用范式（同步、可直接写在脚本里）：

    arm = ArmController(dry_run=True)        # 先干跑，核对序列
    arm.connect_serial("COM9")
    arm.enable_motion(True)                  # 不显式打开，任何运动调用都会报错
    arm.home()
    arm.move_to(x=200, y=0, z=200, pitch=0, roll=0, claw=-60, time_ms=1500)
    arm.jog(dz=+20, time_ms=600)
    arm.move_path([(250, 0, 120), (250, 60, 120)], claw=-60, time_ms=1200)
    print(arm.get_pose())

所有长度单位 mm、角度单位度。pitch 的 ×10 只在本库调用 commands.pack_* 时发生。
"""
from __future__ import annotations

import time
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from . import commands as C
from .safety import DryRunPlan, MotionGate, SafetyError, WorkspaceLimits
from .transport import DEFAULT_BAUDRATE, Transport

#: GUI「全部回中」下发的位姿（反编译 ui.coord_tab.CoordTab.home_all）
HOME_POSE = dict(x=200.0, y=0.0, z=200.0, pitch=0.0, roll=0.0, claw=0.0)


def _default_log(msg: str) -> None:
    print("[nexarm] %s" % msg)


class ArmController:
    def __init__(self, transport: Optional[Transport] = None,
                 log: Optional[Callable[[str], None]] = None,
                 dry_run: bool = False) -> None:
        self.log = log or _default_log
        self.transport = transport or Transport(log=self.log)
        self.limits = WorkspaceLimits()
        self.gate = MotionGate()
        self.dry_run = dry_run
        self.plan = DryRunPlan()
        self.last_pose: Optional[C.Pose] = None
        self._pose_subs: List[Callable[[C.Pose], None]] = []
        self.transport.on(int(C.Cmd.GET_CUR_COORDS), self._on_pose)

    # ------------------------------------------------------------- lifecycle
    def connect_serial(self, port: str, baudrate: int = DEFAULT_BAUDRATE,
                       reset_acc: bool = False) -> None:
        """连接串口。默认**不发任何指令**，保持只读语义。

        NexArm.exe 连上后会自动把起步加速度写为 0；需要同样行为时传
        reset_acc=True。要让臂真的动，还要显式 set_torque(True) + enable_motion(True)。
        """
        self.transport.open_serial(port, baudrate)
        if reset_acc:
            self.set_acc(0)

    def connect_socket(self, host: str, port: int = 8080) -> None:
        self.transport.open_socket(host, port)

    def close(self) -> None:
        self.transport.close()

    @property
    def connected(self) -> bool:
        return self.transport.connected

    def enable_motion(self, enabled: bool = True) -> None:
        """运动闸门。关着的时候 move_* 一律 SafetyError。"""
        self.gate.enabled = bool(enabled)
        self.log("运动闸门 %s" % ("已开启" if enabled else "已关闭"))

    def on_pose(self, callback: Callable[[C.Pose], None]) -> None:
        """订阅实时位姿（每当 cmd 11 回包时触发）。"""
        self._pose_subs.append(callback)

    def _on_pose(self, frame) -> None:
        pose = C.decode_current_coords(frame.args)
        if pose is None:
            return
        self.last_pose = pose
        for cb in list(self._pose_subs):
            try:
                cb(pose)
            except Exception as exc:
                self.log("位姿回调异常: %s" % exc)

    # ----------------------------------------------------------------- queries
    def get_pose(self, timeout: float = 0.3) -> Optional[C.Pose]:
        frame = self.transport.request(int(C.Cmd.GET_CUR_COORDS), timeout=timeout)
        if frame is None:
            return None
        pose = C.decode_current_coords(frame.args)
        if pose is not None:
            self.last_pose = pose
        return pose

    def get_tcp_pose(self, timeout: float = 0.3):
        frame = self.transport.request(int(C.Cmd.GET_REAL_TCP_POSE), timeout=timeout)
        return None if frame is None else C.decode_real_tcp_pose(frame.args)

    def get_real_joints(self, timeout: float = 0.3):
        frame = self.transport.request(int(C.Cmd.GET_REAL_JOINT_ANGLES), timeout=timeout)
        return None if frame is None else C.decode_real_joint_angles(frame.args)

    def get_battery_mv(self, timeout: float = 0.3):
        frame = self.transport.request(int(C.Cmd.CHECK_BAT_LEVEL_CHECK), timeout=timeout)
        return None if frame is None else C.decode_battery_mv(frame.args)

    def get_version(self, timeout: float = 0.5):
        frame = self.transport.request(int(C.Cmd.FIRMWARE_VERSION_CHECK), timeout=timeout)
        return None if frame is None else C.decode_fw_version(frame.args)

    def get_coord_limits(self, refresh: bool = True, timeout: float = 0.3):
        frame = self.transport.request(int(C.Cmd.GET_COORD_LIMITS), timeout=timeout)
        if frame is None:
            return None
        values = C.decode_coord_limits(frame.args)
        if refresh and values:
            self.limits.from_payload(values)
        return values

    def get_kinematics(self, timeout: float = 0.3):
        frame = self.transport.request(int(C.Cmd.GET_KINEMATICS_PARAM), timeout=timeout)
        return None if frame is None else C.decode_kinematics(frame.args)

    def get_pose_offsets(self, first: int = 1, last: int = 6, gap: float = 0.05):
        """逐舵机读偏差。对应 GUI 连接后的 _read_all_offsets（1~6，间隔 50ms）。"""
        out = {}
        for sid in range(first, last + 1):
            frame = self.transport.request(int(C.Cmd.GET_POS_OFFSET),
                                           bytes([sid]), timeout=0.2)
            if frame is not None:
                parsed = C.decode_pos_offset(frame.args)
                if parsed:
                    out[parsed[0]] = parsed[1]
            time.sleep(gap)
        return out

    # ---------------------------------------------------------------- config
    def set_limits(self, **kwargs) -> None:
        """只改传入的字段，例如 set_limits(x=(120, 380))。"""
        for key, value in kwargs.items():
            if not hasattr(self.limits, key):
                raise ValueError("未知限位字段 %s" % key)
            setattr(self.limits, key, tuple(value))

    def push_limits(self) -> None:
        """把当前 limits 写入固件（cmd 79，12×int16）。"""
        l = self.limits
        args = C.pack_coord_limits(l.x[0], l.x[1], l.y[0], l.y[1], l.z[0], l.z[1],
                                   l.pitch[0], l.pitch[1], l.roll[0], l.roll[1],
                                   l.claw[0], l.claw[1])
        self.transport.send(int(C.Cmd.SET_COORD_LIMITS), args)

    def set_kinematics(self, arm_kin: Sequence[float]) -> None:
        self.transport.send(int(C.Cmd.SET_KINEMATICS_PARAM), C.pack_kinematics(arm_kin))

    def set_acc(self, acc: int) -> None:
        """底层 10ms 插补起步加速度：0 最快/无缓冲，254 最平滑。[SDK]"""
        acc = max(0, min(254, int(acc)))
        if self.dry_run:
            self.plan.record("set_acc", acc=acc)
            return
        self.transport.send(int(C.Cmd.SET_MOVE_ACC), C.pack_u8(acc))

    def set_global_acc(self, acc: int) -> None:
        if self.dry_run:
            self.plan.record("set_global_acc", acc=acc)
            return
        self.transport.send(int(C.Cmd.SET_GLOBAL_ACC), C.pack_u8(int(acc) & 0xFF))

    def set_torque(self, enable: bool) -> None:
        """舵机力矩。掉电示教前必须关，运动前必须开。

        启动力矩行为取决于当前固件；发送成功不证明实际力矩状态。
        """
        if self.dry_run:
            self.plan.record("set_torque", enable=bool(enable))
            return
        self.transport.send(int(C.Cmd.SET_TORQUE), C.pack_u8(1 if enable else 0))

    # ----------------------------------------------------------------- motion
    def home(self, time_ms: int = 1000, wait: bool = True,
             tol: float = 8.0, timeout: float = 8.0) -> None:
        """回到 GUI 同款中位 (200, 0, 200, pitch 0)。"""
        self.move_to(time_ms=time_ms, wait=wait, tol=tol, timeout=timeout, **HOME_POSE)

    def reset_all(self, time_ms: int = 1500, wait: bool = True) -> None:
        """底层全关节复位（cmd 54，uint16 时长）。上电复位流程在用这条。"""
        self.gate.require()
        self.log("reset_all %dms" % time_ms)
        if self.dry_run:
            self.plan.record("reset_all", time_ms=time_ms)
        else:
            self.transport.send(int(C.Cmd.ARM_RESET), C.pack_u16(time_ms))
        if wait:
            time.sleep(max(0.1, time_ms / 1000.0) + 0.3)

    def move_to(self, x: float, y: float, z: float, pitch: float = 0.0,
                roll: float = 0.0, claw: float = 0.0, time_ms: int = 1000,
                wait: bool = True, tol: float = 8.0,
                timeout: float = 8.0) -> None:
        """绝对坐标运动（cmd 8）。IK 由 AT32 完成，本库不做逆解。"""
        self.gate.require()
        self.limits.check(x, y, z, pitch, roll, claw)
        self.gate.pace()
        self.log("move_to (%.1f, %.1f, %.1f) pitch=%.1f roll=%.1f claw=%.1f %dms"
                 % (x, y, z, pitch, roll, claw, time_ms))
        if self.dry_run:
            self.plan.record("move_to", x=x, y=y, z=z, pitch=pitch, roll=roll,
                             claw=claw, time_ms=time_ms)
        else:
            self.transport.send(int(C.Cmd.COORDINATE_SET),
                                C.pack_coord(x, y, z, pitch, roll, claw, time_ms))
        if wait and not self.dry_run:
            if not self.wait_for(x, y, z, tol=tol, timeout=timeout,
                                 pitch=pitch, roll=roll, claw=claw,
                                 min_duration=time_ms / 1000.0):
                raise TimeoutError("控制器报告未稳定到达完整目标位姿；停止后续点位")

    def jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
            dpitch: float = 0.0, droll: float = 0.0, dclaw: float = 0.0,
            time_ms: int = 500, wait: bool = False) -> None:
        """增量运动（cmd 50）。字段顺序与绝对值不同，见 commands.INC_FMT。"""
        self.gate.require()
        self.gate.check_step(dx, dy, dz)
        self.gate.pace()
        self.log("jog (Δ%.1f, Δ%.1f, Δ%.1f) Δpitch=%.1f Δroll=%.1f Δclaw=%.1f %dms"
                 % (dx, dy, dz, dpitch, droll, dclaw, time_ms))
        if self.dry_run:
            self.plan.record("jog", dx=dx, dy=dy, dz=dz, dpitch=dpitch,
                             droll=droll, dclaw=dclaw, time_ms=time_ms)
            return
        self.transport.send(int(C.Cmd.ARM_MOVE_INC),
                            C.pack_move_inc(dx, dy, dz, dpitch, droll, dclaw, time_ms))
        if wait:
            time.sleep(time_ms / 1000.0 + 0.2)

    def set_claw(self, angle: float, time_ms: int = 600, wait: bool = True) -> None:
        """只动夹爪：读当前位姿，保持 XYZ/roll，替换 claw。"""
        pose = self.get_pose()
        if pose is None:
            raise RuntimeError("读不到当前位姿，无法只动夹爪")
        self.move_to(pose.x, pose.y, pose.z, pose.pitch, pose.roll, angle,
                     time_ms, wait=wait)

    def move_servo(self, servo_id: int, angle: float, time_ms: int = 800) -> None:
        """旧CMD51封装接口；angle参数实际承载位置计数，不是角度。

        这不是EXE关节滑块的寄存器写接口；新适配使用move_joint_counts。
        """
        self.gate.require()
        import struct
        if self.dry_run:
            self.plan.record("move_servo", servo_id=servo_id, angle=angle,
                             time_ms=time_ms)
            return
        self.transport.send(int(C.Cmd.ARM_SERVO_SINGLE),
                            struct.pack("<BhH", int(servo_id), int(angle), int(time_ms)))

    def move_joint_counts(self, servo_id: int, position: int,
                          acceleration: int, speed: int) -> None:
        """EXE关节滑块：单ID、CMD3、reg41。调用者须先核对起点、步长及现场。

        不做IK，不发力矩命令；不自动等待、不证明实体到位。
        """
        self.gate.require()
        if type(servo_id) is not int or not 1 <= servo_id <= 6:
            raise ValueError('servo_id must be 1..6; broadcast motion refused')
        payload=C.pack_joint_counts(position, acceleration, speed)
        self.gate.pace()
        if self.dry_run:
            self.plan.record('move_joint_counts', servo_id=servo_id, position=position,
                             acceleration=acceleration, speed=speed)
        else:
            self.transport.send(3, payload, target_id=servo_id)

    def move_path(self, waypoints: Iterable[Tuple[float, float, float]],
                  claw: Optional[float] = None, time_ms: int = 1200,
                  pitch: Optional[float] = None, wait: bool = True) -> None:
        """依次经过一串 (x, y, z) 路点。claw/pitch 不给就跟着当前位姿走。"""
        points = [(float(px), float(py), float(pz)) for px, py, pz in waypoints]
        pose = None if self.dry_run else self.get_pose()
        base_pitch = pose.pitch if (pitch is None and pose) else (pitch or 0.0)
        base_claw = pose.claw if (claw is None and pose) else (claw or 0.0)
        base_roll = pose.roll if pose else 0.0
        for i, (x, y, z) in enumerate(points):
            self.log("路点 %d/%d: (%.1f, %.1f, %.1f)" % (i + 1, len(points), x, y, z))
            self.move_to(x, y, z, base_pitch, base_roll, base_claw, time_ms,
                         wait=wait)

    # ------------------------------------------------------------------- wait
    def wait_for(self, x: float, y: float, z: float, tol: float = 8.0,
                 timeout: float = 8.0, poll: float = 0.1,
                 pitch=None, roll=None, claw=None, angle_tol: float = 3.0,
                 min_duration: float = 0.0) -> bool:
        """等待至少三次完整位姿报告稳定；仅是控制器状态，不证明物理到位。

        不能用CMD65与CMD11一致作为独立物理验证。缺失报告会清空稳定计数。
        """
        started = time.monotonic()
        deadline = started + timeout
        stable = 0
        first_stable = None
        while time.monotonic() < deadline:
            pose = self.get_pose()
            good = pose is not None and all(abs(getattr(pose, k)-v) <= tol
                                           for k, v in (('x', x), ('y', y), ('z', z)))
            if good:
                good = all(v is None or abs(getattr(pose, k)-v) <= angle_tol
                           for k, v in (('pitch', pitch), ('roll', roll), ('claw', claw)))
            now = time.monotonic()
            if good and now - started >= min_duration:
                stable += 1
                if first_stable is None:
                    first_stable = now
                if stable >= 3 and now-first_stable >= 0.2:
                    return True
            else:
                stable = 0
                first_stable = None
            time.sleep(poll)
        self.log("等待到位超时：目标 (%.1f, %.1f, %.1f) tol=%.1f" % (x, y, z, tol))
        return False

    # ------------------------------------------------------------ peripherals
    def action_group_run(self, group_id: int) -> None:
        self.gate.require()
        if self.dry_run:
            self.plan.record("action_group_run", group_id=group_id)
            return
        self.transport.send(int(C.Cmd.ACTION_GROUP_RUN),
                            C.pack_action_group_run(group_id))

    def action_group_stop(self) -> None:
        self.transport.send(int(C.Cmd.ACTION_GROUP_STOP))

    def action_group_erase(self, group_id: int) -> None:
        self.transport.send(int(C.Cmd.ACTION_GROUP_ERASE), bytes([group_id & 0xFF]))

    def buzzer(self, on_ms: int = 100, off_ms: int = 100, repeat: int = 1,
               freq: int = 2000) -> None:
        self.transport.send(int(C.Cmd.BUZZER_SET),
                            C.pack_buzzer(on_ms, off_ms, repeat, freq))

    def oled(self, line: int, text: str) -> None:
        self.transport.send(int(C.Cmd.OLED_SET), C.pack_oled_text(line, text))

    def conveyor(self, speed: int) -> None:
        import struct
        self.transport.send(int(C.Cmd.CONVEYOR_SET), struct.pack("<b", int(speed)))

    def stepper_run(self, steps: int) -> None:
        self.transport.send(int(C.Cmd.STEPPER_RUN), C.pack_stepper_run(steps))

    # ---------------------------------------------- 舵机总线直读（第三通道）
    SERVO_CMD_READ = 2            # 舵机帧的 cmd：2=读寄存器 3=写寄存器
    SERVO_CMD_WRITE = 3
    SERVO_REG_TORQUE = 40
    SERVO_REG_GOAL_POS = 42
    SERVO_REG_PRESENT_POS = 56

    def read_servo_raw(self, servo_id: int, reg: int, length: int = 2,
                       timeout: float = 0.25):
        """直接读舵机寄存器（回包 id = 舵机号），返回原始 args 字节。

        注意：这条通道依赖固件把舵机透传数据回推到 USB。若返回 None，说明当前
        固件不支持该透传，属预期情况，不影响 cmd 11/65 两条通道。
        """
        frame = self.transport.request_from(servo_id, self.SERVO_CMD_READ,
                                            bytes([reg, length]), timeout=timeout)
        return None if frame is None else frame.args

    def read_servo_positions(self, first: int = 1, last: int = 6, gap: float = 0.03):
        """逐舵机直读当前位置（reg 56）。返回 {id: (原始args, 两种可能解析)}。"""
        out = {}
        for sid in range(first, last + 1):
            raw = self.read_servo_raw(sid, self.SERVO_REG_PRESENT_POS, 2)
            if raw is not None:
                import struct
                cand = {}
                if len(raw) >= 2:
                    cand["args0_1"] = struct.unpack("<h", raw[0:2])[0]
                if len(raw) >= 3:
                    cand["args1_2"] = struct.unpack("<h", raw[1:3])[0]
                out[sid] = (raw.hex(), cand)
            time.sleep(gap)
        return out

    def stats(self) -> dict:
        return self.transport.stats()
