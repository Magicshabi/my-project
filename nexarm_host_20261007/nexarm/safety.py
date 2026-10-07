# -*- coding: utf-8 -*-
"""安全层：任何会让机械臂动起来的 API 都必须先过这一关。

我们在 P0 里踩过的教训决定了这里的三条硬规则：
  1) 默认锁死 —— 库加载后 motion 是关的，必须显式 arm.enable_motion()。
  2) 目标点先过工作空间盒（默认值取自固件 Robot_Arm.h 的 limit_* 默认值），
     越界不截断而是报错，避免"看起来动了但位置是错的"。
  3) 单位与量纲在库边界统一：mm / 度；到协议层才做 pitch×10。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple


class SafetyError(RuntimeError):
    pass


@dataclass
class WorkspaceLimits:
    """固件侧默认工作空间（mm / 度）。改动前先用 get_coord_limits() 读真机值。"""

    x: Tuple[float, float] = (100.0, 400.0)
    y: Tuple[float, float] = (-450.0, 450.0)
    z: Tuple[float, float] = (20.0, 500.0)
    pitch: Tuple[float, float] = (-90.0, 90.0)
    roll: Tuple[float, float] = (-180.0, 180.0)
    claw: Tuple[float, float] = (-90.0, 90.0)

    def check(self, x: float, y: float, z: float, pitch: float, roll: float,
              claw: float) -> None:
        for name, value, (lo, hi) in (
            ("x", x, self.x), ("y", y, self.y), ("z", z, self.z),
            ("pitch", pitch, self.pitch), ("roll", roll, self.roll),
            ("claw", claw, self.claw),
        ):
            if not lo <= value <= hi:
                raise SafetyError(
                    "%s=%.1f 越界 [%.1f, %.1f]；确认标定后用 set_limits() 放开"
                    % (name, value, lo, hi))

    def from_payload(self, values) -> None:
        if values is None or len(values) < 12:
            return
        self.x = (values[0], values[1])
        self.y = (values[2], values[3])
        self.z = (values[4], values[5])
        self.pitch = (values[6], values[7])
        self.roll = (values[8], values[9])
        self.claw = (values[10], values[11])


@dataclass
class MotionGate:
    """动作闸门 + 速率限制。"""

    enabled: bool = False
    max_step_mm: float = 80.0          # 单条增量指令的最大平移，防手滑
    min_period_s: float = 0.05         # 两条运动指令之间的最小间隔
    _last_send: float = field(default=0.0, repr=False)

    def require(self) -> None:
        if not self.enabled:
            raise SafetyError("运动闸门未开：先调用 arm.enable_motion(True)，"
                              "或在调试时用 dry_run=True 只做越界检查")

    def pace(self) -> None:
        wait = self.min_period_s - (time.monotonic() - self._last_send)
        if wait > 0:
            time.sleep(wait)
        self._last_send = time.monotonic()

    def check_step(self, dx: float, dy: float, dz: float) -> None:
        span = max(abs(dx), abs(dy), abs(dz))
        if span > self.max_step_mm:
            raise SafetyError("单次增量 %.1fmm 超过 max_step_mm=%.1f，"
                              "请拆成多步" % (span, self.max_step_mm))


@dataclass
class DryRunPlan:
    """dry_run 模式下的调用记录，便于离线核对动作序列。"""

    actions: list = field(default_factory=list)

    def record(self, name: str, **kwargs) -> None:
        self.actions.append({"action": name, **kwargs})

    def __len__(self) -> int:
        return len(self.actions)

    def __str__(self) -> str:
        return "\n".join("%3d. %s" % (i + 1, a) for i, a in enumerate(self.actions))


def parse_port_hint(port: Optional[str]) -> Optional[str]:
    return port if port else None
