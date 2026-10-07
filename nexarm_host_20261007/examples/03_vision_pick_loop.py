# -*- coding: utf-8 -*-
"""示例 3：视觉抓取骨架（标定接口留空，未标定直接报错，不会瞎动）。

    python examples/03_vision_pick_loop.py COM9 --dry-run

这段代码的价值在于**结构**：把「像素 -> 机械臂坐标」的换算隔离成一个可替换对象，
识别源（K230 颜色分拣 / AprilTag / 自研视觉）只提供 (u, v) 像素中心，
抓取流程本身不再关心视觉细节。标定好 4 个点后填进 Homography 即可闭环。
"""
import os
import sys
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexarm import ArmController  # noqa: E402

# 与固件一致的抓取高度（mm）：先抬到 APPROACH_Z，再下到 GRAB_Z，抓完抬回
APPROACH_Z = 140.0
GRAB_Z = 60.0
LIFT_Z = 160.0
CLAW_OPEN = -60.0
CLAW_CLOSE = 0.0


@dataclass
class GrabTarget:
    """一次抓取任务：像素中心 + 可选的颜色/类别标签。"""

    u: float
    v: float
    label: str = ""


def solve_homography(pairs: Sequence[Tuple[Tuple[float, float],
                                          Tuple[float, float]]]) -> list:
    """由 4 组 ((u,v) -> (x,y)) 对应点解 3x3 单应矩阵（8 未知数，高斯消元）。

    返回行优先 9 元素列表 H，满足 x = (H0u+H1v+H2)/(H6u+H7v+1)。
    """
    if len(pairs) != 4:
        raise ValueError("需要正好 4 组对应点")
    rows, rhs = [], []
    for (u, v), (x, y) in pairs:
        rows.append([u, v, 1, 0, 0, 0, -u * x, -v * x])
        rhs.append(x)
        rows.append([0, 0, 0, u, v, 1, -u * y, -v * y])
        rhs.append(y)
    n = 8
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-12:
            raise ValueError("退化标定点：四点共线，重新取点")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        rhs[col], rhs[pivot] = rhs[pivot], rhs[col]
        pv = rows[col][col]
        for r in range(n):
            if r == col:
                continue
            factor = rows[r][col] / pv
            for c in range(col, n):
                rows[r][c] -= factor * rows[col][c]
            rhs[r] -= factor * rhs[col]
    h = [rhs[i] / rows[i][i] for i in range(n)]
    return h + [1.0]


class Homography:
    """像素 -> 机械臂 XY 的仿射映射。未标定时 to_arm 抛错。"""

    def __init__(self, matrix: Optional[list] = None) -> None:
        self.h = matrix

    def calibrate(self, pairs) -> None:
        self.h = solve_homography(pairs)

    @property
    def calibrated(self) -> bool:
        return self.h is not None

    def to_arm(self, u: float, v: float) -> Tuple[float, float]:
        if not self.calibrated:
            raise RuntimeError("未标定：请先用 4 组 (像素, 机械臂坐标) 调用 calibrate()")
        h = self.h
        den = h[6] * u + h[7] * v + 1.0
        return ((h[0] * u + h[1] * v + h[2]) / den,
                (h[3] * u + h[4] * v + h[5]) / den)


def pick_once(arm: ArmController, mapping: Homography, target: GrabTarget,
              pitch: float = 0.0, roll: float = 0.0,
              time_ms: int = 1200) -> None:
    """单次抓取：抬升 -> 平移到位 -> 下沉 -> 夹紧 -> 抬升。"""
    x, y = mapping.to_arm(target.u, target.v)
    arm.log("目标 %s 像素(%.0f,%.0f) -> 机械臂(%.1f,%.1f)"
            % (target.label or "?", target.u, target.v, x, y))

    arm.set_claw(CLAW_OPEN, time_ms=400)
    arm.move_to(x, y, APPROACH_Z, pitch, roll, CLAW_OPEN, time_ms)
    arm.move_to(x, y, GRAB_Z, pitch, roll, CLAW_OPEN, time_ms=700)
    arm.set_claw(CLAW_CLOSE, time_ms=400)
    arm.move_to(x, y, LIFT_Z, pitch, roll, CLAW_CLOSE, time_ms)


def main(argv) -> int:
    port = argv[1] if len(argv) > 1 else "COM9"
    dry = "--dry-run" in argv

    mapping = Homography()          # 这里放标定结果；现在故意留空 -> 会报错保护
    arm = ArmController(dry_run=dry)
    if not dry:
        arm.connect_serial(port)
    arm.enable_motion(True)

    # 识别源由外部注入：这里只用两个假目标演示调用方式
    detect: Callable[[], Optional[GrabTarget]] = lambda: GrabTarget(320.0, 240.0, "red")

    target = detect()
    if target is None:
        print("视野里没有目标颜色")
        return 0
    try:
        pick_once(arm, mapping, target)
    except RuntimeError as exc:
        print("已按设计拒绝执行:", exc)
        print("下一步：跑标定，采集 4 组 (像素, 机械臂 XY) 后 mapping.calibrate(pairs)")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
