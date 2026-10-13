#!/usr/bin/env python3
"""将route.json转换成d6a文件，用于上位机验证。"""
import argparse
import json
import sys
from pathlib import Path
import collection as c


def route_to_d6a(root, color, target, output_path, time_ms, acc):
    """读取route.json，生成d6a文件。

    Args:
        root: 采集目录
        color: 篮子颜色（None表示P0）
        target: 目标区域（None表示P0）
        output_path: 输出d6a文件路径
        time_ms: 每段运动时间（毫秒）
        acc: 加速度参数（0-255）
    """
    doc = c.read(c.route_path(root, color, target))
    c.validate(doc, root, color, target)
    points = doc['points']
    name = 'P0' if color is None else f'{color}→{target}'

    if not points:
        raise ValueError(f'{name} 无有效点')

    if len(points) != 8 and color is not None:
        raise ValueError(f'{name} 应有8个点，实际：{len(points)}')

    if len(points) != 1 and color is None:
        raise ValueError(f'P0 应有1个点，实际：{len(points)}')

    # 转换成d6a格式
    d6a_frames = []
    for pt in points:
        encoder_positions = pt.get('encoder_positions')
        if not encoder_positions or len(encoder_positions) != 6:
            raise ValueError(f'点{pt.get("point")}缺少有效的encoder_positions')

        frame = {
            'time': time_ms,
            'servos': [
                {'pos': encoder_positions[i], 'acc': acc}
                for i in range(6)
            ]
        }
        d6a_frames.append(frame)

    # 保存
    output_path = Path(output_path)
    temp_path = output_path.with_suffix('.d6a.tmp')

    try:
        with open(temp_path, 'w', encoding='utf-8') as f:
            json.dump(d6a_frames, f, ensure_ascii=False, indent=2)
        temp_path.replace(output_path)
        print(f'[OK] 已保存到 {output_path}')
        print(f'     {len(d6a_frames)} 帧，time={time_ms}ms, acc={acc}')
        return 0
    except Exception as e:
        if temp_path.exists():
            temp_path.unlink()
        raise RuntimeError(f'保存失败：{e}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=c.ROOT, help='采集目录')
    ap.add_argument('--home', action='store_true', help='转换P0')
    ap.add_argument('--color', choices=c.COLORS, help='篮子颜色')
    ap.add_argument('--target', choices=c.TARGETS, help='目标区域')
    ap.add_argument('--output', required=True, help='输出d6a文件路径')
    ap.add_argument('--time', type=int, default=1000, help='运动时间（毫秒），默认1000')
    ap.add_argument('--acc', type=int, default=100, help='加速度（0-255），默认100')

    a = ap.parse_args(argv)

    try:
        c.require((a.home and a.color is None and a.target is None)
                  or (not a.home and a.color in c.COLORS and a.target in c.TARGETS),
                  '请选择--home或--color+--target')

        return route_to_d6a(a.root, a.color, a.target, a.output, a.time, a.acc)
    except (ValueError, OSError, RuntimeError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
