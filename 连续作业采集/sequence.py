"""Compile a colour command such as "红-蓝-黄" into one continuous joint sequence.

Offline only: never opens serial ports. The same compiler feeds both outputs:
  - a combined .d6a action group for host-software playback;
  - the step list used by executor.py over COM11.

Rules (user-confirmed 2026-10-12/13):
  - exactly three distinct colours, mapped in order to P1, P2, P3;
  - start at P0, then route1 S1..S8 -> route2 S1..S8 -> route3 S1..S8, with no return to P0
    between baskets; finish holding at the third route's S8;
  - ID6 goals come from the route files (S3..S6 already squeezed +200; never added twice);
  - S3/S7 are gripper events unless a route records a user-confirmed S7 body-motion waiver.

  python sequence.py 红-蓝-黄                   # print the compiled plan
  python sequence.py 红-蓝-黄 --d6a out.d6a     # write a host action group
"""
import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import collection as c

COLOR_ALIASES = {'红': '红', '黄': '黄', '蓝': '蓝', '绿': '绿',
                 'red': '红', 'yellow': '黄', 'blue': '蓝', 'green': '绿'}
SEPARATORS = r'[\s\-－—–_,，、;；/>→]+'


def parse_command(text):
    """'红-蓝-黄' / '红 蓝 黄' / '红蓝黄' / 'red,blue,yellow' -> ['红', '蓝', '黄']."""
    c.require(isinstance(text, str) and text.strip(), '指令为空')
    parts = [p for p in re.split(SEPARATORS, text.strip().lower()) if p]
    if len(parts) == 1 and all(ch in '红黄蓝绿' for ch in parts[0]):
        parts = list(parts[0])              # "红蓝黄" with no separators
    colors = []
    for p in parts:
        p = p.removesuffix('色').removesuffix('篮')
        c.require(p in COLOR_ALIASES, f'无法识别的颜色：{p}')
        colors.append(COLOR_ALIASES[p])
    c.require(len(colors) == 3, f'需要恰好三种颜色，实际{len(colors)}个')
    c.require(len(set(colors)) == 3, '三种颜色不能重复')
    return colors


def load_routes(root=c.ROOT):
    """Read and validate P0 + all 12 routes once; returns (home_doc, {(color, target): doc}, pins)."""
    pins = {}
    home_path = c.route_path(root)
    home = c.read(home_path)
    c.validate(home, root, complete=True)
    pins[str(home_path)] = c.sha(home_path)
    routes = {}
    for path, color, target in c.paths(root):
        if color is None:
            continue
        doc = c.read(path)
        c.validate(doc, root, color, target, complete=True)
        routes[(color, target)] = doc
        pins[str(path)] = c.sha(path)
    for extra in (root / 'scene.json', c.LIMITS_FILE):
        pins[str(extra)] = c.sha(extra)
    return home, routes, pins


def goal_of(row):
    """Joint goal for a point: taught ID1..5 plus the recorded ID6 goal (S3..S6 squeezed)."""
    return row['encoder_positions'][:5] + [row['gripper_target_counts']]


def compile_plan(colors, home, routes):
    """Ordered steps. kind: move | close | open; axes None = all joints; carry = basket held."""
    steps = [dict(name='P0', route=None, point=0, kind='move', axes=None, carry=False,
                  target=home['points'][0]['encoder_positions'][:])]
    for color, target in zip(colors, c.TARGETS):
        rows = routes[(color, target)]['points']
        for row in rows:
            i = row['point']
            kind = 'close' if i == 3 else 'open' if i == 7 else 'move'
            waived = i == 7 and bool(row.get('body_motion_waiver'))
            if i == 3 or (i == 7 and not waived):
                axes = (6,)
            elif 4 <= i <= 6:
                axes = (1, 2, 3, 4, 5)      # ID6 keeps the squeezed S3 goal while carrying
            else:
                axes = None
            steps.append(dict(name=f'{color}→{target} S{i}', route=(color, target), point=i,
                              kind=kind, axes=axes, carry=4 <= i <= 6, target=goal_of(row)))
    return steps


def to_d6a(steps, time_ms=1000, acc=100):
    return [dict(time=time_ms, servos=[dict(pos=v, acc=acc) for v in s['target']]) for s in steps]


def write_atomic(path, payload):
    path = Path(path)
    temp = path.with_name(path.name + '.' + str(time.time_ns()) + '.tmp')
    with temp.open('xb') as f:
        f.write(payload)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', help='颜色顺序，例如 红-蓝-黄')
    ap.add_argument('--root', type=Path, default=c.ROOT)
    ap.add_argument('--d6a', type=Path, help='输出组合动作组文件')
    ap.add_argument('--time', type=int, default=1000, help='d6a每帧time（与录制一致，默认1000）')
    ap.add_argument('--acc', type=int, default=100)
    a = ap.parse_args(argv)
    try:
        c.require(100 <= a.time <= 10000 and 1 <= a.acc <= 254, 'time或acc超出范围')
        colors = parse_command(a.command)
        home, routes, _ = load_routes(a.root)
        steps = compile_plan(colors, home, routes)
        print('顺序：' + '，'.join(f'{col}→{t}' for col, t in zip(colors, c.TARGETS)))
        for s in steps:
            axes = '全部' if s['axes'] is None else ','.join(f'ID{i}' for i in s['axes'])
            print(f"{s['name']:<10} {s['kind']:<5} 轴={axes:<20} 目标={s['target']}")
        if a.d6a:
            frames = to_d6a(steps, a.time, a.acc)
            write_atomic(a.d6a, json.dumps(frames, ensure_ascii=False).encode('utf-8'))
            print(f'D6A={a.d6a}（{len(frames)}帧；起点P0，停在第三条S8）')
        return 0
    except (ValueError, OSError, KeyError) as exc:
        print('ERROR=' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
