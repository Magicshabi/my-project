"""Offline import of host-software action groups (.d6a) into canonical route files.

Never opens serial ports. Points come from the operator's taught/test-played
host targets, not from serial sampling; measured feedback is left to the
executor's per-step arrival checks.

  python import_d6a.py 红到P1.d6a --color 红 --target P1 --observed '上位机试运行无碰撞'
  python import_d6a.py 前半.d6a 后半.d6a --color 红 --target P1 --observed '...'   # 分批录入
  python import_d6a.py P0.d6a --home --observed '...'
  python import_d6a.py 红到P1.d6a --color 红 --target P1 --dry-run              # 只检查不写

ID6 rule: S3 goal = taught S3 ID6 + 200 (capped at the software limit), S4..S6 inherit
S3, other points keep their taught ID6. Only empty route files are written.
"""
import argparse
import copy
import json
import os
import sys
import time
from pathlib import Path

import collection as c


def read_frames(paths, drop=()):
    """drop: 'file.d6a#N' entries naming mis-recorded frames to leave out (provenance keeps original N)."""
    drop = set(drop)
    frames = []
    for path in paths:
        path = Path(path)
        raw = path.read_bytes()
        try:
            doc = json.loads(raw.decode('utf-8-sig'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f'{path.name} 不是有效JSON：{exc}')
        c.require(isinstance(doc, list) and doc, f'{path.name} 必须是非空帧数组')
        for i, frame in enumerate(doc):
            where = f'{path.name} 第{i+1}帧'
            c.require(isinstance(frame, dict) and isinstance(frame.get('servos'), list)
                      and len(frame['servos']) == 6, where + '需要6个servos')
            c.require(all(isinstance(s, dict) and type(s.get('pos')) is int for s in frame['servos']),
                      where + ' pos必须是整数')
            pos = [s['pos'] for s in frame['servos']]
            try:
                c.counts(pos)
            except ValueError:
                raise ValueError(f'{where} 刻度超出软件范围：{pos}')
            if f'{path.name}#{i + 1}' in drop:
                drop.discard(f'{path.name}#{i + 1}')
                continue
            frames.append(dict(pos=pos, file=path.name, frame=i + 1, sha256=c.sha(path)))
    c.require(not drop, f'--drop 未匹配任何帧：{sorted(drop)}')
    return frames


def build(doc, root, color, target, frames, observed, s7_waiver=''):
    expected = 1 if color is None else 8
    c.validate(doc, root, color, target)
    c.require(not doc['points'], '目标主文件已有点位，拒绝导入；重采请用新采集目录')
    c.require(len(frames) == expected, f'需要{expected}帧，实际{len(frames)}帧')
    c.require(isinstance(observed, str) and observed.strip(), '需要上位机试运行/摆位依据')
    hi = c.limits()[5][1]
    out = copy.deepcopy(doc)
    close = None
    for i, f in enumerate(frames, 1):
        pos = f['pos']
        if color is None or i in (1, 2, 7, 8):
            goal = pos[5]
        elif i == 3:
            close = goal = min(pos[5] + c.GRIP_SQUEEZE, hi)
        else:
            goal = close
        out['points'].append(dict(
            point=i, stage='home' if color is None else c.STAGES[i - 1],
            encoder_positions=pos, gripper_target_counts=goal,
            keep_gripper_goal=color is not None and 4 <= i <= 6,
            source=c.D6A, d6a=dict(file=f['file'], frame=f['frame'], sha256=f['sha256']),
            samples=None, captured_at=c.now(), evidence_dir='d6a:' + f['file'],
            observation=observed.strip(), physical_pose_confirmed=True,
            motor_driven_pose_confirmed_by_operator=True))
        if i == 7 and color is not None and s7_waiver.strip():
            out['points'][-1]['body_motion_waiver'] = s7_waiver.strip()
    out['complete'] = True
    c.validate(out, root, color, target, complete=True)
    return out


def save(path, doc, original):
    c.require(path.read_bytes() == original, '主文件已被其他进程修改；拒绝覆盖')
    stamp = str(time.time_ns())
    with path.with_name(path.name + '.' + stamp + '.bak').open('xb') as f:
        f.write(original)
    temp = path.with_name(path.name + '.' + stamp + '.tmp')
    payload = (json.dumps(doc, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    with temp.open('xb') as f:
        f.write(payload)
        f.flush()
        os.fsync(f.fileno())
    c.require(path.read_bytes() == original, '保存前主文件发生变化；原文件保留')
    os.replace(temp, path)
    c.require(path.read_bytes() == payload, '主文件读回不一致')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+', type=Path, help='按顺序的d6a文件（分批录入可传多个）')
    ap.add_argument('--root', type=Path, default=c.ROOT)
    ap.add_argument('--home', action='store_true')
    ap.add_argument('--color', choices=c.COLORS)
    ap.add_argument('--target', choices=c.TARGETS)
    ap.add_argument('--observed', default='', help='上位机试运行/摆位依据（写入时必填）')
    ap.add_argument('--dry-run', action='store_true', help='只解析校验并打印，不写文件')
    ap.add_argument('--drop', action='append', default=[], metavar='文件名#帧号',
                    help='跳过误录帧，例如 黄色抬起.d6a#4；可重复')
    ap.add_argument('--s7-body-motion', default='', metavar='依据',
                    help='用户确认S7松爪同时含臂体动作时填写依据；仅放宽S7的12刻度检查并记入数据')
    a = ap.parse_args(argv)
    try:
        c.require((a.home and a.color is None and a.target is None)
                  or (not a.home and a.color in c.COLORS and a.target in c.TARGETS), '请选择--home或颜色+目标')
        frames = read_frames(a.files, a.drop)
        path = c.route_path(a.root, a.color, a.target)
        original = path.read_bytes()
        doc = build(c.read(path), a.root, a.color, a.target, frames,
                    a.observed or ('dry-run' if a.dry_run else ''), a.s7_body_motion)
        name = 'P0' if a.home else f'{a.color}→{a.target}'
        for row in doc['points']:
            p = row['encoder_positions']
            print(f"{name} S{row['point']} {row['stage']}: ID1~6={p} ID6目标={row['gripper_target_counts']}"
                  f" ← {row['d6a']['file']}#{row['d6a']['frame']}")
        if a.dry_run:
            print('DRY_RUN=未写入')
            return 0
        save(path, doc, original)
        print(f'已导入 {name}：{len(doc["points"])}点，sha256={c.sha(path)}；来源d6a，未经串口实测，不代表运动验收')
        return 0
    except (ValueError, OSError) as exc:
        print('ERROR=' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
