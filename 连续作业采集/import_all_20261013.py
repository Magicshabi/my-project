"""One-off: import the 12 routes recorded on 2026-10-13 (color front half + area back half).

User confirmed (2026-10-13): all seams test-played on the host with no basket contact;
黄色抬起.d6a frame 4 is a mis-recording and is dropped; 红→P1 is re-imported from the
current files (previous import backed up under backups/before_import_*).
"""
import json
import sys

import collection as c
import import_d6a as imp

OBSERVED = '2026-10-13 用户确认：上位机逐段试运行，各段衔接合理，未碰其他篮体'
FRONT = {'红': ['红篮采集/红到P1/红篮前四.d6a'], '黄': ['黄篮采集/黄色抬起.d6a'],
         '蓝': ['蓝篮采集/蓝抓起.d6a'], '绿': ['绿篮采集/绿抬起.d6a']}
BACK = {('红', 'P1'): '红篮采集/红到P1/红到P1后四.d6a', ('红', 'P2'): '红篮采集/红到P2/红到P2后四.d6a',
        ('红', 'P3'): '红篮采集/红到P3/红到P3后半.d6a', ('黄', 'P1'): '黄篮采集/黄到P1/黄到P1.d6a',
        ('黄', 'P2'): '黄篮采集/黄到P2/黄到P2后.d6a', ('黄', 'P3'): '黄篮采集/黄到P3/黄到P3后.d6a',
        ('蓝', 'P1'): '蓝篮采集/蓝到P1/蓝P1后.d6a', ('蓝', 'P2'): '蓝篮采集/蓝到P2/蓝P2后.d6a',
        ('蓝', 'P3'): '蓝篮采集/蓝到P3/蓝P3后.d6a', ('绿', 'P1'): '绿篮采集/绿到P1/绿P1后.d6a',
        ('绿', 'P2'): '绿篮采集/绿到P2/绿P2后.d6a', ('绿', 'P3'): '绿篮采集/绿到P3/绿P3后.d6a'}
DROP = ['黄色抬起.d6a#4']
S7_WAIVER = {('红', 'P2'), ('红', 'P3')}  # 2026-10-13 用户确认：S7松爪同时含臂体微调，有意为之
WAIVER_TEXT = '2026-10-13 用户确认：松爪同时臂体微调为有意录制，按普通运动点执行'


def main(dry_run):
    # 红→P1 is re-imported from the current files: reset to an empty template first.
    if not dry_run:
        path = c.route_path(c.ROOT, '红', 'P1')
        doc = c.read(path)
        if doc['points']:
            imp.save(path, c.template(c.ROOT, '红', 'P1'), path.read_bytes())
    failed = 0
    for (color, target), back in BACK.items():
        args = [str(c.ROOT / f) for f in FRONT[color] + [back]]
        args += ['--color', color, '--target', target, '--observed', OBSERVED]
        args += [x for d in DROP for x in ('--drop', d)] if color == '黄' else []
        args += ['--s7-body-motion', WAIVER_TEXT] if (color, target) in S7_WAIVER else []
        print(f'== {color}→{target}')
        failed += imp.main(args + (['--dry-run'] if dry_run else [])) != 0
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main('--dry-run' in sys.argv))
