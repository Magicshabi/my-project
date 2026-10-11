"""Offline acceptance of powered teaching evidence; never opens a device."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from joint_limits import (DEFAULT_LIMITS, STAGES, SafetyError, check_binding,
                          check_counts, load_limits)

HERE = Path(__file__).resolve().parent
SOURCE = HERE / 'data/eight_joints_powered_20261009.json'
EXPORTS = HERE.parent / 'handoff/points_powered'


def review(route, exports, limits):
    if (route.get('schema') != 'nexarm.joint-eight/1'
            or route.get('units') != 'raw_encoder_counts'
            or route.get('servo_ids') != list(range(1, 7))
            or route.get('complete') is not True
            or route.get('require_powered_sliders') is not True):
        raise SafetyError('需要完整的带电主动示教数据')
    rows = route.get('points')
    if not isinstance(rows, list) or len(rows) != 8 or len(exports) != 8:
        raise SafetyError('需要八点及八份共享导出')
    check_binding(route, limits)
    points, segments = [], []
    for i, (row, export) in enumerate(zip(rows, exports), 1):
        if (row.get('point') != i or row.get('stage') != STAGES[i-1]
                or row.get('physical_pose_confirmed') is not True
                or row.get('motor_driven_pose_confirmed_by_operator') is not True):
            raise SafetyError('第%d点缺少次序或主动到达确认' % i)
        target = check_counts(row.get('encoder_positions'), limits, '第%d点' % i)
        if row.get('target_encoder_positions', target) != target:
            raise SafetyError('新路线不接受隐藏目标覆盖')
        for field in ('point', 'stage', 'encoder_positions', 'captured_at',
                      'physical_pose_confirmed', 'motor_driven_pose_confirmed_by_operator'):
            if export.get(field) != row.get(field):
                raise SafetyError('第%d点共享导出不一致：%s' % (i, field))
        samples = row.get('samples')
        if not isinstance(samples, list) or len(samples) != 8:
            raise SafetyError('第%d点需要8组原始样本' % i)
        for sample in samples:
            a = check_counts(sample.get('cmd11_counts'), limits, 'CMD11样本')
            b = check_counts(sample.get('cmd65_counts'), limits, 'CMD65样本')
            if max(abs(x-y) for x,y in zip(a,b)) > 6:
                raise SafetyError('第%d点两通道反馈不一致' % i)
        drift = {key: [max(s[key][j] for s in samples)-min(s[key][j] for s in samples)
                       for j in range(6)] for key in ('cmd11_counts','cmd65_counts')}
        if any(v > 5 for values in drift.values() for v in values):
            raise SafetyError('第%d点原始反馈不稳定' % i)
        if samples[-1]['cmd65_counts'] != target:
            raise SafetyError('第%d点目标与保存时最终反馈不一致' % i)
        points.append(dict(point=i, stage=row['stage'], counts=target,
                           max_sample_spread=max(max(v) for v in drift.values())))
        if i == 1:
            continue
        previous = rows[i-2]
        delta = [b-a for a,b in zip(previous['encoder_positions'],target)]
        event = 'grip' if i == 3 else 'release' if i == 7 else 'transport'
        if event != 'transport' and max(map(abs, delta[:5])) > 12:
            raise SafetyError('夹爪事件的臂体变化超出采集容差')
        # Controller XYZ is descriptive evidence, not calibrated geometry/FK.
        pose_a, pose_b = previous['samples'][-1].get('controller_pose'), samples[-1].get('controller_pose')
        pose_delta = None
        if isinstance(pose_a,dict) and isinstance(pose_b,dict):
            pose_delta = {}
            for k in ('x','y','z','pitch','roll'):
                a,b = pose_a.get(k),pose_b.get(k)
                if (isinstance(a,(int,float)) and isinstance(b,(int,float))
                        and math.isfinite(a) and math.isfinite(b)):
                    pose_delta[k] = round(b-a,3)
        reasons = []
        if max(map(abs,delta[:5])) > 350: reasons.append('large_body_joint_change')
        if event != 'transport' and any(delta[:5]): reasons.append('body_drift_at_gripper_event')
        if event == 'transport' and delta[5]: reasons.append('claw_changes_during_transport')
        segments.append(dict(from_point=i-1,to_point=i,event=event,delta_counts=delta,
                             controller_reported_pose_delta=pose_delta,
                             review_flags=reasons,legacy_40_count_steps=max(1,math.ceil(max(map(abs,delta))/40)),
                             path_verified=False,physical_trial_passed=False))
    return dict(schema='nexarm.powered-review/1',data_integrity_passed=True,
                all_points_within_host_limits=True,hardware_test_performed=False,
                motion_authorized=False,collision_checked=False,continuous_replay_verified=False,
                note='Endpoint evidence only; matching controller channels are not independent physical measurement.',
                points=points,segments=segments)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--file',type=Path,default=SOURCE)
    ap.add_argument('--exports',type=Path,default=EXPORTS)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    raw=args.file.read_bytes()
    exports=[args.exports / ('point_%02d.json'%i) for i in range(1,9)]
    report=review(json.loads(raw.decode('utf-8')),
                  [json.loads(p.read_text(encoding='utf-8')) for p in exports],load_limits())
    report.update(source_file=str(args.file.resolve()),source_sha256=hashlib.sha256(raw).hexdigest(),
                  limits_sha256=hashlib.sha256(DEFAULT_LIMITS.read_bytes()).hexdigest(),
                  export_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in exports})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(data_integrity_passed=True,points=8,max_sample_spread=max(p['max_sample_spread'] for p in report['points']),
                         output=str(args.output),motion_authorized=False),ensure_ascii=False))


if __name__=='__main__':
    main()
