"""Host-only raw-count limits and offline audit. No serial port is opened here."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '舵机基础运行逻辑'))
from nexarm.safety import SafetyError
from nexarm.eight_point import STAGES

DEFAULT_LIMITS = HERE / 'data/joint_limits_20261010.json'
SCHEMA = 'nexarm.host-joint-limits/1'


def validate_limits(document):
    if (not isinstance(document, dict) or document.get('schema') != SCHEMA
            or document.get('units') != 'raw_encoder_counts'
            or document.get('servo_ids') != list(range(1, 7))):
        raise SafetyError('软件限位格式、单位或ID顺序不正确')
    bounds = document.get('bounds')
    if not isinstance(bounds, list) or len(bounds) != 6:
        raise SafetyError('软件限位必须覆盖ID1..6')
    for pair in bounds:
        if (not isinstance(pair, list) or len(pair) != 2
                or any(type(v) is not int for v in pair)
                or not 0 <= pair[0] < pair[1] <= 4095):
            raise SafetyError('软件限位必须为协议范围内的整数上下限')
    return tuple(tuple(pair) for pair in bounds)


def load_limits(path=DEFAULT_LIMITS):
    return validate_limits(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def snapshot(limits):
    return dict(schema=SCHEMA, units='raw_encoder_counts', servo_ids=list(range(1, 7)),
                bounds=[list(pair) for pair in limits])


def checked_counts(values):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise SafetyError('必须有六轴位置反馈')
    if any(type(v) is not int or not 0 <= v <= 4095 for v in values):
        raise SafetyError('编码器反馈超出协议范围或不是整数')
    return list(values)


def violations(values, limits):
    counts = checked_counts(values)
    return [dict(servo_id=i+1, value=v, lower=lo, upper=hi)
            for i, (v, (lo, hi)) in enumerate(zip(counts, limits))
            if not lo <= v <= hi]


def check_counts(values, limits, context='目标'):
    bad = violations(values, limits)
    if bad:
        detail = '; '.join('ID{servo_id}={value} 超出 [{lower},{upper}]'.format(**v) for v in bad)
        raise SafetyError(context + ': ' + detail + '；不裁剪、不写入，请重新摆位')
    return list(values)


def check_axis(servo_id, position, limits):
    if type(servo_id) is not int or not 1 <= servo_id <= 6:
        raise SafetyError('运动目标必须为ID1..6')
    lo, hi = limits[servo_id-1]
    if type(position) is not int or not lo <= position <= hi:
        raise SafetyError('ID%d=%s 超出软件范围 [%d,%d]；拒绝发送' % (servo_id, position, lo, hi))


def check_binding(route, limits):
    bound = route.get('host_joint_limits')
    if bound is None:
        if route.get('points'):
            raise SafetyError('已有点位未绑定当前软件限位；先离线复核，不自动迁移')
        return
    if validate_limits(bound) != limits:
        raise SafetyError('本轮软件限位与采集会话不同；拒绝继续使用旧会话')


def audit_route(route, limits):
    if (route.get('schema') != 'nexarm.joint-eight/1'
            or route.get('units') != 'raw_encoder_counts'
            or route.get('servo_ids') != list(range(1, 7))):
        raise SafetyError('点位格式、单位或ID顺序不正确')
    rows = route.get('points')
    if not isinstance(rows, list) or len(rows) > 8:
        raise SafetyError('点位列表无效')
    report = []
    for i, row in enumerate(rows):
        if row.get('point') != i+1 or row.get('stage') != STAGES[i]:
            raise SafetyError('点位次序不正确')
        raw = checked_counts(row.get('encoder_positions'))
        target = checked_counts(row.get('target_encoder_positions', raw))
        report.append(dict(point=i+1, stage=STAGES[i], raw=raw, target=target,
                           violations=violations(target, limits)))
    try:
        check_binding(route, limits)
        binding_issue = None if route.get('host_joint_limits') else '空会话尚未绑定，将在首次保存时绑定'
    except SafetyError as exc:
        binding_issue = str(exc)
    return dict(scope='offline numeric check only; not motion or collision validation',
                host_joint_limits=snapshot(limits), captured_points=len(rows),
                all_targets_within_limits=bool(rows) and all(not p['violations'] for p in report),
                eight_points_recorded=len(rows) == 8 and route.get('complete') is True,
                binding_issue=binding_issue, hardware_test_performed=False, points=report)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--file', type=Path, required=True)
    a = ap.parse_args()
    result = audit_route(json.loads(a.file.read_text(encoding='utf-8-sig')), load_limits())
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['all_targets_within_limits'] else 2


if __name__ == '__main__':
    sys.exit(main())
