"""Read-only joint teaching capture; no motion, torque, configuration or NVS writes."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
LIB = HERE.parent / '舵机基础运行逻辑'
sys.path.insert(0, str(LIB))
from nexarm import ArmController, Transport, SafetyError, build_frame
from nexarm.eight_point import STAGES, configuration, same_configuration

SCHEMA = 'nexarm.joint-eight/1'
DEFAULT = HERE / 'data' / 'eight_joints_20261009.json'


def timestamp():
    return datetime.now(timezone(timedelta(hours=8))).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def create_route(path, require_powered_sliders=False):
    legacy = HERE.parent / 'validation_runs/readout_20261005_180008/eight_points.json'
    route = dict(schema=SCHEMA, units='raw_encoder_counts', servo_ids=list(range(1, 7)),
                 created_at=timestamp(), method='NexArm.exe powered joint sliders + read-only host capture' if require_powered_sliders else 'Positioning method not yet confirmed',
                 require_powered_sliders=require_powered_sliders,
                 legacy_source=dict(path=str(legacy), sha256=digest(legacy)),
                 configuration=None, complete=False, execution_verified=False, points=[])
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as f:
        json.dump(route, f, ensure_ascii=False, indent=2)
    return route


def check_route(route, point):
    if route.get('schema') != SCHEMA or route.get('units') != 'raw_encoder_counts':
        raise SafetyError('点位文件格式不匹配')
    if route.get('servo_ids') != list(range(1, 7)):
        raise SafetyError('关节顺序不是 ID1..6')
    rows = route.get('points')
    if not isinstance(rows, list) or not 1 <= point <= 8 or len(rows) != point - 1:
        raise SafetyError('只能依次新增下一个点；已有点不会自动覆盖')
    for i, row in enumerate(rows):
        if row.get('point') != i + 1 or row.get('stage') != STAGES[i]:
            raise SafetyError('已有点位顺序不正确')
        checked_counts(row.get('encoder_positions'))
        if row.get('physical_pose_confirmed') is not True:
            raise SafetyError('已有点位缺少现场确认')
        if route.get('require_powered_sliders') and row.get('motor_driven_pose_confirmed_by_operator') is not True:
            raise SafetyError('已有点位缺少电机主动到达的现场确认')


def check_positioning_method(route, confirmed):
    if route.get('require_powered_sliders') and confirmed is not True:
        raise SafetyError('本轮只采电机主动到达的姿态；需用户确认保持力矩并用滑块摆位')


def checked_counts(values):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise SafetyError('必须有六轴位置反馈')
    if any(type(v) is not int or not 0 <= v <= 4095 for v in values):
        raise SafetyError('编码器反馈超出协议范围')
    return list(values)


class ReadOnlyTransport(Transport):
    def __init__(self, emit):
        super().__init__(log=lambda msg: emit('transport', message=msg))
        self.emit = emit

    def send(self, cmd, args=b'', target_id=255):
        allowed = target_id == 255 and (
            (cmd in (1, 2, 11, 65, 80, 64) and not args)
            or (cmd == 58 and len(args) == 1 and 1 <= args[0] <= 6))
        if not allowed:
            raise SafetyError('采集器仅允许查询；拒绝运动、力矩及配置写入')
        self.emit('tx_attempt', cmd=cmd, id=target_id,
                  wire_hex=build_frame(cmd, args, target_id).hex())
        super().send(cmd, args, target_id)
        self.emit('tx_complete', cmd=cmd, id=target_id)


def capture_stable(arm, emit):
    samples = []
    for _ in range(8):
        pose = arm.get_pose()
        real = arm.get_real_joints()
        if pose is None or real is None or len(real) != 6:
            raise SafetyError('CMD11 或 CMD65 回包缺失；未保存')
        a = checked_counts(pose.servos)
        b = checked_counts([x[0] for x in real])
        if any(abs(x-y) > 6 for x, y in zip(a, b)):
            raise SafetyError('CMD11/CMD65 位置不一致；未保存')
        sample = dict(cmd11_counts=a, cmd65_counts=b,
                      controller_pose={k: getattr(pose, k) for k in ('x','y','z','pitch','roll','claw')})
        samples.append(sample)
        emit('sample', **sample)
        time.sleep(.15)
    for key in ('cmd11_counts', 'cmd65_counts'):
        if any(max(s[key][i] for s in samples)-min(s[key][i] for s in samples) > 5
               for i in range(6)):
            raise SafetyError('关节仍在变化，未保存；保持摆稳后再采集')
    # Both channels come from the same controller: agreement is not independent
    # encoder/geometry validation. The operator confirms the physical pose.
    return samples[-1]['cmd65_counts'], samples


def add_capture(route, point, counts, samples, config, observed, log_dir,
                powered_sliders_confirmed=False):
    check_route(route, point)
    check_positioning_method(route, powered_sliders_confirmed)
    checked_counts(counts)
    if not observed.strip():
        raise SafetyError('缺少现场姿态确认')
    if route['configuration'] is not None and not same_configuration(route['configuration'], config):
        raise SafetyError('设备配置与前面点位不同；未保存')
    delta = None
    if route['points']:
        delta = [a-b for a,b in zip(counts, route['points'][-1]['encoder_positions'])]
        if point in (3, 7) and any(abs(v) > 12 for v in delta[:5]):
            raise SafetyError('夹爪开合点的臂体姿态变化过大；需重新核对，未保存')
    result = json.loads(json.dumps(route))
    result['points'].append(dict(point=point, stage=STAGES[point-1], encoder_positions=counts,
        captured_at=timestamp(), physical_pose_confirmed=True, observation=observed,
        samples=samples, delta_from_previous=delta,
        large_joint_change_requires_review=bool(delta and max(map(abs, delta[:5])) > 350),
        evidence_dir=str(log_dir.resolve()), path_to_this_point_verified=False))
    result['points'][-1]['motor_driven_pose_confirmed_by_operator'] = powered_sliders_confirmed is True
    result.update(configuration=config, complete=point == 8, execution_verified=False)
    return result


def save_revision(path, route, original_bytes):
    if path.read_bytes() != original_bytes:
        raise SafetyError('点位文件已被其他进程修改；拒绝覆盖')
    backup = path.with_name(path.name + '.' + str(time.time_ns()) + '.bak')
    with backup.open('xb') as f:
        f.write(original_bytes)
    temp = path.with_name(path.name + '.' + str(time.time_ns()) + '.tmp')
    payload = (json.dumps(route, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    with temp.open('xb') as f:
        f.write(payload)
        f.flush()
        os.fsync(f.fileno())
    if path.read_bytes() != original_bytes:
        raise SafetyError('保存前文件发生变化；原文件保留')
    os.replace(temp, path)
    if path.read_bytes() != payload:
        raise IOError('点位文件读回不一致')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=('init', 'capture'))
    ap.add_argument('--file', type=Path, default=DEFAULT)
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--point', type=int, choices=range(1, 9))
    ap.add_argument('--observed')
    ap.add_argument('--require-powered-sliders', action='store_true',
                    help='init: require explicit motor-driven positioning confirmation for each capture')
    ap.add_argument('--powered-sliders-confirmed', action='store_true',
                    help='capture: operator explicitly confirmed powered slider positioning, not manual dragging')
    a = ap.parse_args()
    if a.mode == 'init':
        create_route(a.file, a.require_powered_sliders)
        print('CREATED=' + str(a.file.resolve()))
        return
    if a.point is None or not a.observed or not a.observed.strip():
        ap.error('capture 需要 --point 与明确的 --observed 现场确认')
    original = a.file.read_bytes()
    route = json.loads(original.decode('utf-8'))
    check_route(route, a.point)
    check_positioning_method(route, a.powered_sliders_confirmed)
    run = HERE / 'logs' / ('capture_' + datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    result = dict(point=a.point, saved=False, motion_commands=0, started_at=timestamp(),
                  sources={str(p): digest(p) for p in (Path(__file__), LIB/'nexarm/arm.py', LIB/'nexarm/transport.py', LIB/'nexarm/commands.py')})
    with (run/'events.jsonl').open('w', encoding='utf-8') as log:
        lock = threading.Lock()
        def emit(event, **data):
            with lock:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False)+'\n')
                log.flush()
        transport = ReadOnlyTransport(emit)
        transport.on_any(lambda f: emit('rx', id=f.id, cmd=f.cmd, args=f.args.hex()))
        arm = ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        try:
            arm.connect_serial(a.port)
            config = configuration(arm)
            if (config['version']['esp_str'], config['version']['at32_str']) != ('1.0.2', '1.0.42'):
                raise SafetyError('固件版本已改变，先核对')
            voltage = arm.get_battery_mv()
            if voltage is None:
                raise SafetyError('电压报告缺失')
            counts, samples = capture_stable(arm, emit)
            if transport.stats()['checksum_errors']:
                raise SafetyError('采集出现坏帧；未保存')
            updated = add_capture(route, a.point, counts, samples, config, a.observed, run,
                                  powered_sliders_confirmed=a.powered_sliders_confirmed)
            save_revision(a.file, updated, original)
            result.update(saved=True, encoder_positions=counts, battery_mv=voltage,
                          configuration=config, file=str(a.file.resolve()), sha256=digest(a.file))
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
            raise
        finally:
            result['stats'] = transport.stats()
            arm.close()
            result['serial_closed'] = True
            (run/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
            print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
