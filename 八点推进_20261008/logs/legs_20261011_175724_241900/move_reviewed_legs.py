"""Run exactly one reviewed empty-load leg from a waypoint plan.

Each leg has an explicit expected start and target written after a read-only
pose review. The arm must already be within the leg's start tolerance; there is
no automatic catch-up, retry or return. Motion uses the verified CMD3/reg41
single-ID writes in chunks of at most 400 counts (not bus-synchronized, not
collision checked). The serial port is closed before the operator is asked to
confirm the physical result.
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import threading
import time

import replay_joint_points as r
import reposition_empty as setup
from nexarm import ArmController
from nexarm.eight_point import configuration, same_configuration

SCHEMA = 'nexarm.reviewed-legs/1'
MAX_CHUNK = 400  # leaves margin for settle tolerance under the 450-count per-call cap
MAX_START_TOLERANCE = 24
ALLOWED_TOLERANCES = ([10] * 6, [10, 10, 20, 10, 10, 10], [20] * 6)


def validate_plan(plan, route_sha, limits_sha, hold_sha, limits):
    if (plan.get('schema') != SCHEMA or plan.get('source_sha256') != route_sha
            or plan.get('limits_sha256') != limits_sha or plan.get('empty_load_only') is not True
            or plan.get('historical_hold_sha256') != hold_sha):
        raise r.SafetyError('路点计划与当前数据/限位/故障锁不匹配')
    legs = plan.get('legs')
    if not isinstance(legs, list) or not legs:
        raise r.SafetyError('路点计划没有腿段')
    names = [leg.get('name') for leg in legs]
    if len(set(names)) != len(names) or not all(isinstance(n, str) and n for n in names):
        raise r.SafetyError('腿段名称需唯一')
    for i, leg in enumerate(legs):
        start = r.check_counts(leg.get('expected_start'), limits, leg['name'] + '起点')
        r.check_counts(leg.get('target'), limits, leg['name'] + '终点')
        if leg.get('settle_tolerances', [20] * 6) not in ALLOWED_TOLERANCES:
            raise r.SafetyError('本轮仅允许已审查的轴容差')
        tol = leg.get('start_tolerance', 12)
        if type(tol) is not int or not 1 <= tol <= MAX_START_TOLERANCE:
            raise r.SafetyError('起点容差需为1..24刻度')
        if i and start != legs[i - 1]['target']:
            raise r.SafetyError('腿段不连续：起点须等于上一段终点')
    return legs


def find_leg(legs, name):
    for index, leg in enumerate(legs):
        if leg['name'] == name:
            return index, leg
    raise r.SafetyError('计划中没有该腿段')


def chunk_goals(start, target, max_chunk=MAX_CHUNK):
    n = max(1, math.ceil(max(abs(a - b) for a, b in zip(start, target)) / max_chunk))
    return [[round(a + (b - a) * k / n) for a, b in zip(start, target)] for k in range(1, n + 1)]


def execute_leg(arm, leg, emit, cancelled, route_sha, limits, limits_sha):
    before, _ = r.capture_stable(arm, emit)
    gap = max(abs(a - b) for a, b in zip(before, leg['expected_start']))
    if gap > leg.get('start_tolerance', 12):
        raise r.SafetyError('当前姿态偏离本段审查起点%d刻度；不自动追赶' % gap)
    tolerances = leg.get('settle_tolerances', [20] * 6)
    current, chunks = before, []
    for goal in chunk_goals(before, leg['target']):
        chunk = setup.execute_direct_p1(arm, current, goal, emit, cancelled, route_sha, limits,
                                        limits_sha, tolerances)
        current = chunk['returned_report']
        chunks.append(chunk)
    return dict(start=before, target=leg['target'], returned_report=current, chunks=chunks,
                controller_converged=True, operator_confirmed=False)


def leg_confirmed(logs, plan_sha, name):
    """True when a closed-port run of this plan's leg has an operator 'yes'."""
    for path in sorted(Path(logs).glob('legs_*/result.json')):
        try:
            res = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        if (res.get('plan_sha256') == plan_sha and res.get('leg') == name
                and res.get('serial_closed') is True and res.get('operator_confirmed') is True):
            return True
    return False


def record_observation(run, answer):
    """Record the operator's answer for a finished leg whose port is already closed."""
    path = Path(run) / 'result.json'
    res = json.loads(path.read_text(encoding='utf-8'))
    if res.get('serial_closed') is not True or 'segment' not in res or res.get('error'):
        raise r.SafetyError('该运行未正常结束，不能补记现场确认')
    if res.get('operator_answer') not in (None, 'no observation'):
        raise r.SafetyError('该运行已有现场答复，不覆盖')
    answer = answer.strip()
    res.update(operator_answer=answer, operator_confirmed=answer == 'yes', complete=answer == 'yes',
               observation_recorded_separately=True)
    with (Path(run) / 'events.jsonl').open('a', encoding='utf-8') as log:
        log.write(json.dumps(dict(event='operator_observation', t=time.monotonic(), answer=answer,
                                  recorded_separately=True), ensure_ascii=False) + '\n')
    path.write_text(json.dumps(res, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port')
    ap.add_argument('--plan', type=Path)
    ap.add_argument('--leg')
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--empty-load', action='store_true')
    ap.add_argument('--record-observation', type=Path, metavar='RUN_DIR',
                    help='只补记已结束运行的现场答复，不连接串口')
    ap.add_argument('--answer')
    a = ap.parse_args()
    if a.record_observation is not None:
        if a.answer is None:
            ap.error('--record-observation 需要 --answer')
        res = record_observation(a.record_observation, a.answer)
        print('RECORDED=' + json.dumps(dict(leg=res.get('leg'), operator_confirmed=res['operator_confirmed']),
                                       ensure_ascii=False), flush=True)
        return 0 if res['operator_confirmed'] else 2
    if not (a.port and a.plan and a.leg):
        ap.error('运动需要 --port --plan --leg')
    if not (a.go and a.empty_load):
        ap.error('需现场确认空载清障并显式--go --empty-load')
    raw = r.SOURCE.read_bytes()
    route_sha = r.hashlib.sha256(raw).hexdigest()
    route = json.loads(raw.decode('utf-8'))
    limits = r.load_limits()
    limits_sha = r.sha(r.DEFAULT_LIMITS)
    hold_sha = r.sha(r.DIAGNOSTIC_HOLD)
    plan = json.loads(a.plan.read_text(encoding='utf-8'))
    plan_sha = r.sha(a.plan)
    legs = validate_plan(plan, route_sha, limits_sha, hold_sha, limits)
    index, leg = find_leg(legs, a.leg)
    if index and not leg_confirmed(r.HERE / 'logs', plan_sha, legs[index - 1]['name']):
        raise r.SafetyError('上一段%s没有现场yes记录；不运行本段' % legs[index - 1]['name'])
    lease_path, lease_sha, expires = r.require_device_lease(a.port)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('legs_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    for source in (Path(__file__), Path(r.__file__), Path(setup.__file__), a.plan):
        (run / source.name).write_bytes(source.read_bytes())
    result = dict(started_at=now.isoformat(), leg=leg['name'], leg_index=index + 1, leg_count=len(legs),
                  plan_sha256=plan_sha, source_sha256=route_sha, limits_sha256=limits_sha,
                  empty_load=True, complete=False, operator_confirmed=False,
                  physical_stop_verified=False, collision_checked=False, port=a.port)

    def save():
        (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def cancelled():
        try:
            return ((run / 'STOP').exists() or datetime.now(timezone.utc) >= expires
                    or r.sha(lease_path) != lease_sha or r.sha(a.plan) != plan_sha
                    or r.sha(r.DIAGNOSTIC_HOLD) != hold_sha)
        except OSError:
            return True
    save()
    print('RUN_DIR=' + str(run), flush=True)
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        log_lock = threading.Lock()

        def emit(kind, **data):
            with log_lock:
                log.write(json.dumps(dict(event=kind, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = r.ReplayTransport(emit, cancelled, limits)
        arm = ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        try:
            arm.connect_serial(a.port)
            live = configuration(arm)
            if not same_configuration(route['configuration'], live):
                raise r.SafetyError('设备配置改变')
            battery = arm.get_battery_mv()
            if battery is None:
                raise r.SafetyError('缺少供电报告')
            result.update(configuration=live, battery_mv=battery)
            save()
            arm.enable_motion(True)
            result['segment'] = execute_leg(arm, leg, emit, cancelled, route_sha, limits, limits_sha)
            save()
            print('LEG_DONE=' + json.dumps(result['segment']['returned_report']), flush=True)
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
            emit('aborted', reason=result['error'])
            print('STOP: 停止后续写入；无自动重试或回位。', flush=True)
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(motion_attempts=transport.motion_attempts, serial_closed=True, stats=transport.stats())
            save()
    if 'segment' in result and 'error' not in result:
        answer = setup.ask_observation()
        result.update(observation_requested_after_close=True, operator_answer=answer,
                      operator_confirmed=answer == 'yes', complete=answer == 'yes')
        with (run / 'events.jsonl').open('a', encoding='utf-8') as log:
            log.write(json.dumps(dict(event='operator_observation', t=time.monotonic(), answer=answer),
                                 ensure_ascii=False) + '\n')
        save()
    print('RESULT=' + json.dumps(dict(leg=leg['name'], complete=result['complete'], error=result.get('error'),
                                      motion_attempts=result.get('motion_attempts'),
                                      result_file=str(run / 'result.json')), ensure_ascii=False), flush=True)
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
