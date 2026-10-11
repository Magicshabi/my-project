"""Smooth, joint-synchronized empty-load run of the taught eight-point route.

Why: the settled-chunk runner (run_empty_route.py) splits each segment into
<=450-count chunks, drives every joint at speed 2000 / acc 100 one after another
(50 ms apart) and then waits ~1.8 s for stable feedback. That gives whip-like
starts, overshoot, joints finishing at different times and long pauses.

What this does instead, still with the verified CMD3/reg41 single-ID write:
- one write per moving joint per segment, to the taught target;
- per-joint speed/acceleration scaled so every joint follows the same
  trapezoidal time profile (start together, ramp together, arrive together);
  the path is the same straight joint-space line between taught points that
  the chunked runner followed, so no corner cutting near the baskets;
- frames for one segment are sent back to back (10 ms pacing, not 50 ms);
- the next segment starts as soon as feedback arrives (no stability dwell),
  with short dwells only after gripper close/open.

Monitoring (feedback ~10 Hz, CMD65): any joint leaving its start..goal box by
more than BOX_MARGIN, or not arriving by T + ARRIVE_TIMEOUT, stops further
segments and writes "hold at current feedback" to every joint. Cancel (STOP
file, lease, pins) stops further segments; an already-sent segment completes
to its reviewed target. Physical emergency stop is still the power switch.
Not collision checked; empty load only.
"""
import argparse
from collections import deque
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import threading
import time

import replay_joint_points as r
import reposition_empty as setup

SCHEMA = 'nexarm.smooth-cycle/1'
PROFILES = {               # counts/s, counts/s^2  (1 count = 0.088 deg)
    'gentle': (650, 1600),
    'normal': (1000, 2500),
    'brisk': (1400, 4000),
}
SPEED_REG_MAX = 3400
ACC_UNIT = 100             # servo acc register unit, counts/s^2 (HX-30HM/STS style)
MIN_JOINT_DELTA = 4        # smaller corrections are not sent (feedback jitter level)
PACING_S = 0.010
ARRIVE_TOL = 25            # proceed when every moving joint is within this
DROOP_TOL = 40             # ...or when stopped after T and within this (gravity droop)
BOX_MARGIN = 45
ARRIVE_TIMEOUT = 1.5
STOPPED_COUNTS = 3
GRIP_DWELL = {'close': 0.35, 'open': 0.20}
CAREFUL = {(1, 2): (0.6, 1.0), (5, 6): (0.6, 1.0)}   # (speed scale, min duration s) near objects
START_TOL = 24


class SmoothAbort(r.SafetyError):
    pass


def segment_profile(start, target, vmax, amax, min_duration=0.0, axes=None):
    """Synchronized trapezoid: every joint shares T and ramp time ta.

    Leading joint: trapezoid if it can reach vmax, else triangular. Each joint
    then gets v_i = d_i / (T - ta), a_i = v_i / ta, so all positions follow
    start + d_i * s(t) with the same normalized s(t).
    """
    deltas = [b - a for a, b in zip(start, target)]
    moving = [i for i, d in enumerate(deltas)
              if abs(d) >= MIN_JOINT_DELTA and (axes is None or i + 1 in axes)]
    if not moving:
        return dict(T=0.0, ta=0.0, joints=[])
    lead = max(abs(deltas[i]) for i in moving)
    if lead >= vmax * vmax / amax:
        ta = vmax / amax
        T = lead / vmax + ta
    else:
        ta = math.sqrt(lead / amax)
        T = 2 * ta
    if T < min_duration:          # stretch: keep the ramp share, slow everything
        scale = min_duration / T
        T, ta = T * scale, ta * scale
    joints = []
    for i in moving:
        v = abs(deltas[i]) / (T - ta)
        a = v / ta
        joints.append(dict(id=i + 1, target=target[i], delta=deltas[i],
                           speed=max(1, min(SPEED_REG_MAX, round(v))),
                           acc=max(1, min(254, round(a / ACC_UNIT)))))
    return dict(T=T, ta=ta, joints=joints)


def build_steps(targets, start_mode):
    """Ordered steps from the start pose to P8. Gripper events move ID6 only."""
    steps = []
    body = None
    if start_mode == 'p8':
        transit = targets[7][:]
        transit[0] = targets[0][0]
        steps.append(dict(name='p8-base-to-p1-azimuth', target=transit, axes=None, kind='move', pair=None))
        steps.append(dict(name='to-p1', target=targets[0][:], axes=None, kind='move', pair=None))
    elif start_mode != 'p1':
        raise r.SafetyError('未知起点模式')
    body = targets[0][:5]
    for point in range(2, 9):
        t = targets[point - 1][:]
        if point in (3, 7):          # gripper event: body held at the previous taught pose
            t = body + t[5:]
            steps.append(dict(name='p%d-grip-%s' % (point, 'close' if point == 3 else 'open'),
                              target=t, axes=(6,), kind='close' if point == 3 else 'open',
                              pair=(point - 1, point)))
        else:
            steps.append(dict(name='p%d' % point, target=t, axes=None, kind='move', pair=(point - 1, point)))
        body = t[:5]
    return steps


def plan_steps(steps, start, profile):
    vmax, amax = PROFILES[profile]
    current = start[:]
    out = []
    for step in steps:
        scale, min_t = CAREFUL.get(step['pair'], (1.0, 0.0))
        prof = segment_profile(current, step['target'], vmax * scale, amax * scale * scale,
                               min_t, step['axes'])
        out.append(dict(step, profile=prof, start=current[:]))
        current = [step['target'][i] if (step['axes'] is None or i + 1 in step['axes']) else current[i]
                   for i in range(6)]
    return out


def read_feedback(arm, limits):
    real = arm.get_real_joints()
    if real is None or len(real) != 6:
        raise SmoothAbort('CMD65反馈缺失')
    return r.check_counts([x[0] for x in real], limits, 'CMD65反馈')


def send_joints(arm, transport, joints):
    transport.pending = deque(r.build_frame(3, r.pack_joint_counts(j['target'], j['acc'], j['speed']), j['id'])
                              for j in joints)
    for j in joints:
        arm.move_joint_counts(j['id'], j['target'], j['acc'], j['speed'])
    if transport.pending:
        raise SmoothAbort('存在未发送帧')


def hold_here(arm, transport, feedback, emit):
    """After a monitoring fault: every joint's goal := its current feedback."""
    joints = [dict(id=i + 1, target=v, acc=50, speed=500) for i, v in enumerate(feedback)]
    try:
        transport.failed = False
        send_joints(arm, transport, joints)
        emit('hold_written', goals=feedback)
    except BaseException as exc:          # never mask the original fault
        emit('hold_failed', reason=str(exc))


def run_step(arm, transport, step, emit, cancelled, limits, clock=time.monotonic, sleep=time.sleep):
    prof = step['profile']
    if not prof['joints']:
        emit('step_skip', name=step['name'])
        return None
    if cancelled():
        raise r.SafetyError('停止后续段')
    start, target = step['start'], step['target']
    ids = [j['id'] - 1 for j in prof['joints']]
    lo = [min(a, b) - BOX_MARGIN for a, b in zip(start, target)]
    hi = [max(a, b) + BOX_MARGIN for a, b in zip(start, target)]
    emit('step_begin', name=step['name'], T=round(prof['T'], 3), ta=round(prof['ta'], 3),
         joints=prof['joints'], start=start, target=target)
    t0 = clock()
    send_joints(arm, transport, prof['joints'])
    history = deque(maxlen=3)
    fb = start
    while True:
        sleep(0.06)
        fb = read_feedback(arm, limits)
        t = clock() - t0
        emit('sample', name=step['name'], dt=round(t, 3), cmd65=fb)   # 't' is emit's own timestamp
        history.append(fb)
        if any(not lo[i] <= fb[i] <= hi[i] for i in range(6)):
            hold_here(arm, transport, fb, emit)
            raise SmoothAbort('%s：关节越出本段区间%s，已写入原地保持' % (step['name'], fb))
        err = [abs(fb[i] - target[i]) for i in ids]
        stopped = len(history) == 3 and all(
            max(h[i] for h in history) - min(h[i] for h in history) <= STOPPED_COUNTS for i in ids)
        if step['kind'] == 'close' and t >= prof['T'] and stopped:
            break                            # a gripper closing on a handle stalls short of target
        if max(err) <= ARRIVE_TOL:
            break
        if t >= prof['T'] and stopped and max(err) <= DROOP_TOL:
            emit('droop_accepted', name=step['name'], err=err)
            break
        if t >= prof['T'] + ARRIVE_TIMEOUT:
            hold_here(arm, transport, fb, emit)
            raise SmoothAbort('%s：%.1fs内未到位(误差%s)，已写入原地保持' % (step['name'], t, err))
    dwell = GRIP_DWELL.get(step['kind'], 0.0)
    if dwell:
        sleep(dwell)
    elapsed = clock() - t0
    emit('step_done', name=step['name'], elapsed=round(elapsed, 3), feedback=fb)
    return dict(name=step['name'], planned_T=round(prof['T'], 3), elapsed=round(elapsed, 3),
                feedback=fb, target=target)


def validate_plan(plan, pins):
    if (plan.get('schema') != SCHEMA or plan.get('empty_load_only') is not True
            or plan.get('start_mode') not in ('p1', 'p8') or plan.get('profile') not in PROFILES
            or plan.get('cycles') != 1 or plan.get('pins') != pins):
        raise r.SafetyError('平滑整程计划范围或文件绑定不匹配')


def describe(planned):
    rows = []
    for s in planned:
        p = s['profile']
        rows.append('%-22s T=%.2fs ramp=%.2fs  %s' % (
            s['name'], p['T'], p['ta'],
            ' '.join('ID%d:%+d@v%d/a%d' % (j['id'], j['delta'], j['speed'], j['acc']) for j in p['joints'])))
    total = sum(s['profile']['T'] + GRIP_DWELL.get(s['kind'], 0) for s in planned)
    rows.append('估计纯运动+夹爪停留合计约 %.1fs（另加每段到位判定约0.1~0.3s）' % total)
    return '\n'.join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', type=Path, help='平滑整程计划JSON（运行时必需）')
    ap.add_argument('--port')
    ap.add_argument('--holder', default='claude', choices=('claude', 'codex'))
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--empty-load', action='store_true')
    ap.add_argument('--dry-run', choices=('p1', 'p8'), help='只打印时间规划，不连接串口')
    ap.add_argument('--profile', default='normal', choices=sorted(PROFILES))
    a = ap.parse_args()
    files = dict(route=r.SOURCE, limits=r.DEFAULT_LIMITS, hold=r.DIAGNOSTIC_HOLD,
                 runner=Path(__file__), replay=Path(r.__file__))
    route_raw = r.SOURCE.read_bytes()
    route = json.loads(route_raw.decode('utf-8'))
    limits = r.load_limits()
    targets = r.resolve_route(route, limits)
    r.check_binding(route, limits)
    if a.dry_run:
        ref = targets[0] if a.dry_run == 'p1' else targets[7]
        print(describe(plan_steps(build_steps(targets, a.dry_run), ref, a.profile)))
        return 0
    if not (a.plan and a.port and a.go and a.empty_load):
        ap.error('运行需要 --plan --port --go --empty-load')
    if route.get('require_powered_sliders') is not True:
        raise r.SafetyError('只接受主动示教路线')
    pins = {k: r.sha(p) for k, p in files.items()}
    plan = json.loads(a.plan.read_text(encoding='utf-8'))
    plan_sha = r.sha(a.plan)
    validate_plan(plan, pins)
    steps = build_steps(targets, plan['start_mode'])
    reference = targets[0] if plan['start_mode'] == 'p1' else targets[7]
    lease, lease_sha, expires = r.require_device_lease(a.port, holder=a.holder)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('smooth_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run / 'route_snapshot.json').write_bytes(route_raw)
    (run / 'plan.json').write_bytes(a.plan.read_bytes())
    (run / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    result = dict(plan=plan, port=a.port, steps=[], controller_complete=False, operator_confirmed=False,
                  complete=False, empty_load=True, physical_stop_verified=False, collision_checked=False,
                  smooth_blending='synchronized-trapezoid, stop-at-each-point')

    def save():
        (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def cancelled():
        try:
            return ((run / 'STOP').exists() or datetime.now(timezone.utc) >= expires
                    or r.sha(lease) != lease_sha or r.sha(a.plan) != plan_sha
                    or any(r.sha(p) != pins[k] for k, p in files.items()))
        except OSError:
            return True
    save()
    print('RUN_DIR=' + str(run), flush=True)
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        mutex = threading.Lock()

        def emit(event, **data):
            with mutex:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = r.ReplayTransport(emit, cancelled, limits)
        arm = r.ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        arm.gate.min_period_s = PACING_S
        try:
            arm.connect_serial(a.port)
            live = r.configuration(arm)
            if not r.same_configuration(route['configuration'], live):
                raise r.SafetyError('设备配置改变')
            battery = arm.get_battery_mv()
            if battery is None:
                raise r.SafetyError('缺少供电报告')
            start, _ = r.capture_stable(arm, emit)
            gap = max(abs(x - y) for x, y in zip(start, reference))
            if gap > START_TOL:
                raise r.SafetyError('起点偏离%s %d刻度；不自动追赶' % (plan['start_mode'].upper(), gap))
            planned = plan_steps(steps, start, plan['profile'])
            result.update(configuration=live, battery_mv=battery, start=start,
                          planned=[dict(name=s['name'], T=round(s['profile']['T'], 3)) for s in planned])
            save()
            arm.enable_motion(True)
            t_run = time.monotonic()
            for step in planned:
                done = run_step(arm, transport, step, emit, cancelled, limits)
                if done:
                    result['steps'].append(done)
                    save()
                    print('STEP_DONE=' + json.dumps(done), flush=True)
            result.update(controller_complete=True, total_s=round(time.monotonic() - t_run, 2))
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
            emit('aborted', reason=result['error'])
            print('STOP=' + result['error'], flush=True)
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(serial_closed=True, motion_attempts=transport.motion_attempts, stats=transport.stats())
            save()
    if result['controller_complete']:
        answer = setup.ask_observation('OBSERVE: 串口已关闭。yes=整程动作平顺、到位正确、无异常\n')
        result.update(operator_answer=answer, operator_confirmed=answer == 'yes', complete=answer == 'yes')
        save()
    print('RESULT=' + json.dumps(dict(complete=result['complete'], error=result.get('error'),
                                      total_s=result.get('total_s'), result_file=str(run / 'result.json')),
                                 ensure_ascii=False), flush=True)
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
