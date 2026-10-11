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
Not collision checked. Empty load by default; a basket run needs a separate
load=basket plan (empty_load_only=false, gentle only) and --loaded.
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
LOADED_PROFILE = 'gentle'
GRIP_SQUEEZE = 200         # basket runs: P3 close goal beyond taught; 2026-10-11 basket slipped at taught goal
GRIP_CLOSED_MAX = 2731     # ID6 software limit (joint_limits_20261010.json)
HOME_FILE = Path(__file__).resolve().parent / 'data' / 'home_pose_20261011.json'   # user-set 2026-10-11


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


def build_steps(targets, start_mode, squeeze=0, home=None, from_p8=False):
    """Ordered steps from the start pose to P8. Gripper events move ID6 only.

    squeeze > 0 (basket runs): the P3 close goal is taught+squeeze so the jaws
    press on the handle, and P4..P6 leave ID6 out, so its goal stays squeezed.
    start_mode 'home': [P8 -> home if from_p8], then base-only turn to P1's
    azimuth at the home pose, then down to P1 (same pattern as the P8 transit).
    """
    steps = []
    body = None
    if start_mode in ('p8', 'home'):
        if start_mode == 'home':
            if home is None:
                raise r.SafetyError('缺少初始位置')
            if from_p8:
                steps.append(dict(name='p8-to-home', target=home[:], axes=None, kind='move', pair=None))
            origin, label = home, 'home'
        else:
            origin, label = targets[7], 'p8'
        transit = origin[:]
        transit[0] = targets[0][0]
        steps.append(dict(name='%s-base-to-p1-azimuth' % label, target=transit, axes=None, kind='move', pair=None))
        steps.append(dict(name='to-p1', target=targets[0][:], axes=None, kind='move', pair=None))
    elif start_mode != 'p1':
        raise r.SafetyError('未知起点模式')
    body = targets[0][:5]
    for point in range(2, 9):
        t = targets[point - 1][:]
        if point in (3, 7):          # gripper event: body held at the previous taught pose
            t = body + t[5:]
            if point == 3 and squeeze:
                t[5] = min(t[5] + squeeze, GRIP_CLOSED_MAX)
            steps.append(dict(name='p%d-grip-%s' % (point, 'close' if point == 3 else 'open'),
                              target=t, axes=(6,), kind='close' if point == 3 else 'open',
                              pair=(point - 1, point)))
        else:
            carried = squeeze and 4 <= point <= 6
            steps.append(dict(name='p%d' % point, target=t, axes=(1, 2, 3, 4, 5) if carried else None,
                              kind='move', pair=(point - 1, point), carry=bool(carried)))
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


def hold_here(arm, transport, feedback, emit, grip_goal=None):
    """After a monitoring fault: every joint's goal := its current feedback.

    grip_goal: while carrying, ID6 keeps its squeezed goal instead (a goal at
    the feedback would relax the jaws and drop the basket).
    """
    goals = list(feedback)
    if grip_goal is not None:
        goals[5] = grip_goal
    joints = [dict(id=i + 1, target=v, acc=50, speed=500) for i, v in enumerate(goals)]
    try:
        transport.failed = False
        send_joints(arm, transport, joints)
        emit('hold_written', goals=goals)
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
    grip_goal = start[5] if step.get('carry') else None    # planned ID6 goal = squeezed close goal
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
            hold_here(arm, transport, fb, emit, grip_goal)
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
            hold_here(arm, transport, fb, emit, grip_goal)
            raise SmoothAbort('%s：%.1fs内未到位(误差%s)，已写入原地保持' % (step['name'], t, err))
    dwell = GRIP_DWELL.get(step['kind'], 0.0)
    if dwell:
        sleep(dwell)
    elapsed = clock() - t0
    emit('step_done', name=step['name'], elapsed=round(elapsed, 3), feedback=fb)
    return dict(name=step['name'], planned_T=round(prof['T'], 3), elapsed=round(elapsed, 3),
                feedback=fb, target=target)


def load_home(limits):
    data = json.loads(HOME_FILE.read_text(encoding='utf-8'))
    if data.get('schema') != 'nexarm.home-pose/1':
        raise r.SafetyError('初始位置文件格式不符')
    return r.check_counts(data['counts'], limits, '初始位置')


def steps_for_start(targets, start_mode, start, squeeze=0, home=None):
    """Pick the step list for the live start pose; never chase an unknown pose.

    p1/p8: start must be at that point. home: start at home, or at P8 (end of
    the previous cycle), in which case P8 -> home is prepended.
    """
    gap = lambda ref: max(abs(x - y) for x, y in zip(start, ref))
    if start_mode in ('p1', 'p8'):
        ref = targets[0] if start_mode == 'p1' else targets[7]
        if gap(ref) > START_TOL:
            raise r.SafetyError('起点偏离%s %d刻度；不自动追赶' % (start_mode.upper(), gap(ref)))
        return build_steps(targets, start_mode, squeeze)
    if start_mode != 'home' or home is None:
        raise r.SafetyError('未知起点模式')
    if gap(home) <= START_TOL:
        return build_steps(targets, 'home', squeeze, home)
    if gap(targets[7]) <= START_TOL:
        return build_steps(targets, 'home', squeeze, home, from_p8=True)
    raise r.SafetyError('起点既不在初始位置(偏%d)也不在P8(偏%d)；不自动追赶' % (gap(home), gap(targets[7])))


def validate_plan(plan, pins, loaded=False):
    if (plan.get('schema') != SCHEMA
            or plan.get('start_mode') not in ('p1', 'p8', 'home') or plan.get('profile') not in PROFILES
            or plan.get('cycles') != 1 or plan.get('pins') != pins):
        raise r.SafetyError('平滑整程计划范围或文件绑定不匹配')
    # Loaded (basket) runs: separate plan + --loaded flag, gentle only (user-approved 2026-10-11).
    if plan.get('load', 'empty') == 'basket':
        if not loaded or plan.get('empty_load_only') is not False or plan.get('profile') != LOADED_PROFILE:
            raise r.SafetyError('带载计划须 --loaded、empty_load_only=false 且仅限%s' % LOADED_PROFILE)
    elif plan.get('load', 'empty') != 'empty' or loaded or plan.get('empty_load_only') is not True:
        raise r.SafetyError('空载计划须 --empty-load 且 empty_load_only=true')


def pinned_files():
    return dict(route=r.SOURCE, limits=r.DEFAULT_LIMITS, hold=r.DIAGNOSTIC_HOLD,
                runner=Path(__file__), replay=Path(r.__file__), home=HOME_FILE)


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
    load = ap.add_mutually_exclusive_group()
    load.add_argument('--empty-load', action='store_true')
    load.add_argument('--loaded', action='store_true', help='带篮运行：仅接受 load=basket 的 gentle 计划')
    ap.add_argument('--dry-run', choices=('p1', 'p8', 'home', 'home-from-p8'), help='只打印时间规划，不连接串口')
    ap.add_argument('--profile', default='normal', choices=sorted(PROFILES))
    a = ap.parse_args()
    files = pinned_files()
    route_raw = r.SOURCE.read_bytes()
    route = json.loads(route_raw.decode('utf-8'))
    limits = r.load_limits()
    targets = r.resolve_route(route, limits)
    r.check_binding(route, limits)
    home = load_home(limits)
    if a.dry_run:
        mode = 'home' if a.dry_run.startswith('home') else a.dry_run
        ref = dict(p1=targets[0], p8=targets[7], home=home)['p8' if a.dry_run == 'home-from-p8' else mode]
        squeeze = GRIP_SQUEEZE if a.loaded else 0
        print(describe(plan_steps(steps_for_start(targets, mode, ref, squeeze, home), ref, a.profile)))
        return 0
    if not (a.plan and a.port and a.go and (a.empty_load or a.loaded)):
        ap.error('运行需要 --plan --port --go 以及 --empty-load 或 --loaded')
    if route.get('require_powered_sliders') is not True:
        raise r.SafetyError('只接受主动示教路线')
    pins = {k: r.sha(p) for k, p in files.items()}
    plan = json.loads(a.plan.read_text(encoding='utf-8'))
    plan_sha = r.sha(a.plan)
    validate_plan(plan, pins, loaded=a.loaded)
    squeeze = GRIP_SQUEEZE if a.loaded else 0
    lease, lease_sha, expires = r.require_device_lease(a.port, holder=a.holder)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('smooth_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run / 'route_snapshot.json').write_bytes(route_raw)
    (run / 'plan.json').write_bytes(a.plan.read_bytes())
    (run / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    result = dict(plan=plan, port=a.port, steps=[], controller_complete=False, operator_confirmed=False,
                  complete=False, empty_load=not a.loaded, physical_stop_verified=False, collision_checked=False,
                  smooth_blending='synchronized-trapezoid, stop-at-each-point', grip_squeeze=squeeze)

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
            steps = steps_for_start(targets, plan['start_mode'], start, squeeze, home)
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
