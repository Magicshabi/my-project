"""Serial executor for colour commands ("红-蓝-黄") over COM11.

Compiles the command with sequence.py, then drives the joints with the
synchronized-trapezoid motion layer from 八点推进_20261008/smooth_route.py
(CMD3/reg41 whitelist writes, start..goal box monitoring, hold-on-fault that
keeps the squeezed ID6 goal while carrying). Not collision checked; physical
emergency stop is still the power switch. Create a STOP file in the run
directory to stop further segments.

Before running: release the host-software serial connection, then
  python handoff/lock.py acquire COM11 --holder claude --minutes 60 --purpose 连续作业执行

  python executor.py 红-蓝-黄 --dry-run                     # offline timing plan, no serial
  python executor.py 红-蓝-黄 --port COM11 --go --empty-load
  python executor.py 红-蓝-黄 --port COM11 --go --loaded    # basket run, gentle only
  python executor.py --port COM11 --go --loaded             # interactive: one command per batch

A batch must start at P0. If the arm rests at the previous batch's S8, pass
--from-s8 to prepend a direct S8 -> P0 move (only if that move was verified).
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
import sys
import threading
import time
from pathlib import Path

import collection as c
import sequence as seq

sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
import replay_joint_points as r  # noqa: E402
import smooth_route as sr  # noqa: E402

CAREFUL_POINTS = {2: (0.6, 1.0), 6: (0.6, 1.0)}   # descending onto the handle / into the area
START_TOL = sr.START_TOL
# 2026-10-13 user choice A: a joint that stops short of a host-recorded target is accepted at its
# reachable position (the host software plays the same files without complaint).
STALL_TOL = 120
# ID4 was never measured above ~3088 powered, and a 3234 goal once left it frozen (ledger id4-limit).
# Commands are clamped here so ID4 moves to its reachable top instead of risking no motion at all;
# route files keep the recorded values.
ID4_REACH = 3085


def stall_ok(joint, fb, target):
    return abs(fb - target) <= STALL_TOL


def run_step(arm, transport, step, emit, cancelled, limits, clock=time.monotonic, sleep=time.sleep):
    """smooth_route.run_step, plus: after the planned time, joints that have stopped short are
    accepted if stall_ok (logged as 'stall_accepted'). Box and timeout checks are unchanged."""
    prof = step['profile']
    if not prof['joints']:
        emit('step_skip', name=step['name'])
        return None
    if cancelled():
        raise r.SafetyError('停止后续段')
    start, target = step['start'], step['target']
    ids = [j['id'] - 1 for j in prof['joints']]
    lo = [min(a, b) - sr.BOX_MARGIN for a, b in zip(start, target)]
    hi = [max(a, b) + sr.BOX_MARGIN for a, b in zip(start, target)]
    emit('step_begin', name=step['name'], T=round(prof['T'], 3), joints=prof['joints'], start=start, target=target)
    grip_goal = start[5] if step.get('carry') else None
    t0 = clock()
    sr.send_joints(arm, transport, prof['joints'])
    history = []
    stalled = {}
    while True:
        sleep(0.06)
        fb = sr.read_feedback(arm, limits)
        t = clock() - t0
        emit('sample', name=step['name'], dt=round(t, 3), cmd65=fb)
        history = (history + [fb])[-3:]
        if any(not lo[i] <= fb[i] <= hi[i] for i in range(6)):
            sr.hold_here(arm, transport, fb, emit, grip_goal)
            raise sr.SmoothAbort('%s：关节越出本段区间%s，已写入原地保持' % (step['name'], fb))
        err = [abs(fb[i] - target[i]) for i in ids]
        stopped = len(history) == 3 and all(
            max(h[i] for h in history) - min(h[i] for h in history) <= sr.STOPPED_COUNTS for i in ids)
        if step['kind'] == 'close' and t >= prof['T'] and stopped:
            break
        if max(err) <= sr.ARRIVE_TOL:
            break
        if t >= prof['T'] and stopped and all(stall_ok(i + 1, fb[i], target[i]) for i in ids):
            stalled = {i + 1: fb[i] - target[i] for i in ids if abs(fb[i] - target[i]) > sr.ARRIVE_TOL}
            emit('stall_accepted', name=step['name'], short=stalled)
            break
        if t >= prof['T'] + sr.ARRIVE_TIMEOUT:
            sr.hold_here(arm, transport, fb, emit, grip_goal)
            raise sr.SmoothAbort('%s：%.1fs内未到位(误差%s)，已写入原地保持' % (step['name'], t, err))
    dwell = sr.GRIP_DWELL.get(step['kind'], 0.0)
    if dwell:
        sleep(dwell)
    elapsed = clock() - t0
    emit('step_done', name=step['name'], elapsed=round(elapsed, 3), feedback=fb)
    return dict(name=step['name'], planned_T=round(prof['T'], 3), elapsed=round(elapsed, 3),
                feedback=fb, target=target, short=stalled)


def drive(arm, transport, steps, start, profile, emit, cancelled, limits, on_done=lambda d: None):
    """Run steps in order, planning each segment from the measured pose (so a joint accepted short
    of its target starts the next segment from where it really is).

    Carrying (S4..S6): ID6 is not commanded; its box expects the measured close stall and
    hold-on-fault re-writes the squeezed close goal.
    """
    vmax, amax = sr.PROFILES[profile]
    current = start[:]
    stall6 = None
    for step in steps:
        target = step['target'][:]
        if target[3] > ID4_REACH:
            emit('id4_clamped', name=step['name'], recorded=target[3], commanded=ID4_REACH)
            target[3] = ID4_REACH
        box_start = current[:]
        if step['carry']:
            if stall6 is None:
                raise r.SafetyError('缺少本篮合爪后的ID6反馈，拒绝带篮运动')
            box_start[5] = step['target'][5]      # squeezed goal: what hold-on-fault re-writes
            target[5] = stall6
        scale, min_t = CAREFUL_POINTS.get(step['point'], (1.0, 0.0))
        prof = sr.segment_profile(current, target, vmax * scale, amax * scale * scale, min_t, step['axes'])
        done = run_step(arm, transport, dict(step, target=target, start=box_start, profile=prof),
                        emit, cancelled, limits)
        if not done:
            continue
        current = done['feedback'][:]
        if step['kind'] == 'close':
            stall6 = current[5]
        elif step['kind'] == 'open':
            stall6 = None
        on_done(done)
    return current


def plan_motion(steps, start, profile):
    """Attach synchronized-trapezoid profiles; ID6 is left out while carrying so its goal stays squeezed."""
    vmax, amax = sr.PROFILES[profile]
    current = start[:]
    out = []
    for step in steps:
        scale, min_t = CAREFUL_POINTS.get(step['point'], (1.0, 0.0))
        prof = sr.segment_profile(current, step['target'], vmax * scale, amax * scale * scale,
                                  min_t, step['axes'])
        out.append(dict(step, profile=prof, start=current[:]))
        current = [step['target'][i] if (step['axes'] is None or i + 1 in step['axes']) else current[i]
                   for i in range(6)]
    return out


def start_steps(steps, start, routes, from_s8):
    """The batch begins at P0. From a route's S8 a direct S8->P0 move is prepended only with from_s8."""
    gap = lambda ref: max(abs(x - y) for x, y in zip(start, ref))
    p0 = steps[0]['target']
    if gap(p0) <= START_TOL:
        return steps
    def s8_gap(doc):   # recorded S8 as actually reachable (ID4 clamp); ID6 may rest short when open
        ref = seq.goal_of(doc['points'][7])
        ref[3] = min(ref[3], ID4_REACH)
        return max(abs(x - y) if i != 5 else max(0, abs(x - y) - STALL_TOL)
                   for i, (x, y) in enumerate(zip(start, ref)))
    near = [(k, s8_gap(doc)) for k, doc in routes.items()]
    k, d = min(near, key=lambda kd: kd[1])
    if d <= START_TOL and from_s8:
        return [dict(name=f'{k[0]}→{k[1]} S8→P0', route=k, point=0, kind='move', axes=None,
                     carry=False, target=p0[:])] + steps[1:]
    if d <= START_TOL:
        raise r.SafetyError(f'当前停在{k[0]}→{k[1]} S8，不在P0；确认S8→P0安全后加 --from-s8，或用上位机回P0')
    raise r.SafetyError(f'起点偏离P0 {gap(p0)}刻度，也不在任何S8；不自动追赶')


def describe(planned):
    rows = [f"{s['name']:<14} {s['kind']:<5} T={s['profile']['T']:.2f}s" for s in planned]
    total = sum(s['profile']['T'] + sr.GRIP_DWELL.get(s['kind'], 0) for s in planned)
    rows.append(f'估计运动+夹爪停留约 {total:.1f}s（另加每段到位判定）')
    return '\n'.join(rows)


def run_batch(a, colors):
    home, routes, pins = seq.load_routes(a.root)
    steps = seq.compile_plan(colors, home, routes)
    lease, lease_sha, expires = r.require_device_lease(a.port, holder=a.holder)
    limits = r.load_limits()
    now = datetime.now(timezone(timedelta(hours=8)))
    logs = getattr(a, 'logs', None) or a.root / 'logs'
    run = logs / ('run_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    result = dict(command='-'.join(colors), mapping=dict(zip(c.TARGETS, colors)), port=a.port,
                  profile=a.profile, loaded=a.loaded, pins=pins, steps=[], controller_complete=False,
                  operator_confirmed=False, physical_stop_verified=False, collision_checked=False)

    def save():
        (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def cancelled():
        try:
            return ((run / 'STOP').exists() or datetime.now(timezone.utc) >= expires
                    or r.sha(lease) != lease_sha or any(c.sha(p) != h for p, h in pins.items()))
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
        arm.gate.min_period_s = sr.PACING_S
        try:
            arm.connect_serial(a.port)
            live = r.configuration(arm)
            result['configuration'] = live
            if not r.same_configuration(home['configuration'], live):
                # 2026-10-13 用户要求：配置与P0采集时不同不再拦截，只记录并提示
                diff = {k: dict(p0=home['configuration'].get(k), live=live.get(k))
                        for k in set(live) | set(home['configuration'])
                        if json.dumps(live.get(k), sort_keys=True) != json.dumps(home['configuration'].get(k), sort_keys=True)}
                result['configuration_diff'] = diff
                print('注意：设备配置与P0采集时不同（仅记录，不拦截）：'
                      + json.dumps(diff, ensure_ascii=False), flush=True)
            battery = arm.get_battery_mv()
            if battery is None:
                raise r.SafetyError('缺少供电报告')
            start, _ = r.capture_stable(arm, emit)
            steps_run = start_steps(steps, start, routes, a.from_s8)
            planned = plan_motion(steps_run, start, a.profile)      # time estimates for the record
            result.update(battery_mv=battery, start=start,
                          planned=[dict(name=s['name'], T=round(s['profile']['T'], 3)) for s in planned])
            save()
            arm.enable_motion(True)
            t_run = time.monotonic()

            def on_done(done):
                result['steps'].append(done)
                save()
                short = f" 未到位已接受{done['short']}" if done['short'] else ''
                print(f"STEP_DONE={done['name']} {done['elapsed']}s{short}", flush=True)
            drive(arm, transport, steps_run, start, a.profile, emit, cancelled, limits, on_done)
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
        answer = input('OBSERVE: 串口已关闭。yes=三篮均正确放到P1/P2/P3、无掉篮无碰撞: ').strip().lower()
        result.update(operator_answer=answer, operator_confirmed=answer == 'yes')
        save()
    print('RESULT=' + json.dumps(dict(command=result['command'], complete=result['operator_confirmed'],
                                      error=result.get('error'), total_s=result.get('total_s'),
                                      result_file=str(run / 'result.json')), ensure_ascii=False), flush=True)
    return result


def drain_typeahead():
    """BUSY rule: commands typed while a batch ran are discarded, never queued."""
    try:
        import msvcrt
    except ImportError:
        return 0
    n = 0
    while msvcrt.kbhit():
        msvcrt.getwch()
        n += 1
    return n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', nargs='?', help='颜色顺序，例如 红-蓝-黄；省略则进入交互模式')
    ap.add_argument('--root', type=Path, default=c.ROOT)
    ap.add_argument('--port', choices=('COM11',))
    ap.add_argument('--holder', default='claude', choices=('claude', 'codex'))
    ap.add_argument('--go', action='store_true')
    load = ap.add_mutually_exclusive_group()
    load.add_argument('--empty-load', action='store_true')
    load.add_argument('--loaded', action='store_true', help='带篮运行，仅限gentle')
    ap.add_argument('--profile', default='gentle', choices=sorted(sr.PROFILES))
    ap.add_argument('--from-s8', action='store_true', help='允许从上一批S8直接回P0再开始')
    ap.add_argument('--dry-run', action='store_true', help='只打印时间规划（假定从P0开始），不连串口')
    a = ap.parse_args(argv)
    try:
        if a.dry_run:
            colors = seq.parse_command(a.command or '')
            home, routes, _ = seq.load_routes(a.root)
            steps = seq.compile_plan(colors, home, routes)
            print(describe(plan_motion(steps, steps[0]['target'], a.profile)))
            return 0
        if not (a.port and a.go and (a.empty_load or a.loaded)):
            ap.error('运行需要 --port COM11 --go 以及 --empty-load 或 --loaded')
        if a.loaded and a.profile != sr.LOADED_PROFILE:
            ap.error('带篮运行仅限 --profile %s' % sr.LOADED_PROFILE)
        if a.command:
            res = run_batch(a, seq.parse_command(a.command))
            return 0 if res['operator_confirmed'] else 2
        print('交互模式：每次输入一组颜色（例如 红-蓝-黄），输入 q 退出。空闲时输入新指令即视为场地已复位。')
        while True:
            text = input('指令> ').strip()
            if text.lower() in ('q', 'quit', 'exit'):
                return 0
            try:
                colors = seq.parse_command(text)
            except ValueError as exc:
                print('拒绝：' + str(exc))
                continue
            res = run_batch(a, colors)
            dropped = drain_typeahead()
            if dropped:
                print(f'BUSY：执行期间的{dropped}个按键已丢弃，请重新输入')
            if res.get('error'):
                print('本批未完成，交互模式退出；先核对现场和姿态再重启')
                return 2
    except (ValueError, OSError, r.SafetyError) as exc:
        print('ERROR=' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
