"""One supervised empty-load cycle. Settled waypoints, no smooth blending/retry."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

import reposition_empty as setup

r = setup.r


def validate_plan(plan, pins, targets, limits):
    if (plan.get('schema') != 'nexarm.empty-cycle/1'
            or plan.get('empty_load_only') is not True
            or plan.get('sequence') != list(range(1, 9))
            or plan.get('cycles') != 1
            or plan.get('preparation') not in ('base-return', 'approach-p1', 'accepted-p1')
            or plan.get('settle_tolerances') != [20]*6
            or plan.get('pins') != pins):
        raise r.SafetyError('整程计划范围或文件绑定不匹配')
    start = r.check_counts(plan.get('expected_start'), limits, '确认的P8起点')
    reference = targets[7][:]
    if plan['preparation'] == 'approach-p1':
        reference[0] = targets[0][0]
    start_limit = 20
    if plan['preparation'] == 'accepted-p1':
        if plan.get('operator_accepted_p1') is not True or not plan.get('start_evidence'):
            raise r.SafetyError('必须有实体P1接受记录及实测证据')
        reference = targets[0]
        start_limit = 24  # Start acceptance only; all movement goals stay at 20.
    if max(abs(a-b) for a,b in zip(start, reference)) > start_limit:
        raise r.SafetyError('准备阶段必须从已确认的P8或高位回转终点附近开始')
    return start


def run_cycle(arm, expected, targets, emit, cancelled, route_sha, limits, limits_sha, on_stage,
              preparation='base-return'):
    # Reuse the physically checked high P8 base-only transit before approaching P1.
    transit = expected[:]
    transit[0] = targets[0][0]
    if cancelled():
        raise r.SafetyError('执行前已取消')
    if preparation not in ('base-return', 'approach-p1', 'accepted-p1'):
        raise r.SafetyError('未知准备阶段')
    current = expected[:]
    if preparation == 'base-return':
        report = r.execute_segment(arm, expected, transit, emit, cancelled,
            source=r.SOURCE, expected_sha=route_sha, limits=limits, limits_sha=limits_sha,
            active_axes=(1,), tolerances=[20]*6)
        on_stage('prepare-base', report)
        current = report['returned_report']
    if preparation == 'accepted-p1':
        actual, _ = r.capture_stable(arm, emit)
        if max(abs(a-b) for a,b in zip(actual, expected)) > 12:
            raise r.SafetyError('已接受P1起点改变，不自动追赶')
        on_stage('accepted-point-1',dict(returned_report=actual,recorded_target=targets[0],
            controller_converged=False,operator_confirmed=True,start_only_acceptance=True))
        current = actual
    for point in range(2 if preparation == 'accepted-p1' else 1, 9):
        if cancelled():
            raise r.SafetyError('停止后续阶段')
        report = setup.execute_forward_segment(arm, current, targets[point-1], emit,
            cancelled, route_sha, limits, limits_sha, [20]*6, point in (3, 7))
        on_stage('point-' + str(point), report)
        current = report['returned_report']
    return current


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--plan', required=True, type=Path)
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--empty-load', action='store_true')
    a = ap.parse_args()
    if not a.go or not a.empty_load:
        ap.error('必须显式确认单次空载运行')
    files = dict(route=r.SOURCE, limits=r.DEFAULT_LIMITS, hold=r.DIAGNOSTIC_HOLD,
                 runner=Path(__file__), setup=Path(setup.__file__), replay=Path(r.__file__))
    pins = {k:r.sha(p) for k,p in files.items()}
    route_raw = r.SOURCE.read_bytes()
    route = json.loads(route_raw.decode('utf-8'))
    limits = r.load_limits()
    targets = r.resolve_route(route, limits)
    r.check_binding(route, limits)
    if route.get('require_powered_sliders') is not True:
        raise r.SafetyError('只接受主动示教路线')
    plan = json.loads(a.plan.read_text(encoding='utf-8'))
    plan_sha = r.sha(a.plan)
    expected = validate_plan(plan, pins, targets, limits)
    lease, lease_sha, expires = r.require_device_lease(a.port, holder='codex')
    run = r.HERE/'logs'/('cycle_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run/'route_snapshot.json').write_bytes(route_raw)
    (run/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    for key in ('runner','setup','replay'):
        (run/files[key].name).write_bytes(files[key].read_bytes())
    result = dict(plan=plan, port=a.port, stages=[], controller_complete=False,
                  operator_confirmed=False, complete=False, empty_load=True,
                  physical_stop_verified=False, collision_checked=False, smooth_blending=False)
    def save():
        (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    def cancelled():
        try:
            return ((run/'STOP').exists() or datetime.now(timezone.utc)>=expires
                or r.sha(lease)!=lease_sha or r.sha(a.plan)!=plan_sha
                or any(r.sha(p)!=pins[k] for k,p in files.items()))
        except OSError:
            return True
    save()
    print('RUN_DIR='+str(run),flush=True)
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        mutex=threading.Lock()
        def emit(event,**data):
            with mutex:
                log.write(json.dumps(dict(event=event,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        def on_stage(name, report):
            result['stages'].append(dict(name=name, **report))
            save()
            print('STAGE_DONE='+json.dumps(dict(name=name,actual=report['returned_report'])),flush=True)
        transport=r.ReplayTransport(emit,cancelled,limits)
        arm=r.ArmController(transport=transport,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial(a.port)
            live=setup.configuration(arm)
            if not setup.same_configuration(route['configuration'],live):
                raise r.SafetyError('设备配置改变')
            battery=arm.get_battery_mv()
            if battery is None:
                raise r.SafetyError('缺少供电报告')
            result.update(configuration=live,battery_mv=battery)
            arm.enable_motion(True)
            result['final_counts']=run_cycle(arm,expected,targets,emit,cancelled,
                pins['route'],limits,pins['limits'],on_stage,preparation=plan['preparation'])
            result['controller_complete']=True
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'])
            print('STOP='+result['error'],flush=True)
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(serial_closed=True,motion_attempts=transport.motion_attempts,stats=transport.stats())
            save()
    if result['controller_complete']:
        answer=setup.ask_observation('OBSERVE: 串口已关闭。yes=整程实体动作正确且停稳无异常\n')
        result.update(operator_answer=answer,operator_confirmed=answer=='yes',complete=answer=='yes')
        save()
    print('RESULT='+json.dumps(dict(complete=result['complete'],error=result.get('error'),
        result_file=str(run/'result.json')),ensure_ascii=False),flush=True)
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
