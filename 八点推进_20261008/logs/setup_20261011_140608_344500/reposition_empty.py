"""Supervised empty-load return from P8 via a high base-only transit to P1.

This is a setup experiment, not a collision-checked default/home pose.
Each invocation performs one leg and waits for physical observation.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import replay_joint_points as r
from nexarm.eight_point import configuration, same_configuration


def targets_for_leg(targets, leg):
    start, finish = targets[7][:], targets[0][:]
    transit = start[:]
    transit[0] = finish[0]
    if leg == 'base':
        return start, transit
    if leg == 'approach':
        return transit, finish
    raise r.SafetyError('未知回位阶段')


def validate_setup(plan, route_sha, limits_sha, leg):
    if (plan.get('schema') != 'nexarm.empty-setup-trial/1'
            or plan.get('source_sha256') != route_sha
            or plan.get('limits_sha256') != limits_sha
            or plan.get('empty_load_only') is not True
            or leg not in plan.get('allowed_legs', [])
            or plan.get('historical_hold_sha256') != r.sha(r.DIAGNOSTIC_HOLD)):
        raise r.SafetyError('回位试验计划不匹配')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True)
    ap.add_argument('--leg',required=True,choices=['base','approach'])
    ap.add_argument('--plan',required=True,type=Path)
    ap.add_argument('--go',action='store_true')
    ap.add_argument('--empty-load',action='store_true')
    a=ap.parse_args()
    if not (a.go and a.empty_load): ap.error('需现场确认空载清障并显式--go --empty-load')
    raw=r.SOURCE.read_bytes(); route_sha=r.hashlib.sha256(raw).hexdigest()
    route=json.loads(raw.decode('utf-8'));limits=r.load_limits();limits_sha=r.sha(r.DEFAULT_LIMITS)
    plan=json.loads(a.plan.read_text(encoding='utf-8'));plan_sha=r.sha(a.plan)
    validate_setup(plan,route_sha,limits_sha,a.leg)
    if route.get('require_powered_sliders') is not True: raise r.SafetyError('只接受主动示教路线')
    targets=r.resolve_route(route,limits);r.check_binding(route,limits)
    expected,target=targets_for_leg(targets,a.leg)
    lease_path,lease_sha,expires=r.require_device_lease(a.port)
    run=r.HERE/'logs'/('setup_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run/'route_snapshot.json').write_bytes(raw)
    for source in (Path(__file__),Path(r.__file__)):
        (run/source.name).write_bytes(source.read_bytes())
    result=dict(leg=a.leg,source_sha256=route_sha,limits_sha256=limits_sha,
                expected_start=expected,target=target,empty_load=True,plan=plan,
                complete=False,operator_confirmed=False,physical_stop_verified=False,
                collision_checked=False,port=a.port)
    def save(): (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    def cancelled():
        try:
            return ((run/'STOP').exists() or datetime.now(timezone.utc)>=expires
                    or r.sha(lease_path)!=lease_sha or r.sha(a.plan)!=plan_sha
                    or r.sha(r.DIAGNOSTIC_HOLD)!=plan['historical_hold_sha256'])
        except OSError: return True
    save();print('RUN_DIR='+str(run),flush=True)
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        log_lock=r.threading.Lock()
        def emit(kind,**data):
            with log_lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n');log.flush()
        transport=r.ReplayTransport(emit,cancelled,limits)
        arm=r.ArmController(transport=transport,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial(a.port)
            live=configuration(arm)
            if not same_configuration(route['configuration'],live): raise r.SafetyError('设备配置改变')
            battery=arm.get_battery_mv()
            if battery is None: raise r.SafetyError('缺少供电报告')
            result.update(configuration=live,battery_mv=battery);save()
            arm.enable_motion(True)
            result['segment']=r.execute_segment(arm,expected,target,emit,cancelled,
                    source=r.SOURCE,expected_sha=route_sha,limits=limits,limits_sha=limits_sha,
                    active_axes=(1,) if a.leg=='base' else None)
            save();print('LEG_DONE='+json.dumps(result['segment']),flush=True)
            answer=input('OBSERVE: yes=实体到位且无异常；其他输入停止\n').strip()
            result.update(operator_answer=answer,operator_confirmed=answer=='yes',complete=answer=='yes')
            emit('operator_observation',answer=answer)
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'])
            print('STOP: 停止后续写入；无自动重试或回位。',flush=True)
        finally:
            transport.pending.clear();arm.enable_motion(False);arm.close()
            result.update(motion_attempts=transport.motion_attempts,serial_closed=True,stats=transport.stats());save()
            print('RESULT='+json.dumps(dict(complete=result['complete'],error=result.get('error'),
                  motion_attempts=transport.motion_attempts,result_file=str(run/'result.json')),ensure_ascii=False),flush=True)
    return 0 if result['complete'] else 2


if __name__=='__main__': raise SystemExit(main())
