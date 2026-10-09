"""One ID4 comparison toward center, operator pause, then planned return.

This is not limit expansion or eight-point replay. The route diagnostic hold
stays in place. No automatic return on failed feedback or operator observation.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time

from capture_joint_points import capture_stable, timestamp, digest
from inspect_id4_registers import RegisterQueries
from replay_joint_points import ReplayTransport, wait_step
from nexarm import ArmController, SafetyError, build_frame
from nexarm.commands import pack_joint_counts
from nexarm.eight_point import configuration, same_configuration

ROOT = Path(__file__).resolve().parent
EXPECTED = [2358, 1555, 2859, 3090, 2414, 1251]


class ProbeTransport(ReplayTransport):
    open_serial = RegisterQueries.open_serial


def plan(base):
    if len(base) != 6 or any(type(v) is not int or abs(v-e)>8 for v,e in zip(base,EXPECTED)):
        raise SafetyError('当前姿态偏离已复查基线，零动作退出')
    goal = base[:]
    goal[3] -= 150
    # A test envelope, not a claim about the installed servo's real limits.
    if not 2900 <= goal[3] < base[3] <= 3100:
        raise SafetyError('超出本次已评估的单轴对照范围')
    return goal


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--go', action='store_true')
    a = ap.parse_args()
    if not a.go:
        ap.error('需要 --go；仅执行已告知用户的空载ID4对照')
    run = ROOT/'logs'/('id4_lower_range_'+str(time.time_ns()))
    run.mkdir(parents=True)
    print('RUN_DIR='+str(run),flush=True)
    result = dict(started_at=timestamp(),complete=False,operator_confirmed=False,
                  limit_writes=0,torque_writes=0,physical_stop_verified=False,
                  source_sha256=digest(__file__))
    def save():
        (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    cancelled = lambda: (run/'STOP').exists()
    lock = threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(event,**data):
            with lock:
                log.write(json.dumps(dict(event=event,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        transport = ProbeTransport(emit,cancelled)
        transport.on_any(lambda f:emit('rx',id=f.id,cmd=f.cmd,args_hex=f.args.hex()))
        arm = ArmController(transport=transport,log=lambda s:emit('library',message=s))
        try:
            route = json.loads((ROOT/'data/eight_joints_20261009.json').read_text(encoding='utf-8'))
            arm.connect_serial('COM9')
            config = configuration(arm)
            if not same_configuration(config,route['configuration']):
                raise SafetyError('配置变化，零动作退出')
            base,_ = capture_stable(arm,emit)
            goal = plan(base)
            if transport.stats()['checksum_errors']:
                raise SafetyError('预检出现坏帧，零动作退出')
            result.update(configuration=config,baseline=base,target=goal)
            save()
            print('PLAN='+json.dumps(dict(from_counts=base,to_counts=goal,id=4,acc=100,speed=2000)),flush=True)
            transport.pending = deque([build_frame(3,pack_joint_counts(goal[3],100,2000),4)])
            arm.enable_motion(True)
            arm.move_joint_counts(4,goal[3],100,2000)
            at_goal = wait_step(arm,base,goal,emit,cancelled)
            result['at_target'] = at_goal
            save()
            print('AT_TARGET='+json.dumps(at_goal),flush=True)
            observed = input('OBSERVE_OUTWARD: yes=ID4实体转动且已静止无异常，才允许返回; 其他=停止\n').strip()
            result['outward_operator_answer'] = observed
            save()
            if observed != 'yes':
                raise SafetyError('现场未确认，停止且不自动回位')
            fresh,_ = capture_stable(arm,emit)
            if max(abs(v-g) for v,g in zip(fresh,goal))>10 or cancelled():
                raise SafetyError('暂停后姿态变化或已停止，不返回')
            transport.pending = deque([build_frame(3,pack_joint_counts(base[3],100,2000),4)])
            arm.move_joint_counts(4,base[3],100,2000)
            returned = wait_step(arm,fresh,base,emit,cancelled)
            result.update(returned=returned,controller_round_trip=True)
            save()
            print('RETURNED='+json.dumps(returned),flush=True)
            observed = input('OBSERVE_RETURN: yes=实体恢复且无异常; 其他=未验证\n').strip()
            result.update(return_operator_answer=observed,operator_confirmed=observed=='yes',complete=observed=='yes')
            save()
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'])
            print('ABORTED='+result['error'],flush=True)
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(motion_attempts=transport.motion_attempts,stats=transport.stats(),serial_closed=True)
            save()
            print('RESULT='+json.dumps(result,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
