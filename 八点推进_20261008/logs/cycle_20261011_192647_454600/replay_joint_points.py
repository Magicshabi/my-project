"""Empty-load, point-by-point validation using bounded individual CMD3 writes.

Joint interpolation is not a Cartesian/collision guarantee or synchronous servo
start. STOP prevents further sends; physical emergency stop is not verified.
"""
import argparse
from collections import deque
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
LIB = HERE.parent / '舵机基础运行逻辑'
sys.path.insert(0, str(LIB))
from nexarm import ArmController, Transport, SafetyError, build_frame
from nexarm.commands import pack_joint_counts
from nexarm.eight_point import STAGES, configuration, same_configuration
from capture_joint_points import checked_counts, capture_stable
from joint_limits import (DEFAULT_LIMITS, check_counts, check_axis, check_binding,
                          load_limits, snapshot)

SOURCE = HERE / 'data/eight_joints_powered_20261009.json'
DIAGNOSTIC_HOLD = HERE / 'replay_diagnostic_hold.json'
EXPECTED_SHA = 'b39780d98da8557dde8e3b2096bf32c400e5678431b54ec38e9a396742b18a8d'
MAX_STEP = 40
TOLERANCE = 10
DEFAULT_POINTS = list(range(1, 9))  # Offline test coverage, never an automatic run.
LOCKS = HERE.parent / 'handoff/locks'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_trial_plan(path, source_sha, limits_sha, points):
    plan=json.loads(Path(path).read_text(encoding='utf-8'))
    if (plan.get('schema') != 'nexarm.empty-segment-trial/1'
            or plan.get('source_sha256') != source_sha
            or plan.get('limits_sha256') != limits_sha
            or plan.get('empty_load_only') is not True
            or points not in plan.get('allowed_segments', [])
            or plan.get('historical_hold_sha256') != sha(DIAGNOSTIC_HOLD)):
        raise SafetyError('单段试验计划不匹配；不解除历史故障锁')
    return plan


LEASE_HOLDER = 'claude'  # 2026-10-11: Claude took over the former Codex test role.


def require_device_lease(port, holder=LEASE_HOLDER):
    if not port.upper().startswith('COM') or not port[3:].isdigit():
        raise SafetyError('需要明确的Windows COM端口')
    path=LOCKS/(port.upper()+'.lock')
    try:
        lease=json.loads(path.read_text(encoding='utf-8'))
        since=datetime.fromisoformat(lease['since'])
        expires=since+timedelta(minutes=lease['ttl_min'])
        if (lease.get('holder') != holder or lease.get('device') != port.upper()
                or since.tzinfo is None or datetime.now(timezone.utc) >= expires):
            raise ValueError('wrong owner or expired')
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise SafetyError('先通过handoff/lock.py登记有效的%s设备锁'%holder) from exc
    return path,sha(path),expires


def resolve_route(route, limits=None):
    limits = load_limits() if limits is None else limits
    rows = route.get('points', [])
    if (route.get('schema') != 'nexarm.joint-eight/1' or route.get('complete') is not True
            or route.get('units') != 'raw_encoder_counts'
            or route.get('servo_ids') != list(range(1, 7))
            or not isinstance(rows, list) or len(rows) != 8):
        raise SafetyError('需要完整八点关节记录')
    targets = []
    for i,row in enumerate(rows):
        if row.get('point') != i+1 or row.get('stage') != STAGES[i] or not row.get('physical_pose_confirmed'):
            raise SafetyError('记录顺序或原始姿态确认无效')
        raw = checked_counts(row.get('encoder_positions'))
        target = checked_counts(row.get('target_encoder_positions', raw))
        if target != raw and (i != 2 or target[:5] != raw[:5] or target[5] != rows[3]['encoder_positions'][5]):
            raise SafetyError('仅允许已声明的第3点夹爪修正')
        check_counts(target, limits, '第%d点执行目标' % (i+1))
        if route.get('require_powered_sliders') and row.get('motor_driven_pose_confirmed_by_operator') is not True:
            raise SafetyError('第%d点缺少电机主动到达确认' % (i+1))
        targets.append(target)
    return targets


def check_sequence(points, *, forward_trial=False):
    if len(points) < 2 or any(type(p) is not int or not 1<=p<=8 for p in points):
        raise SafetyError('点序列无效')
    if any(abs(a-b) != 1 for a,b in zip(points,points[1:])):
        raise SafetyError('首轮仅允许相邻点，不跨点或自动回零')
    if forward_trial and (len(points) != 2 or points[1] != points[0]+1):
        raise SafetyError('首轮每次仅执行一个正向相邻段，不反向复演夹持事件')


def interpolate(start, target, limits=None):
    limits = load_limits() if limits is None else limits
    check_counts(start, limits, '插值起点')
    check_counts(target, limits, '插值终点')
    n = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,target)) / MAX_STEP))
    return [[round(a+(b-a)*k/n) for a,b in zip(start,target)] for k in range(1,n+1)]


def reads(arm, limits=None):
    limits = load_limits() if limits is None else limits
    p = arm.get_pose()
    j = arm.get_real_joints()
    if p is None or j is None or len(j) != 6:
        raise SafetyError('位置回包缺失；不补发、不自动返回')
    return (check_counts(p.servos, limits, 'CMD11反馈'),
            check_counts([x[0] for x in j], limits, 'CMD65反馈'))


class ReplayTransport(Transport):
    def __init__(self, emit, cancelled, limits=None):
        super().__init__(log=lambda msg: emit('transport', message=msg))
        self.emit, self.cancelled = emit, cancelled
        self.pending = deque()
        self.failed = False
        self.motion_attempts = 0
        self.limits = load_limits() if limits is None else limits

    def send(self, cmd, args=b'', target_id=255):
        query = target_id == 255 and ((cmd in (1,2,11,65,80,64) and not args)
                                    or (cmd == 58 and len(args)==1 and 1<=args[0]<=6))
        wire = build_frame(cmd,args,target_id)
        if self.failed or self.cancelled():
            raise SafetyError('已请求停止或通信失败，不再下发')
        if not query:
            if cmd != 3 or len(args) != 8 or args[0] != 41:
                raise SafetyError('仅允许已验证的CMD3/reg41位置写入')
            check_axis(target_id, int.from_bytes(args[2:4], 'little'), self.limits)
            if not self.pending or wire != self.pending[0]:
                raise SafetyError('运动不在当前小步白名单内')
            self.pending.popleft()
            self.motion_attempts += 1
        self.emit('tx_attempt', cmd=cmd, id=target_id, wire_hex=wire.hex())
        try:
            super().send(cmd,args,target_id)
        except BaseException:
            self.failed = True
            self.pending.clear()
            raise
        self.emit('tx_complete', cmd=cmd, id=target_id)


def wait_step(arm, start, goal, emit, cancelled, timeout=6, limits=None, tolerances=None):
    limits = load_limits() if limits is None else limits
    tolerances = [TOLERANCE]*6 if tolerances is None else tolerances
    if (not isinstance(tolerances,(list,tuple)) or len(tolerances)!=6
            or any(type(v) is not int or not 1<=v<=20 for v in tolerances)):
        raise SafetyError('到位容差需为六轴1..20刻度的明确值')
    begun = time.monotonic()
    stable = []
    while time.monotonic()-begun < timeout:
        if cancelled():
            raise SafetyError('停止后续写入；不保证当前物理运动已停止')
        a,b = reads(arm, limits)
        emit('sample', cmd11=a, cmd65=b, goal=goal)
        if arm.transport.stats()['checksum_errors']:
            raise SafetyError('检测到坏帧，停止后续写入')
        for values in (a,b):
            if any(not min(start[i],goal[i])-16 <= values[i] <= max(start[i],goal[i])+16 for i in range(6)):
                raise SafetyError('反馈偏离当前小步区间，不再推进')
        if (time.monotonic()-begun >= .5
            and max(abs(x-y) for x,y in zip(a,b)) <= 6
            and all(abs(x-y)<=t for x,y,t in zip(a,goal,tolerances))
            and all(abs(x-y)<=t for x,y,t in zip(b,goal,tolerances))):
            stable.append((a,b)); stable=stable[-4:]
            if len(stable)==4 and all(max(s[j][i] for s in stable)-min(s[j][i] for s in stable)<=4
                                     for j in (0,1) for i in range(6)):
                return b
        else:
            stable=[]
        time.sleep(.12)
    raise TimeoutError('小步位置未稳定到位；不重试、不自动回位')


def execute_segment(arm, expected_start, target, emit, cancelled, *,
                    source=SOURCE, expected_sha=EXPECTED_SHA, limits=None, limits_sha=None,
                    active_axes=None):
    limits = load_limits() if limits is None else limits
    check_counts(expected_start, limits, '计划起点')
    check_counts(target, limits, '计划终点')
    before,samples = capture_stable(arm, emit)
    check_counts(before, limits, '当前起点')
    if max(abs(a-b) for a,b in zip(before,expected_start)) > 12:
        raise SafetyError('当前起点偏离对应示教点；不自动追赶')
    goals = interpolate(before, target, limits)
    actual = before
    for index,goal in enumerate(goals):
        if cancelled():
            raise SafetyError('已请求停止')
        if sha(source) != expected_sha:
            raise SafetyError('运行中点位文件被更改，停止')
        if limits_sha is not None and sha(DEFAULT_LIMITS) != limits_sha:
            raise SafetyError('运行中限位文件被更改，停止')
        check_counts(goal, limits, '当前小步目标')
        if max(abs(a-b) for a,b in zip(actual,goal)) > MAX_STEP+12:
            raise SafetyError('单步目标相对反馈过大')
        moves = [(i+1,v) for i,v in enumerate(goal) if v != actual[i]
                 and (active_axes is None or i+1 in active_axes)]
        arm.transport.pending=deque(build_frame(3,pack_joint_counts(v,100,2000),sid) for sid,v in moves)
        emit('step_begin', index=index+1, count=len(goals), start=actual, target=goal)
        for sid,value in moves:
            if cancelled():
                raise SafetyError('停止发送剩余关节')
            arm.move_joint_counts(sid,value,100,2000)
        if arm.transport.pending:
            raise SafetyError('当前小步存在未发送帧')
        actual=wait_step(arm,actual,goal,emit,cancelled,limits=limits)
        emit('step_converged', index=index+1, positions=actual)
    return dict(start=before, target=target, returned_report=actual, steps=len(goals),
                controller_converged=True, operator_confirmed=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, help='显式指定已核对端口，当前记录COM11')
    ap.add_argument('--file', type=Path, default=SOURCE)
    ap.add_argument('--expected-sha', required=True, help='本次经审核文件的SHA256')
    ap.add_argument('--points',type=int,nargs=2,required=True,help='单个正向段，例如1 2')
    ap.add_argument('--go',action='store_true')
    ap.add_argument('--empty-load',action='store_true')
    ap.add_argument('--trial-plan',type=Path,help='针对新路线的限定空载单段试验计划')
    a=ap.parse_args()
    check_sequence(a.points, forward_trial=True)
    if not a.go or not a.empty_load:
        ap.error('首次验证必须 --go --empty-load，并现场确认空载清障')
    if DIAGNOSTIC_HOLD.exists() and a.trial_plan is None:
        raise SafetyError('旧ID4故障回放入口仍暂停；用户已报告舵机修复，需核验新点位与路径后再更新执行入口。')
    expected_sha = a.expected_sha
    source_bytes = a.file.read_bytes()
    if expected_sha is None or hashlib.sha256(source_bytes).hexdigest() != expected_sha:
        raise SafetyError('点位文件指纹改变，先核对')
    route=json.loads(source_bytes.decode('utf-8'))
    limits=load_limits()
    limits_sha=sha(DEFAULT_LIMITS)
    targets=resolve_route(route,limits)
    check_binding(route,limits)
    trial_plan=None
    if a.trial_plan is not None:
        trial_plan=validate_trial_plan(a.trial_plan,expected_sha,limits_sha,a.points)
    lease_path,lease_sha,lease_expires=require_device_lease(a.port)
    trial_sha=sha(a.trial_plan) if a.trial_plan is not None else None
    now=datetime.now(timezone(timedelta(hours=8)))
    run=HERE/'logs'/('replay_'+now.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run/'route_snapshot.json').write_bytes(source_bytes)
    print('RUN_DIR='+str(run),flush=True)
    result=dict(started_at=now.isoformat(),source_sha256=expected_sha,points=a.points,
                source_file=str(a.file.resolve()),host_joint_limits=snapshot(limits),limits_sha256=limits_sha,
                empty_load=True,complete=False,segments=[],physical_stop_verified=False,
                synchronized_bus_write=False,limits_note='bounded joint increments are not collision proof',
                trial_plan=trial_plan,port=a.port,device_lease_sha256=lease_sha,
                sources={str(p):sha(p) for p in (Path(__file__),HERE/'capture_joint_points.py',HERE/'joint_limits.py',DEFAULT_LIMITS,LIB/'nexarm/arm.py',LIB/'nexarm/commands.py',LIB/'nexarm/transport.py')})
    def save():
        (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    def cancelled():
        try:
            return ((run/'STOP').exists() or datetime.now(timezone.utc)>=lease_expires
                    or sha(lease_path)!=lease_sha
                    or (a.trial_plan is not None and (sha(a.trial_plan)!=trial_sha
                        or sha(DIAGNOSTIC_HOLD)!=trial_plan['historical_hold_sha256'])))
        except OSError:
            return True
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            with lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        transport=ReplayTransport(emit,cancelled,limits)
        transport.on_any(lambda f:emit('rx',id=f.id,cmd=f.cmd,args=f.args.hex()))
        arm=ArmController(transport=transport,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial(a.port)
            live=configuration(arm)
            if not same_configuration(route['configuration'],live):
                raise SafetyError('设备配置与采集时不同')
            battery=arm.get_battery_mv()
            if battery is None:
                raise SafetyError('缺少供电报告')
            baseline,_=capture_stable(arm,emit)
            check_counts(baseline,limits,'执行前当前反馈')
            result.update(baseline=baseline,configuration=live,battery_mv=battery)
            save();print('BASELINE='+json.dumps(baseline),flush=True)
            if transport.stats()['checksum_errors']:
                raise SafetyError('预检存在坏帧')
            arm.enable_motion(True)
            for start,end in zip(a.points,a.points[1:]):
                print('SEGMENT %d -> %d'%(start,end),flush=True)
                entry=execute_segment(arm,targets[start-1],targets[end-1],emit,cancelled,
                                      source=a.file,expected_sha=expected_sha,limits=limits,limits_sha=limits_sha)
                entry.update(from_point=start,to_point=end)
                result['segments'].append(entry);save()
                print('POINT_DONE='+json.dumps(entry),flush=True)
                try:
                    answer=input('OBSERVE_POINT%d: yes=现场实际到位且无异常; 其他输入停止\n'%end).strip()
                except EOFError:
                    answer='no observation'
                entry.update(operator_answer=answer,operator_confirmed=answer=='yes')
                emit('operator_observation',from_point=start,to_point=end,answer=answer);save()
                if answer!='yes':
                    raise SafetyError('现场未确认，不继续下一段')
            result['complete']=True
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'],physical_stop_verified=False)
            print('STOP: 已停止后续写入。若仍运动或有异常，请切断舵机电源。',flush=True)
            raise
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(motion_attempts=transport.motion_attempts,stats=transport.stats(),serial_closed=True)
            save()
            print('RESULT='+json.dumps(dict(complete=result['complete'],
                  error=result.get('error'),motion_attempts=transport.motion_attempts,
                  serial_closed=True,result_file=str(run/'result.json')),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
