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

SOURCE = HERE / 'data/eight_joints_20261009.json'
EXPECTED_SHA = 'b39780d98da8557dde8e3b2096bf32c400e5678431b54ec38e9a396742b18a8d'
MAX_STEP = 40
TOLERANCE = 10
DEFAULT_POINTS = [8,7,6,5,4,3,2,1,2,3,4,5,6,7,8]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve_route(route):
    rows = route.get('points', [])
    if route.get('schema') != 'nexarm.joint-eight/1' or not route.get('complete') or len(rows) != 8:
        raise SafetyError('需要完整八点关节记录')
    targets = []
    for i,row in enumerate(rows):
        if row.get('point') != i+1 or row.get('stage') != STAGES[i] or not row.get('physical_pose_confirmed'):
            raise SafetyError('记录顺序或原始姿态确认无效')
        raw = checked_counts(row.get('encoder_positions'))
        target = checked_counts(row.get('target_encoder_positions', raw))
        if target != raw and (i != 2 or target[:5] != raw[:5] or target[5] != rows[3]['encoder_positions'][5]):
            raise SafetyError('仅允许已声明的第3点夹爪修正')
        targets.append(target)
    return targets


def check_sequence(points):
    if len(points) < 2 or any(type(p) is not int or not 1<=p<=8 for p in points):
        raise SafetyError('点序列无效')
    if any(abs(a-b) != 1 for a,b in zip(points,points[1:])):
        raise SafetyError('首轮仅允许相邻点，不跨点或自动回零')


def interpolate(start, target):
    checked_counts(start); checked_counts(target)
    n = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,target)) / MAX_STEP))
    return [[round(a+(b-a)*k/n) for a,b in zip(start,target)] for k in range(1,n+1)]


def reads(arm):
    p = arm.get_pose()
    j = arm.get_real_joints()
    if p is None or j is None or len(j) != 6:
        raise SafetyError('位置回包缺失；不补发、不自动返回')
    return checked_counts(p.servos), checked_counts([x[0] for x in j])


class ReplayTransport(Transport):
    def __init__(self, emit, cancelled):
        super().__init__(log=lambda msg: emit('transport', message=msg))
        self.emit, self.cancelled = emit, cancelled
        self.pending = deque()
        self.failed = False
        self.motion_attempts = 0

    def send(self, cmd, args=b'', target_id=255):
        query = target_id == 255 and ((cmd in (1,2,11,65,80,64) and not args)
                                    or (cmd == 58 and len(args)==1 and 1<=args[0]<=6))
        wire = build_frame(cmd,args,target_id)
        if self.failed or self.cancelled():
            raise SafetyError('已请求停止或通信失败，不再下发')
        if not query:
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


def wait_step(arm, start, goal, emit, cancelled, timeout=6):
    begun = time.monotonic()
    stable = []
    while time.monotonic()-begun < timeout:
        if cancelled():
            raise SafetyError('停止后续写入；不保证当前物理运动已停止')
        a,b = reads(arm)
        emit('sample', cmd11=a, cmd65=b, goal=goal)
        if arm.transport.stats()['checksum_errors']:
            raise SafetyError('检测到坏帧，停止后续写入')
        for values in (a,b):
            if any(not min(start[i],goal[i])-16 <= values[i] <= max(start[i],goal[i])+16 for i in range(6)):
                raise SafetyError('反馈偏离当前小步区间，不再推进')
        if (time.monotonic()-begun >= .5
            and max(abs(x-y) for x,y in zip(a,b)) <= 6
            and max(abs(x-y) for x,y in zip(a,goal)) <= TOLERANCE
            and max(abs(x-y) for x,y in zip(b,goal)) <= TOLERANCE):
            stable.append((a,b)); stable=stable[-4:]
            if len(stable)==4 and all(max(s[j][i] for s in stable)-min(s[j][i] for s in stable)<=4
                                     for j in (0,1) for i in range(6)):
                return b
        else:
            stable=[]
        time.sleep(.12)
    raise TimeoutError('小步位置未稳定到位；不重试、不自动回位')


def execute_segment(arm, expected_start, target, emit, cancelled):
    before,samples = capture_stable(arm, emit)
    if max(abs(a-b) for a,b in zip(before,expected_start)) > 12:
        raise SafetyError('当前起点偏离对应示教点；不自动追赶')
    goals = interpolate(before, target)
    actual = before
    for index,goal in enumerate(goals):
        if cancelled():
            raise SafetyError('已请求停止')
        if sha(SOURCE) != EXPECTED_SHA:
            raise SafetyError('运行中点位文件被更改，停止')
        if max(abs(a-b) for a,b in zip(actual,goal)) > MAX_STEP+12:
            raise SafetyError('单步目标相对反馈过大')
        moves = [(i+1,v) for i,v in enumerate(goal) if v != actual[i]]
        arm.transport.pending=deque(build_frame(3,pack_joint_counts(v,100,2000),sid) for sid,v in moves)
        emit('step_begin', index=index+1, count=len(goals), start=actual, target=goal)
        for sid,value in moves:
            if cancelled():
                raise SafetyError('停止发送剩余关节')
            arm.move_joint_counts(sid,value,100,2000)
        if arm.transport.pending:
            raise SafetyError('当前小步存在未发送帧')
        actual=wait_step(arm,actual,goal,emit,cancelled)
        emit('step_converged', index=index+1, positions=actual)
    return dict(start=before, target=target, returned_report=actual, steps=len(goals),
                controller_converged=True, operator_confirmed=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--points',type=int,nargs='+',default=DEFAULT_POINTS)
    ap.add_argument('--go',action='store_true')
    ap.add_argument('--empty-load',action='store_true')
    a=ap.parse_args()
    check_sequence(a.points)
    if not a.go or not a.empty_load:
        ap.error('首次验证必须 --go --empty-load，并现场确认空载清障')
    if sha(SOURCE) != EXPECTED_SHA:
        raise SafetyError('点位文件指纹改变，先核对')
    route=json.loads(SOURCE.read_text(encoding='utf-8'))
    targets=resolve_route(route)
    now=datetime.now(timezone(timedelta(hours=8)))
    run=HERE/'logs'/('replay_'+now.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run/'route_snapshot.json').write_bytes(SOURCE.read_bytes())
    print('RUN_DIR='+str(run),flush=True)
    result=dict(started_at=now.isoformat(),source_sha256=EXPECTED_SHA,points=a.points,
                empty_load=True,complete=False,segments=[],physical_stop_verified=False,
                synchronized_bus_write=False,limits_note='bounded joint increments are not collision proof',
                sources={str(p):sha(p) for p in (Path(__file__),HERE/'capture_joint_points.py',LIB/'nexarm/arm.py',LIB/'nexarm/commands.py',LIB/'nexarm/transport.py')})
    def save():
        (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    cancelled=lambda:(run/'STOP').exists()
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            with lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        transport=ReplayTransport(emit,cancelled)
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
            result.update(baseline=baseline,configuration=live,battery_mv=battery)
            save();print('BASELINE='+json.dumps(baseline),flush=True)
            if transport.stats()['checksum_errors']:
                raise SafetyError('预检存在坏帧')
            arm.enable_motion(True)
            for start,end in zip(a.points,a.points[1:]):
                print('SEGMENT %d -> %d'%(start,end),flush=True)
                entry=execute_segment(arm,targets[start-1],targets[end-1],emit,cancelled)
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
            save();print('RESULT='+json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
