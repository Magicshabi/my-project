"""Sequential six-axis raw-write round trips; host-observed + operator-observed results.

One joint at a time. Each round permits exactly two writes: excursion and original
baseline. No torque/config writes, homing, broadcast motion, retries or auto recovery.
After each round enter 'yes' only for a real operator-confirmed movement AND return.
"""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

HERE=Path(__file__).resolve().parent
LIB=HERE.parent/'舵机基础运行逻辑'
sys.path.insert(0,str(LIB))
from nexarm import ArmController, Transport, SafetyError, build_frame
from nexarm.commands import pack_joint_counts
from nexarm.eight_point import configuration

EXPECTED=(1972,2003,1675,2104,1989,1802)
STEPS=((6,-500,2000),(5,150,200),(4,-150,200),(3,150,200),(2,150,200),(1,150,200))


def stable_positions(arm):
    samples=[]
    for _ in range(5):
        p=arm.get_pose()
        if p is None or len(p.servos)!=6: raise SafetyError('缺少完整当前位置报告')
        values=list(p.servos)
        if any(type(v) is not int or not 0<=v<=4095 for v in values):
            raise SafetyError('位置报告超出协议计数范围')
        samples.append(values); time.sleep(.1)
    if any(max(p[i] for p in samples)-min(p[i] for p in samples)>5 for i in range(6)):
        raise SafetyError('当前位置报告不稳定')
    return samples[-1]


def make_plan(base):
    if len(base)!=6 or any(type(v) is not int or abs(v-EXPECTED[i])>12 for i,v in enumerate(base)):
        raise SafetyError('当前姿态已偏离本轮预检位置，需要重新评估')
    result=[]
    for sid,delta,speed in STEPS:
        start=base[sid-1];target=start+delta
        low,high=(1100,1900) if sid==6 else (900,3200)
        if not low<=start<=high or not low<=target<=high:
            raise SafetyError('目标超出本轮保守测试区间；不是机械限位判定')
        result.append(dict(id=sid,start=start,target=target,delta=delta,acceleration=100,speed=speed))
    return result


class GuardedTransport(Transport):
    def __init__(self,emit):
        super().__init__(log=lambda msg:emit('transport',message=msg))
        self.emit=emit;self.pending=[];self.motion_attempts=0;self.failed_write=False
    def send(self,cmd,args=b'',target_id=255):
        wire=build_frame(cmd,args,target_id)
        read=(target_id==255 and ((cmd in (1,2,11,65,80,64) and not args)
                                 or (cmd==58 and len(args)==1 and 1<=args[0]<=6)))
        if not read:
            if self.failed_write or not self.pending or wire!=self.pending[0]:
                raise SafetyError('命令不在本轴已审核的两条写入中，拒绝')
            self.pending.pop(0);self.motion_attempts+=1
        self.emit('tx_attempt',cmd=cmd,id=target_id,wire_hex=wire.hex())
        try: super().send(cmd,args,target_id)
        except BaseException:
            self.failed_write=True
            raise
        self.emit('tx_complete',cmd=cmd,id=target_id)


def wait_report(arm, sid, target, others, emit, cancelled, timeout=8):
    start=time.monotonic(); accepted=[]
    while time.monotonic()-start<timeout:
        if cancelled(): raise SafetyError('已停止后续下发；物理停止未确认')
        p=arm.get_pose()
        if p is None: raise SafetyError('回包缺失；不补发、不自动回位')
        values=list(p.servos)
        emit('sample',sid=sid,target=target,positions=values)
        if any(abs(values[i]-others[i])>12 for i in range(6) if i!=sid-1):
            raise SafetyError('其他关节报告发生明显变化，停止后续下发')
        if time.monotonic()-start>=2 and abs(values[sid-1]-target)<=12:
            accepted.append(values[sid-1]);accepted=accepted[-4:]
            if len(accepted)==4 and max(accepted)-min(accepted)<=3:
                return values
        else: accepted=[]
        time.sleep(.12)
    raise TimeoutError('ID%d未报告稳定到位；不重试、不自动回位'%sid)


def run_round(arm, plan, original, emit, cancelled):
    sid=plan['id'];before=stable_positions(arm)
    if any(abs(before[i]-original[i])>12 for i in range(6)):
        raise SafetyError('本轴测试前已偏离整轮初始姿态')
    transport=arm.transport
    transport.pending=[build_frame(3,pack_joint_counts(p,100,plan['speed']),sid)
                       for p in (plan['target'],plan['start'])]
    if cancelled(): raise SafetyError('已请求停止')
    print('MOVE '+json.dumps(plan,ensure_ascii=False),flush=True)
    arm.move_joint_counts(sid,plan['target'],100,plan['speed'])
    at_target=wait_report(arm,sid,plan['target'],before,emit,cancelled)
    print('ID%d 报告已到目标，停留1秒后回到起点。'%sid,flush=True)
    for _ in range(10):
        if cancelled(): raise SafetyError('已请求停止，不自动回位')
        time.sleep(.1)
    arm.move_joint_counts(sid,plan['start'],100,plan['speed'])
    returned=wait_report(arm,sid,plan['start'],before,emit,cancelled)
    return dict(**plan,at_target=at_target,returned=returned,
                return_error=returned[sid-1]-plan['start'],
                controller_round_trip=True,operator_confirmed=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM9');ap.add_argument('--go',action='store_true')
    ap.add_argument('--axes',type=int,nargs='+',choices=range(1,7),default=[6,5,4,3,2,1])
    ap.add_argument('--body-speed',type=int,choices=(200,2000),default=200)
    a=ap.parse_args()
    if len(set(a.axes))!=len(a.axes):ap.error('轴号不能重复')
    if not a.go: ap.error('需要--go及已确认的空载、托稳、避障现场')
    now=datetime.now(timezone(timedelta(hours=8)))
    run=HERE/'logs'/('six_joints_'+now.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True);print('RUN_DIR='+str(run),flush=True)
    result=dict(started_at=now.isoformat(),complete=False,axes=[],requested_axes=a.axes,
                body_speed=a.body_speed,physical_stop_verified=False)
    lock=threading.Lock()
    def save():
        (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            with lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n');log.flush()
        transport=GuardedTransport(emit)
        transport.on_any(lambda f:emit('rx',id=f.id,cmd=f.cmd,args=f.args.hex()))
        arm=ArmController(transport=transport,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial(a.port)
            config=configuration(arm)
            if config['version']['esp_str']!='1.0.2' or config['version']['at32_str']!='1.0.42':
                raise SafetyError('版本已改变，先核对')
            voltage=arm.get_battery_mv()
            if voltage is None: raise SafetyError('未收到电压报告')
            base=stable_positions(arm);all_steps={p['id']:p for p in make_plan(base)}
            plan=[dict(all_steps[sid],speed=2000 if sid==6 else a.body_speed) for sid in a.axes]
            if transport.stats()['checksum_errors']:raise SafetyError('基线出现坏帧')
            sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in (Path(__file__),LIB/'nexarm/arm.py',LIB/'nexarm/commands.py',LIB/'nexarm/transport.py')}
            result.update(configuration=config,battery_mv=voltage,baseline=base,plan=plan,sources=sources)
            save();print('BASELINE='+json.dumps(base),flush=True)
            arm.enable_motion(True)
            cancelled=lambda:(run/'STOP').exists()
            for step in plan:
                entry=run_round(arm,step,base,emit,cancelled)
                result['axes'].append(entry);save()
                print('ROUND_DONE='+json.dumps(entry),flush=True)
                try:answer=input('OBSERVE_ID%d: yes=实体移动并回位且无异常; 其他输入=停止\n'%step['id']).strip()
                except EOFError:answer='no operator response'
                entry['operator_answer']=answer;entry['operator_confirmed']=(answer=='yes')
                emit('operator_observation',sid=step['id'],answer=answer);save()
                if answer!='yes':raise SafetyError('现场未确认本轴运动并回位，停止后续轴')
            final=stable_positions(arm)
            if any(abs(final[i]-base[i])>12 for i in range(6)):
                raise SafetyError('结束报告偏离初始姿态')
            result.update(complete=True,final_positions=final,stats=arm.stats())
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'],physical_stop_verified=False)
            print('STOP: 后续动作已停止。当前物理动作未验证停止；若有异常请托稳并断舵机电源。',flush=True)
            raise
        finally:
            arm.enable_motion(False);arm.close()
            result['motion_attempts']=transport.motion_attempts;save()
            print('RESULT='+json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
