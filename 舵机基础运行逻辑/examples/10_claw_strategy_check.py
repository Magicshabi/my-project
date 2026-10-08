"""CLAW-XREV S1->S2: one ID6 -50-count CMD51 request over 1000ms.

S3 frame comparison is a separate, explicitly selected run. No torque, homing,
KEY1 emulation, automatic return, retry, or encoder-to-XYZ conversion.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import sys
import threading
import time

LIB=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(LIB))
from nexarm import ArmController, Transport, SafetyError, build_frame
from nexarm.commands import pack_joint_counts
from nexarm.eight_point import configuration, pose_dict


def plan_request(samples, claw_deg, claw_limits, interface):
    if len(samples)!=5 or any(len(s)!=6 for s in samples):
        raise SafetyError('需要五帧完整报告')
    if any(max(s[i] for s in samples)-min(s[i] for s in samples)>3 for i in range(6)):
        raise SafetyError('起点报告未稳定')
    base=samples[-1][5]
    target=base-50
    # Conservative test interior, not an assertion about mechanical end stops.
    if not 1200<=target<base<=2100:
        raise SafetyError('夹爪位置不在本次已审查的测试区间')
    expected_claw=claw_deg-50*360/4096
    if not claw_limits[0]<=expected_claw<=claw_limits[1]:
        raise SafetyError('计划夹爪角度超出当前配置范围')
    if interface=='cmd51':
        cmd,sid,args=51,255,struct.pack('<BhH',6,target,1000)
    elif interface=='gui_raw':
        cmd,sid,args=3,6,pack_joint_counts(target,100,2000)
    else:
        raise SafetyError('未知接口')
    return dict(servo_id=6,baseline=base,target=target,delta_counts=-50,
                requested_duration_ms=1000 if interface=='cmd51' else None,
                interface=interface,cmd=cmd,id=sid,args=args.hex(),
                wire_hex=build_frame(cmd,args,sid).hex())


class StrategyTransport(Transport):
    def __init__(self,emit):
        super().__init__(log=lambda s:emit('library',message=s))
        self.emit=emit; self.allowed=None; self.motion_attempted=False
    def send(self,cmd,args=b'',target_id=255):
        request=(target_id,cmd,bytes(args))
        read=(target_id==255 and ((cmd in (1,2,11,65,80,64) and not args)
                                 or (cmd==58 and len(args)==1 and 1<=args[0]<=6)))
        if not read:
            if self.allowed!=request or self.motion_attempted:
                raise SafetyError('禁止未审查命令、力矩写入和重复动作')
            self.motion_attempted=True
        self.emit('tx_attempt',id=target_id,cmd=cmd,wire_hex=build_frame(cmd,args,target_id).hex())
        super().send(cmd,args,target_id)
        self.emit('tx_complete',id=target_id,cmd=cmd)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM9')
    ap.add_argument('--interface',choices=('cmd51','gui_raw'),default='cmd51')
    ap.add_argument('--go',action='store_true')
    a=ap.parse_args()
    if not a.go: ap.error('需要--go及当前现场准备确认')
    run=LIB/'logs'/('claw_xrev_'+a.interface+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True); print('RUN_DIR='+str(run),flush=True)
    result=dict(strategy='PROJECT_STATE CLAW-XREV',interface=a.interface,
                motion_write_completed=False,physical_test_passed=False,
                physical_observation='pending operator',return_sent=False)
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            with lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n'); log.flush()
        transport=StrategyTransport(emit)
        transport.on_any(lambda f:emit('rx',id=f.id,cmd=f.cmd,args=f.args.hex()))
        arm=ArmController(transport=transport,log=lambda s:emit('library',message=s))
        try:
            arm.connect_serial(a.port)
            config=configuration(arm)
            mv=arm.get_battery_mv()
            if mv is None: raise SafetyError('未收到电压报告')
            samples=[]; poses=[]; joints=[]
            for _ in range(5):
                p=arm.get_pose(); j=arm.get_real_joints()
                if p is None or j is None or len(j)!=6:
                    raise SafetyError('S1缺少完整位姿/关节报告')
                samples.append(list(p.servos)); poses.append(pose_dict(p)); joints.append(j)
                time.sleep(.15)
            if any(max(j[i][0] for j in joints)-min(j[i][0] for j in joints)>3 for i in range(6)):
                raise SafetyError('S1关节报告未稳定')
            if transport.stats()['checksum_errors']:
                raise SafetyError('S1存在坏帧')
            plan=plan_request(samples,poses[-1]['claw'],arm.limits.claw,a.interface)
            snapshot=dict(configuration=config,battery_mv=mv,poses=poses,positions=samples,joints=joints,
                          stable_tolerance_counts=3,independent_physical_feedback=False,
                          script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
            (run/'baseline.json').write_text(json.dumps(snapshot,indent=2),encoding='utf-8')
            result.update(plan=plan,configuration=config,battery_mv=mv)
            emit('plan',**plan)
            print('PLAN='+json.dumps(plan,ensure_ascii=False),flush=True)
            current=arm.get_pose()
            if current is None or any(abs(current.servos[i]-samples[-1][i])>3 for i in range(6)):
                raise SafetyError('发送前起点发生变化')
            if (run/'STOP').exists(): raise SafetyError('已请求停止，未发送')
            transport.allowed=(plan['id'],plan['cmd'],bytes.fromhex(plan['args']))
            arm.enable_motion(True); arm.gate.require()
            transport.send(plan['cmd'],bytes.fromhex(plan['args']),target_id=plan['id'])
            result['motion_write_completed']=True
            end=time.monotonic()+5; observed=[]
            while time.monotonic()<end:
                if (run/'STOP').exists(): raise SafetyError('停止监测；当前物理运动是否停止须现场确认')
                p=arm.get_pose()
                if p is None: raise SafetyError('动作后报告缺失，不补发')
                observed.append(list(p.servos)); emit('sample',pose=pose_dict(p),positions=list(p.servos))
                time.sleep(.12)
            result.update(final_positions=observed[-1],
                          reported_id6_change=observed[-1][5]-plan['baseline'],
                          target_error=observed[-1][5]-plan['target'],
                          controller_near_target=all(abs(s[5]-plan['target'])<=8 for s in observed[-3:]),
                          stats=arm.stats())
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'],physical_stop_verified=False)
            raise
        finally:
            arm.enable_motion(False); arm.close()
            result['motion_attempted']=transport.motion_attempted
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__': main()
