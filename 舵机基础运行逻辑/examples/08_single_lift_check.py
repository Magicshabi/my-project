"""Once-only CMD8 +10mm Z check. No torque commands, return, retry or calibration writes."""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time

LIB=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(LIB))
from nexarm import ArmController
from nexarm import commands as C
from nexarm.eight_point import checked_pose, configuration, near, pose_dict, stable_capture
from nexarm.frames import build_frame
from nexarm.safety import SafetyError


def lift_target(current, expected, limits):
    current=checked_pose(current)
    if not near(current,checked_pose(expected),3,3):
        raise SafetyError('起点与刚刚检查的姿态不一致，不发动作')
    limits.check(**current)
    target=dict(current,z=current['z']+10)
    limits.check(**target)
    return target


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM9')
    ap.add_argument('--expected',nargs=6,type=float,required=True,metavar='VALUE')
    ap.add_argument('--go',action='store_true')
    a=ap.parse_args()
    if not a.go: ap.error('需要 --go；现场先确认空载、托稳、无障碍及异常')
    run=LIB/'logs'/('single_lift_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    print('RUN_DIR='+str(run),flush=True)
    lock=threading.Lock()
    result=dict(motion_write_completed=False,physical_motion_confirmed=False,return_sent=False)
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            row=dict(event=kind,t=time.monotonic(),**data)
            with lock:
                log.write(json.dumps(row,ensure_ascii=False)+'\n'); log.flush()
        arm=ArmController(log=lambda msg:emit('library',message=msg))
        arm.transport.on_any(lambda f:emit('rx_frame',id=f.id,cmd=f.cmd,args=f.args.hex()))
        try:
            arm.connect_serial(a.port)
            config=configuration(arm)
            baseline=stable_capture(arm)
            target=lift_target(baseline,dict(zip(('x','y','z','pitch','roll','claw'),a.expected)),arm.limits)
            tcp=arm.get_tcp_pose()
            joints=arm.get_real_joints()
            if tcp is None or joints is None or len(joints)!=6:
                raise SafetyError('位姿或关节报告不完整')
            if not near(baseline,{k:tcp[k] for k in baseline},3,3):
                raise SafetyError('两组位姿报告不一致')
            (run/'baseline.json').write_text(json.dumps(dict(configuration=config,pose=baseline,tcp=tcp,joints=joints),indent=2),encoding='utf-8')
            duration=2500
            emit('motion_intent',target=target,duration_ms=duration,
                 wire_hex=build_frame(int(C.Cmd.COORDINATE_SET),C.pack_coord(**target,time_ms=duration)).hex())
            if (run/'STOP').exists(): raise SafetyError('已请求停止，未下发')
            arm.enable_motion(True)
            arm.move_to(**target,time_ms=duration,wait=False)
            result['motion_write_completed']=True
            emit('motion_write_completed')
            end=time.monotonic()+5
            samples=[]
            while time.monotonic()<end:
                if (run/'STOP').exists(): raise SafetyError('停止监测；当前运动未获物理停止证明')
                pose=arm.get_pose()
                joint=arm.get_real_joints()
                if pose is None or joint is None: raise SafetyError('动作后报告缺失，不补发')
                samples.append(pose_dict(pose))
                emit('sample',pose=asdict(pose),joints=joint)
                time.sleep(.12)
            result.update(baseline=baseline,target=target,final=samples[-1],
                          reported_z_change=samples[-1]['z']-baseline['z'],
                          controller_within_3mm_3deg=all(near(p,target,3,3) for p in samples[-3:]),
                          stats=arm.stats(),physical_observation='pending operator')
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            emit('aborted',reason=result['error'],physical_stop_verified=False)
            raise
        finally:
            arm.enable_motion(False)
            arm.close()
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__': main()
