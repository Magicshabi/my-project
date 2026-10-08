"""Single ID5 +64-count check of the successful HEX slider interface. No return/retry."""
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

EXPECTED=[1969,2007,1926,2104,1990,1875]
HEX_SHA='f9ca0477fe3b90162d90ef193cf121f5fed33b6e89f1344216e5f4c554879cfc'


def choose_target(samples):
    if len(samples)!=5 or any(len(s)!=6 for s in samples):
        raise SafetyError('需要五帧完整关节报告')
    if any(max(s[i] for s in samples)-min(s[i] for s in samples)>3 for i in range(6)):
        raise SafetyError('起点不稳定')
    if any(abs(samples[-1][i]-EXPECTED[i])>5 for i in range(6)):
        raise SafetyError('起点已偏离成功HEX的末帧；不猜测新姿态')
    target=samples[-1][4]+64
    if not 1988<=target<=2605:
        raise SafetyError('目标不在成功日志的ID5范围内')
    return target


class OnceTransport(Transport):
    def __init__(self,emit):
        super().__init__(log=lambda s:emit('library',message=s))
        self.emit=emit; self.allowed=None; self.attempted=False
    def send(self,cmd,args=b'',target_id=255):
        if target_id==5 and cmd==3 and self.allowed is not None and args==self.allowed:
            if self.attempted: raise SafetyError('单次动作额度已用完')
            self.attempted=True
        elif target_id!=255 or cmd not in (1,11,65) or args:
            raise SafetyError('本测试仅允许只读查询及一条已核对ID5命令')
        self.emit('tx_attempt',hex=build_frame(cmd,args,target_id).hex(),cmd=cmd,id=target_id)
        super().send(cmd,args,target_id)
        self.emit('tx_complete',cmd=cmd,id=target_id)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM9'); ap.add_argument('--go',action='store_true')
    a=ap.parse_args()
    if not a.go: ap.error('需要--go及已确认的空载、托稳、无障碍现场')
    source=LIB.parent/'HEX.txt'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=HEX_SHA:
        raise SafetyError('成功HEX已变化，先重新核对')
    run=LIB/'logs'/('hex_joint_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True); print('RUN_DIR='+str(run),flush=True)
    result=dict(motion_write_completed=False,physical_motion_confirmed=False,return_sent=False)
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(kind,**data):
            with lock:
                log.write(json.dumps(dict(event=kind,t=time.monotonic(),**data),ensure_ascii=False)+'\n'); log.flush()
        transport=OnceTransport(emit)
        transport.on_any(lambda f:emit('rx',id=f.id,cmd=f.cmd,args=f.args.hex()))
        arm=ArmController(transport=transport,log=lambda s:emit('library',message=s))
        try:
            arm.connect_serial(a.port)
            version=arm.get_version()
            if not version or version['esp_str']!='1.0.2' or version['at32_str']!='1.0.42':
                raise SafetyError('版本与已检查设备不符')
            samples=[]
            for _ in range(5):
                p=arm.get_pose()
                if p is None: raise SafetyError('缺少起点报告')
                samples.append(list(p.servos)); time.sleep(.15)
            target=choose_target(samples)
            result.update(version=version,baseline=samples[-1],servo_id=5,target=target,
                          delta_counts=64,acceleration=100,speed=2000,source_sha256=HEX_SHA)
            transport.allowed=pack_joint_counts(target,100,2000)
            emit('plan',**result)
            if (run/'STOP').exists(): raise SafetyError('停止文件存在，未下发')
            arm.enable_motion(True)
            arm.move_joint_counts(5,target,100,2000)
            result['motion_write_completed']=True
            observed=[]; end=time.monotonic()+4
            while time.monotonic()<end:
                if (run/'STOP').exists(): raise SafetyError('停止监测；物理停止未验证')
                p=arm.get_pose()
                if p is None: raise SafetyError('报告缺失，不补发')
                observed.append(list(p.servos)); emit('sample',positions=list(p.servos)); time.sleep(.12)
            result.update(final=observed[-1],reported_id5_change=observed[-1][4]-samples[-1][4],
                          target_error=observed[-1][4]-target,
                          controller_near_target=all(abs(s[4]-target)<=8 for s in observed[-3:]),
                          stats=arm.stats(),physical_observation='pending operator')
        except BaseException as exc:
            result['error']=str(exc) or type(exc).__name__
            raise
        finally:
            arm.enable_motion(False); arm.close()
            result['motion_attempted']=transport.attempted
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__': main()
