"""Only CMD77 angle-limit queries, plus current position; no settings are written.

CMD77 + one servo ID verified against official Robot_Arm.cpp/Global.h and EXE.
Reply retained as raw bytes: display units are not inferred from packet alone.
"""
import json
from datetime import datetime,timezone,timedelta
from pathlib import Path
import threading
import time
from capture_joint_points import ReadOnlyTransport,ArmController,configuration
from nexarm import Transport,build_frame


class LimitQueries(ReadOnlyTransport):
    def send(self,cmd,args=b'',target_id=255):
        if target_id==255 and cmd==77 and len(args)==1 and 1<=args[0]<=6:
            self.emit('tx_attempt',cmd=cmd,id=target_id,wire_hex=build_frame(cmd,args,target_id).hex())
            Transport.send(self,cmd,args,target_id)
            self.emit('tx_complete',cmd=cmd,id=target_id)
        else:
            super().send(cmd,args,target_id)


def main():
    root=Path(__file__).resolve().parent
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    run=root/'logs'/('servo_limits_'+stamp+'_'+str(time.time_ns())[-6:]);run.mkdir(parents=True)
    result={'read_only':True,'motion_commands':0,'limits':{}}
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as f:
        def emit(event,**data):
            with lock:
                f.write(json.dumps(dict(event=event,t=time.monotonic(),**data))+'\n');f.flush()
        t=LimitQueries(emit);t.on_any(lambda v:emit('rx',id=v.id,cmd=v.cmd,args=v.args.hex()))
        arm=ArmController(transport=t,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial('COM9')
            p=arm.get_pose();result['before']=list(p.servos) if p else None
            for sid in range(1,7):
                reply=t.request(77,bytes([sid]),timeout=.7)
                result['limits'][str(sid)]=None if reply is None else dict(id=reply.id,args_hex=reply.args.hex(),bytes=list(reply.args))
                time.sleep(.12)
            p=arm.get_pose();result['after']=list(p.servos) if p else None
        finally:
            arm.close();result['stats']=t.stats();result['serial_closed']=True
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False));print('RUN_DIR='+str(run))


if __name__=='__main__':main()
