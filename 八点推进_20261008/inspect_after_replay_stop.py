"""Read-only follow-up after the first replay step; no torque/position writes."""
import json
from pathlib import Path
import threading
import time
from datetime import datetime,timezone,timedelta
from capture_joint_points import ReadOnlyTransport,ArmController,configuration,checked_counts

HERE=Path(__file__).resolve().parent


def main():
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    run=HERE/'logs'/('post_stop_'+stamp+'_'+str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    result=dict(read_only=True,motion_commands=0,samples=[],
                operator_observation='ID4偏移、末端向前伸；现已静止，无红灯、异响或接触',
                prior_run='replay_20261009_171715_298500')
    lock=threading.Lock()
    with (run/'events.jsonl').open('w',encoding='utf-8') as log:
        def emit(event,**data):
            with lock:
                log.write(json.dumps(dict(event=event,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        t=ReadOnlyTransport(emit)
        t.on_any(lambda f:emit('rx',cmd=f.cmd,id=f.id,args=f.args.hex()))
        arm=ArmController(transport=t,log=lambda msg:emit('library',message=msg))
        try:
            arm.connect_serial('COM9')
            result['configuration']=configuration(arm)
            result['battery_mv']=arm.get_battery_mv()
            for _ in range(20):
                a=arm.get_pose();b=arm.get_real_joints()
                if a is None or b is None or len(b)!=6:
                    raise RuntimeError('Missing feedback')
                row=dict(cmd11=checked_counts(a.servos),cmd65=checked_counts([x[0] for x in b]),
                         controller_pose={k:getattr(a,k) for k in ('x','y','z','pitch','roll','claw')})
                result['samples'].append(row);emit('sample',**row)
                time.sleep(.2)
            result['position_ranges']={k:[[min(s[k][i] for s in result['samples']),max(s[k][i] for s in result['samples'])] for i in range(6)] for k in ('cmd11','cmd65')}
            result['last']=result['samples'][-1]
        except BaseException as exc:
            result['error']=str(exc)
            raise
        finally:
            arm.close();result.update(serial_closed=True,stats=t.stats())
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps({k:v for k,v in result.items() if k!='samples'},ensure_ascii=False),flush=True)
            print('RUN_DIR='+str(run),flush=True)


if __name__=='__main__':main()
