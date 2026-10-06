"""Reset/capture single-joint firmware boot, verify it never moves on startup."""
import argparse
import json
from pathlib import Path
import time
import serial
from read_teach_record_r6 import decode_snapshot

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True)
p.add_argument('--reference',required=True,type=Path)
p.add_argument('--firmware',default='nexarm-single-joint-probe-r1',choices=('nexarm-single-joint-probe-r1','nexarm-single-joint-probe-r1b'))
args=p.parse_args()
port=serial.Serial(port=None,baudrate=1000000,timeout=.1)
port.port=args.port; port.dtr=False; port.rts=True
raw=bytearray()
try:
    port.open(); port.dtr=False; port.rts=False; time.sleep(.05)
    port.rts=True; time.sleep(.12); port.reset_input_buffer(); port.rts=False
    deadline=time.monotonic()+20
    while time.monotonic()<deadline: raw.extend(port.read(max(1,min(port.in_waiting,8192))))
finally: port.close()
base=Path(__file__).parent/('single_joint_boot_'+time.strftime('%Y%m%d_%H%M%S'))
base.with_suffix('.bin').write_bytes(raw)
text=raw.decode('utf-8',errors='replace'); base.with_suffix('.log').write_text(text,encoding='utf-8')
frames=[]; errors=[]
for line in text.splitlines():
    if line.startswith('{'):
        try: frames.append(json.loads(line))
        except ValueError as e: errors.append(str(e))
records=[decode_snapshot(f) for f in frames if f.get('type')=='teach_record']
states=[f for f in frames if f.get('type')=='single_probe_state']
goals=[f for f in frames if f.get('type')=='single_goal_sent']
faults=[f for f in frames if f.get('type')=='probe_fault']
reference=args.reference.read_bytes()
report={'log':str(base.with_suffix('.log')),'self_test':'[SingleProbeTest] PASS:' in text,
        'firmware':args.firmware,'ready_count':text.count(args.firmware+' READY;'),
        'record_matches':bool(records) and all(b==reference for _,b in records),
        'actual65_count':sum(f.get('type')=='actual65' for f in frames),'goals_at_boot':goals,
        'faults':faults,'states_idle':bool(states) and all(f['phase']==0 and f['spent'] is False for f in states),
        'last_state':states[-1] if states else None,'json_errors':errors,
        'panic':any(w in text for w in ('Guru Meditation','Brownout','Backtrace:')),
        'hardware_movement_verified':False}
report['ok']=bool(report['self_test'] and report['ready_count']==1 and report['record_matches'] and
                  report['actual65_count']>0 and not goals and not faults and report['states_idle'] and
                  not errors and not report['panic'])
base.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(0 if report['ok'] else 1)
