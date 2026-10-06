"""Capture the read-only r2 diagnostic; preserve raw responses and eight points."""
import argparse, json, time
from pathlib import Path
import serial
from read_handle_teach import decode_record
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',default='COM9')
p.add_argument('--log',type=Path,help='Re-analyze saved capture without opening a serial port')
a=p.parse_args()
if a.log:
    base=a.log.with_suffix('');raw=base.with_suffix('.bin').read_bytes()
else:
    base=Path(__file__).parent/('read_diagnostic_'+time.strftime('%Y%m%d_%H%M%S'))
    s=serial.Serial(port=None,baudrate=1000000,timeout=.1);s.port=a.port;s.dtr=False;s.rts=True
    raw=bytearray()
    try:
        s.open();s.dtr=False;s.rts=False;time.sleep(.05);s.rts=True;time.sleep(.12);s.reset_input_buffer();s.rts=False
        end=time.monotonic()+30
        with base.with_suffix('.bin').open('wb') as f:
            while time.monotonic()<end:
                chunk=s.read(max(1,min(s.in_waiting,8192)));raw.extend(chunk);f.write(chunk);f.flush()
    finally:s.close()
text=raw.decode('utf-8',errors='replace');base.with_suffix('.log').write_text(text,encoding='utf-8')
frames=[];errors=[]
# Capture may end between bytes of a valid line. Preserve but do not parse it.
complete,sep,trailing=text.rpartition('\n')
if not sep:complete='';trailing=text
for line in complete.splitlines():
    if line.startswith('{'):
        try:frames.append(json.loads(line))
        except ValueError as e:errors.append(str(e))
records=[r for r in frames if r.get('type')=='teach_record']
reference=(Path(__file__).parent/'validation_runs/readout_20261005_180008/eight_points.bin').read_bytes()
record_ok=bool(records)
for r in records:
    try:
        b=bytes.fromhex(r['record_hex']);d=decode_record(b)
        record_ok &= b==reference and d['valid_mask']==255 and r['firmware']=='nexarm-read-diagnostic-r2' and r['namespace']=='handle_free' and r['storage_fault'] is False and r['playback_enabled'] is False
    except (ValueError,KeyError):record_ok=False
select=lambda t:[r for r in frames if r.get('type')==t]
states=select('diag_state');power=select('supply_estimate')
report={'log':str(base.with_suffix('.log')),'firmware':'nexarm-read-diagnostic-r2',
    'self_test':'[ReadDiag] SELF TEST PASS' in text,'ready_count':text.count('nexarm-read-diagnostic-r2 READY;'),
    'record_matches':bool(record_ok),'queries':select('diag_query'),'query_ends':select('diag_window_end'),
    'raw_replies':select('diag_rx'),'last_state':states[-1] if states else None,
    'supply_estimate_range_mv':[min(r['estimated_board_mv'] for r in power),max(r['estimated_board_mv'] for r in power)] if power else None,
    'supply_last':power[-1] if power else None,'actual65_count':len(select('actual65')),
    'json_errors':errors,'trailing_fragment_bytes':len(trailing.encode('utf-8')),'fault':any(w in text for w in ('Guru Meditation','Brownout','Backtrace:','TX BLOCKED','TX FAILED','INVALID RECORD','SELF TEST FAIL')),
    'host_commands_sent':False,'motor_motion_tested':False}
report['capture_ok']=bool(report['self_test'] and report['ready_count']==1 and record_ok and states and states[-1]['done'] and len(report['queries'])==len(report['query_ends']) and not errors and not report['fault'])
base.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('queries','query_ends','raw_replies')},indent=2),flush=True)
raise SystemExit(0 if report['capture_ok'] else 1)
