"""Log ID6 torque test; default is passive. --run-once sends one guarded start."""
import argparse,json,time
from pathlib import Path
import serial
from read_handle_teach import decode_record
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',default='COM9');p.add_argument('--run-once',action='store_true');p.add_argument('--reset',action='store_true')
a=p.parse_args()
if a.reset and a.run_once:p.error('Never combine reset and motion start')
base=Path(__file__).parent/('id6_torque_'+('run_' if a.run_once else 'boot_')+time.strftime('%Y%m%d_%H%M%S'))
reference=(Path(__file__).parent/'validation_runs/readout_20261005_180008/eight_points.bin').read_bytes()
s=serial.Serial(port=None,baudrate=1000000,timeout=.05);s.port=a.port;s.dtr=False;s.rts=a.reset
rows=[];errors=[];pending=bytearray();state=None;valid_record=False;positions=[];sent_at=None;stop_sent=False;terminal_at=None
raw=bytearray()
try:
    s.open();s.dtr=False;s.rts=False
    if a.reset:time.sleep(.05);s.rts=True;time.sleep(.12);s.reset_input_buffer();s.rts=False
    deadline=time.monotonic()+20
    with base.with_suffix('.bin').open('wb') as f,base.with_suffix('.log').open('w',encoding='utf-8') as log:
        while time.monotonic()<deadline:
            chunk=s.read(max(1,min(s.in_waiting,8192)));raw.extend(chunk);f.write(chunk);f.flush();pending.extend(chunk)
            while b'\n' in pending:
                line,_,pending=pending.partition(b'\n');text=line.decode('utf-8',errors='replace').rstrip('\r');log.write(text+'\n');log.flush()
                if not text.startswith('{'):continue
                try:r=json.loads(text)
                except ValueError as e:errors.append(str(e));continue
                rows.append(r)
                if r.get('type')=='actual65':positions=(positions+[r])[-5:]
                if r.get('type')=='teach_record':
                    try:
                        b=bytes.fromhex(r['record_hex']);d=decode_record(b)
                        valid_record=(b==reference and d['valid_mask']==255 and r['firmware']=='nexarm-id6-torque-probe-r3' and r['namespace']=='handle_free' and r['storage_fault'] is False and r['playback_enabled'] is False)
                    except (ValueError,KeyError):valid_record=False
                if r.get('type')=='torque_probe_tx':print(json.dumps(r),flush=True)
                if r.get('type')=='torque_probe_state':
                    state=r
                    if state['phase']==5 and terminal_at is None:terminal_at=time.monotonic();print(json.dumps(r),flush=True)
            now=time.monotonic()
            if a.run_once and sent_at is None:
                fresh=positions and state and all(type(v) is int and 0<=v<=4095 for r in positions for v in r['pulses'])
                if fresh and len(positions)==5 and valid_record and not errors:
                    assert state['firmware']=='nexarm-id6-torque-probe-r3' and state['self_test_passed'] is True
                    if state['phase']!=0 or state['spent']:raise RuntimeError('Already used or non-idle; will not restart or repeat')
                    assert all(max(r['pulses'][j] for r in positions)-min(r['pulses'][j] for r in positions)<=2 for j in range(6)), 'Position not still'
                    sent_at=now;deadline=now+15  # consume before touching UART; no retry
                    if s.write(b'RUN_ID6_ONCE\n')!=13:raise RuntimeError('Short start write; no retry')
                    s.flush()
                    print('START SENT ONCE: observing ID6, no retries',flush=True)
            if sent_at is not None and terminal_at is None and now-sent_at>10 and not stop_sent:
                s.write(b'STOP_ID6\n');s.flush();stop_sent=True
                print('HOST DEADLINE: release requested; cut servo power if abnormal',flush=True)
            if terminal_at is not None and now-terminal_at>=3:break
finally:
    if sent_at is not None and terminal_at is None and s.is_open:
        try:
            s.write(b'\nSTOP_ID6\n');s.flush();stop_sent=True
            print('No completed result: release requested; operator must check hardware',flush=True)
        except serial.SerialException:print('Release request failed; operator must cut external servo power',flush=True)
    s.close()
states=[r for r in rows if r.get('type')=='torque_probe_state'];tx=[r for r in rows if r.get('type')=='torque_probe_tx']
report={'log':str(base.with_suffix('.log')),'start_sent':sent_at is not None,'stop_request_sent':stop_sent,'record_matches':valid_record,'tx':tx,'last_state':state,'actual65_count':sum(r.get('type')=='actual65' for r in rows),'json_errors':errors,'trailing_fragment_bytes':len(pending),'operator_observation':'pending','full_playback_validated':False}
if a.run_once:
    report['capture_ok']=bool(sent_at is not None and state and state['phase']==5 and valid_record and not errors)
else:
    report['capture_ok']=bool(states and all(r['phase']==0 and not r['spent'] and r['self_test_passed'] for r in states) and valid_record and not tx and not errors and report['actual65_count']>=5)
base.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['capture_ok'] else 1)
