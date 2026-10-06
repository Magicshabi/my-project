"""Passively log one operator-triggered probe; never send host commands/reset."""
import argparse
import json
from pathlib import Path
import time
import serial

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--port', default='COM9')
p.add_argument('--seconds', type=int, default=300)
a = p.parse_args()
if not 10 <= a.seconds <= 600:
    p.error('--seconds must be between 10 and 600')
base = Path(__file__).parent / ('single_joint_trial_' + time.strftime('%Y%m%d_%H%M%S'))
port = serial.Serial(port=None, baudrate=1000000, timeout=.1)
port.port = a.port
port.dtr = False
port.rts = False
rows, errors, pending = [], [], bytearray()
end_after = None
first_state = False
try:
    port.open()
    print(json.dumps({'listening': str(base.with_suffix('.log')), 'port': a.port,
                      'host_commands_sent': False, 'seconds': a.seconds}), flush=True)
    deadline = time.monotonic() + a.seconds
    with base.with_suffix('.bin').open('wb') as binary, base.with_suffix('.log').open('w', encoding='utf-8') as log:
        while time.monotonic() < deadline:
            chunk = port.read(max(1, min(port.in_waiting, 8192)))
            binary.write(chunk); binary.flush()
            pending.extend(chunk)
            while b'\n' in pending:
                line, _, pending = pending.partition(b'\n')
                text = line.decode('utf-8', errors='replace').rstrip('\r')
                log.write(text + '\n'); log.flush()
                if not text.startswith('{'):
                    continue
                try:
                    row = json.loads(text)
                except ValueError as e:
                    errors.append(str(e)); continue
                rows.append(row)
                kind = row.get('type')
                if kind == 'single_probe_state' and not first_state:
                    print(json.dumps(row), flush=True); first_state = True
                if kind in ('single_goal_sent', 'single_probe_result', 'probe_fault', 'single_probe_evidence'):
                    print(json.dumps(row), flush=True)
                if kind in ('single_probe_result', 'probe_fault', 'single_probe_evidence') and end_after is None:
                    end_after = time.monotonic() + 3
            if end_after is not None and time.monotonic() >= end_after:
                break
finally:
    port.close()

actual = [r for r in rows if r.get('type') == 'actual65']
report = {'log': str(base.with_suffix('.log')), 'host_commands_sent': False,
          'goals': [r for r in rows if r.get('type') == 'single_goal_sent'],
          'results': [r for r in rows if r.get('type') == 'single_probe_result'],
          'faults': [r for r in rows if r.get('type') == 'probe_fault'],
          'evidence': [r for r in rows if r.get('type') == 'single_probe_evidence'],
          'actual65_count': len(actual), 'json_errors': errors,
          'operator_observation': 'pending', 'full_playback_validated': False}
base.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2), flush=True)
