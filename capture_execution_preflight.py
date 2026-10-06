"""Capture read-only execution preflight boot; ESP32 reset, no host commands."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import serial
from read_teach_record_r6 import decode_snapshot

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--port', required=True)
p.add_argument('--reference', type=Path, required=True)
args = p.parse_args()
port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
port.port = args.port
port.dtr = False
port.rts = True
raw = bytearray()
try:
    port.open()
    port.dtr = False
    port.rts = False
    time.sleep(.05)
    port.rts = True
    time.sleep(.12)
    port.reset_input_buffer()
    port.rts = False
    deadline = time.monotonic() + 22
    while time.monotonic() < deadline:
        raw.extend(port.read(max(1, min(port.in_waiting, 8192))))
finally:
    port.close()
base = Path(__file__).parent / ('execution_preflight_boot_' + time.strftime('%Y%m%d_%H%M%S'))
base.with_suffix('.bin').write_bytes(raw)
text = raw.decode('utf-8', errors='replace')
base.with_suffix('.log').write_text(text, encoding='utf-8')
frames = []
parse_errors = []
for line in text.splitlines():
    if line.startswith('{'):
        try:
            frames.append(json.loads(line))
        except ValueError as exc:
            parse_errors.append(str(exc))
snapshots = [decode_snapshot(f) for f in frames if f.get('type') == 'teach_record']
reference = args.reference.read_bytes()
restored = bool(snapshots) and all(b == reference and r['complete'] for r, b in snapshots)
probe = [f for f in frames if f.get('type') == 'probe_result']
summary = {
    'port': args.port, 'log': str(base.with_suffix('.log')),
    'self_test': '[PreflightTest] PASS:' in text,
    'ready_count': text.count('nexarm-execution-preflight-r1b READY;'),
    'oled_ack': '[OLED] FOUND' in text,
    'panic': any(s in text for s in ('Guru Meditation', 'Brownout', 'Backtrace:', '[PreflightTest] FAIL')),
    'snapshots_match_original_eight_points': restored,
    'original_record_sha256': hashlib.sha256(reference).hexdigest(),
    'actual65_count': sum(f.get('type') == 'actual65' for f in frames),
    'register_reads': [f for f in frames if f.get('type') == 'register_read'],
    'register_timeouts': [f for f in frames if f.get('type') == 'register_timeout'],
    'other_frames': [f for f in frames if f.get('type') == 'other_frame'],
    'probe_results': probe, 'json_errors': parse_errors,
    'motion_sent': False, 'target_write_verified': False,
    'physical_power_cycle_verified': False,
}
summary['boot_ok'] = bool(summary['self_test'] and summary['ready_count'] == 1 and
                          summary['actual65_count'] > 0 and summary['oled_ack'] and
                          restored and not summary['panic'] and not parse_errors)
base.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
raise SystemExit(0 if summary['boot_ok'] else 1)
