"""Capture USB-only read-only diagnostic boot; never send application commands."""
import json
import pathlib
import time
from collections import Counter
import serial

port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
port.port = 'COM9'
port.dtr = False
port.rts = True
received = bytearray()
try:
    port.open()
    port.dtr = False
    port.rts = False
    time.sleep(0.05)
    port.rts = True
    time.sleep(0.12)
    port.reset_input_buffer()
    port.rts = False
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        received.extend(port.read(max(1, port.in_waiting)))
finally:
    port.close()

base = pathlib.Path(__file__).parent / 'readonly_r1_boot'
decoded = received.decode('utf-8', errors='replace')
base.with_suffix('.bin').write_bytes(received)
base.with_suffix('.log').write_text(decoded, encoding='utf-8')
ready = decoded.count('[ReadOnly] nexarm-readonly-r1 READY; only queries 11 and 65; no torque/target writes')
passed = '[ReadOnlyTest] PASS: actual-position decoder' in decoded
panic = any(s in decoded for s in ('Guru Meditation', 'Rebooting...', 'Backtrace:', 'Brownout', '[ReadOnlyTest] FAIL'))
counts = Counter()
examples = {}
for line in decoded.splitlines():
    if line.startswith('{'):
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            counts['invalid_json'] += 1
            continue
        kind = frame.get('type', 'unknown')
        counts[kind] += 1
        examples.setdefault(kind, frame)
    elif '[ReadOnly' in line:
        print(line[line.index('[ReadOnly'):])
summary = dict(bytes=len(received), self_test=passed, ready_count=ready, panic=panic,
               counts=dict(counts), first_examples=examples,
               external_servo_power='disconnected, confirmed by user')
base.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
raise SystemExit(0 if passed and ready == 1 and not panic else 1)
