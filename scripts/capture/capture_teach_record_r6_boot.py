"""Capture record-only firmware boot; resets ESP32, sends no motor commands."""
import argparse
import json
from pathlib import Path
import time
from collections import Counter
import serial
from read_teach_record_r6 import decode_snapshot

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM9')
parser.add_argument('--variant', choices=('r6', 'r6k1', 'r7', 'r7k2'), default='r6')
args = parser.parse_args()

port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
port.port = args.port
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
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        received.extend(port.read(max(1, min(port.in_waiting, 8192))))
finally:
    port.close()

base = Path(__file__).parent / f'teach_record_{args.variant}_boot'
decoded = received.decode('utf-8', errors='replace')
base.with_suffix('.bin').write_bytes(received)
base.with_suffix('.log').write_text(decoded, encoding='utf-8')
ready = decoded.count(f'[TeachRecord] nexarm-teach-record-{args.variant} READY;')
passed = '[TeachRecordTest] PASS: decode, CRC, capture, stage order, keys' in decoded
protocol_passed = '[ProtocolBoundaryTest] PASS:' in decoded
if args.variant in ('r7', 'r7k2'):
    passed = passed and protocol_passed
key_polarity_passed = '[KeyPolarityTest] PASS:' in decoded
if args.variant == 'r7k2':
    passed = passed and key_polarity_passed
oled = '[OLED] FOUND at 0x3C' in decoded
panic = any(s in decoded for s in ('Guru Meditation', 'Rebooting...', 'Backtrace:', 'Brownout', '[TeachRecordTest] FAIL'))
counts = Counter()
snapshots = []
last_positions = None
last_key_pins = None
errors = []
for line in decoded.splitlines():
    if not line.startswith('{'):
        continue
    try:
        frame = json.loads(line)
        counts[frame.get('type', 'unknown')] += 1
        if frame.get('type') == 'teach_record':
            snapshots.append(decode_snapshot(frame)[0])
        elif frame.get('type') == 'actual65':
            last_positions = frame['pulses']
        elif frame.get('type') == 'key_pins':
            last_key_pins = frame
    except (ValueError, KeyError, TypeError) as error:
        errors.append(str(error))
ok = passed and oled and ready == 1 and not panic and bool(snapshots) and not errors and counts['actual65'] > 0
summary = dict(ok=ok, port=args.port, variant=args.variant, bytes=len(received), self_test=passed, oled_found=oled,
               key_polarity_test=key_polarity_passed,
               protocol_boundary_test=protocol_passed,
               ready_count=ready, panic=panic, counts=dict(counts), errors=errors,
               last_positions=last_positions,
               last_key_pins=last_key_pins,
               last_record=snapshots[-1] if snapshots else None,
               external_servo_power='not measured by this script',
               physical_buttons_tested=False, live_servo_feedback_verified=False)
base.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
raise SystemExit(0 if ok else 1)
