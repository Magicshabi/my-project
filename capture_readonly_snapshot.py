"""Listen to read-only firmware JSON; no application writes or deliberate reset."""
import argparse
import json
from pathlib import Path
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument('--label', required=True, choices=('before', 'after'))
parser.add_argument('--seconds', type=int, default=10, choices=range(3, 61))
parser.add_argument('--port', default='COM9', help='Current ESP32 port; enumerate before opening')
args = parser.parse_args()
port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
port.port = args.port
port.dtr = False
port.rts = False
raw = bytearray()
try:
    port.open()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        raw.extend(port.read(max(1, port.in_waiting)))
finally:
    port.close()

stamp = time.strftime('%Y%m%d_%H%M%S')
base = Path(__file__).parent / f'readonly_{args.label}_{stamp}'
base.with_suffix('.bin').write_bytes(raw)
text = raw.decode('utf-8', errors='replace')
base.with_suffix('.log').write_text(text, encoding='utf-8')
frames = []
for line in text.splitlines():
    try:
        value = json.loads(line)
    except json.JSONDecodeError:
        continue
    if not isinstance(value, dict):
        continue
    pulses = value.get('pulses')
    if (value.get('type') in ('actual65', 'status11') and
            isinstance(pulses, list) and len(pulses) == 6 and
            all(type(p) is int and 0 <= p <= 4095 for p in pulses)):
        frames.append(value)
summary = {'label': args.label, 'port': args.port, 'log': str(base.with_suffix('.log')), 'data': {}}
for kind in ('actual65', 'status11'):
    rows = [f for f in frames if f['type'] == kind]
    summary['data'][kind] = {
        'count': len(rows), 'first': rows[0] if rows else None,
        'last': rows[-1] if rows else None,
        'ranges': [[min(r['pulses'][i] for r in rows), max(r['pulses'][i] for r in rows)]
                   for i in range(6)] if rows else []}
base.with_suffix('.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary['data']['actual65']['count'] >= 3 else 1)
