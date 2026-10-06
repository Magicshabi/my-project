"""Reset COM9 and capture diagnostic firmware at its application baud rate.

Run only after the operator supports the arm: diagnostic startup releases torque.
"""
import argparse
import pathlib
import time
import serial
import re

parser = argparse.ArgumentParser()
parser.add_argument('--port', default='COM9')
parser.add_argument('--seconds', type=float, default=30)
parser.add_argument('--version', default='1.0.2-match-safe-r2')
parser.add_argument('--output', default='match_safe_r2_boot.log')
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
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        received.extend(port.read(max(1, port.in_waiting)))
finally:
    port.close()
path = pathlib.Path(__file__).with_name(args.output)
text = received.decode('utf-8', errors='replace')
path.with_suffix('.bin').write_bytes(received)
path.write_text(text, encoding='utf-8')
for run in re.findall(r'[\x20-\x7e\r\n]{8,}', text):
    print(run)
ready = text.count('Color sort firmware ' + args.version + ' ready')
passed = '[ProtocolTest] PASS:' in text
manual = '[PoseCal] Torque OFF: startup requires manual confirmation' in text
panic = any(s in text for s in ['Guru Meditation', 'Rebooting...', 'Backtrace:', 'Brownout', '[ProtocolTest] FAIL'])
print(f'Captured {len(received)} bytes. Self-test={passed}, manual={manual}, ready_count={ready}, panic={panic}')
raise SystemExit(0 if passed and manual and ready == 1 and not panic else 1)
