"""Capture one restored ESP32 boot; no host motion commands.

If the factory application attempts an AT32 update, return ESP32 to its ROM
loader during the documented three-second pre-erase delay. This restoration
is for the ESP32 application, not an unverified AT32 rewrite.
"""
import json
from pathlib import Path
import re
import time
import serial

dest = Path(__file__).resolve().parent/'backups/before_motion_restore_20261007'
s = serial.Serial(port=None, baudrate=1000000, timeout=0.02, write_timeout=0.5)
s.port = 'COM9'
s.dtr = False
s.rts = True
data = bytearray()
guard = False
try:
    s.open()
    s.dtr = False
    s.rts = True
    time.sleep(0.15)
    s.rts = False
    end = time.monotonic()+30
    while time.monotonic() < end:
        chunk = s.read(max(1, min(s.in_waiting, 8192)))
        if not chunk:
            continue
        data.extend(chunk)
        if b'[OTA] Upgrading...' in data or b'[OTA] Step1: CMD 90' in data:
            # Same RTS/DTR sequence as the standard ESP32 ROM-loader entry.
            s.dtr = False
            s.rts = True
            time.sleep(0.1)
            s.dtr = True
            s.rts = False
            time.sleep(0.1)
            s.dtr = False
            guard = True
            break
finally:
    s.close()
    (dest/'restored_boot.bin').write_bytes(data)

lines = [m.decode('ascii') for m in re.findall(rb'[\x20-\x7e]{5,}', data)]
selected = [line for line in lines if any(tag in line for tag in
    ('[OTA]', '[PID]', '[LittleFS]', '[PS3 Init]', '[BT]', 'BLE mode',
     'Guru Meditation', 'Backtrace', 'Brownout', 'abort()', 'rst:'))]
result = dict(bytes=len(data), at32_update_guard_triggered=guard,
              selected_boot_lines=selected, motion_commands_sent=False,
              external_servo_power='user_confirmed_disconnected')
(dest/'restored_boot.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result, indent=2), flush=True)
if guard:
    raise SystemExit(2)
