"""Export r6 free-teach records from a log or passive serial listening.

No serial writes or deliberate reset. A USB driver can still reset the ESP32
on opening the port, so support the arm. Device emits a snapshot every 5 s.
An AT32 actual65 reply alone does not prove powered/live servo feedback.
"""
import argparse
import json
from pathlib import Path
import time
from read_handle_teach import decode_record


def decode_snapshot(frame):
    if frame.get('type') != 'teach_record':
        raise ValueError('Not a teach-record snapshot')
    if frame.get('firmware') not in ('nexarm-teach-record-r6', 'nexarm-teach-record-r6k1', 'nexarm-teach-record-r7', 'nexarm-teach-record-r7k2', 'nexarm-execution-preflight-r1', 'nexarm-execution-preflight-r1b', 'nexarm-single-joint-probe-r1', 'nexarm-single-joint-probe-r1b') or frame.get('namespace') != 'handle_free':
        raise ValueError('Wrong firmware or record namespace')
    if frame.get('storage_fault') is not False or frame.get('playback_enabled') is not False:
        raise ValueError('Storage is uncertain or unexpected playback state')
    payload = bytes.fromhex(frame['record_hex'])
    decoded = decode_record(payload)
    if decoded['valid_mask'] != frame.get('valid_mask'):
        raise ValueError('Snapshot mask does not match record')
    decoded.update(firmware=frame['firmware'], namespace=frame['namespace'],
                   saved_without_lock=True, live_servo_feedback_verified=False,
                   physical_path_validated=False)
    return decoded, payload


def latest_snapshot(data):
    latest = None
    for line in data.decode('utf-8', errors='replace').splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and value.get('type') == 'teach_record':
            latest = value
    if latest is None:
        raise ValueError('No complete r6 snapshot found')
    # Do not silently fall back to an older success after an NVS failure.
    return decode_snapshot(latest)


def listen(port_name, seconds=12):
    import serial
    port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
    port.port = port_name
    port.dtr = False
    port.rts = False
    received = bytearray()
    try:
        port.open()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            received.extend(port.read(max(1, min(port.in_waiting, 8192))))
    finally:
        port.close()
    return bytes(received)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--log', type=Path)
    source.add_argument('--port')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.suffix.lower() != '.json':
        parser.error('--output must have a .json suffix')
    if args.log and args.log.resolve() in {
        args.output.resolve(), args.output.with_suffix('.log').resolve(),
        args.output.with_suffix('.bin').resolve()
    }:
        parser.error('Output paths must differ from the input log')
    data = args.log.read_bytes() if args.log else listen(args.port)
    args.output.with_suffix('.log').write_bytes(data)
    decoded, payload = latest_snapshot(data)
    args.output.with_suffix('.bin').write_bytes(payload)
    args.output.write_text(json.dumps(decoded, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(decoded, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
