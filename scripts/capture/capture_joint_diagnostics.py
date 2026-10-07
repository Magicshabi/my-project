"""Read-only r5 feedback capture. Support the arm before opening USB serial.

No torque, target-position, motion or deliberate reset commands are sent.
Default capture writes command-65 actual pulses and command-11 status separately.
"""
import argparse
import json
import pathlib
import struct
import time


def take_frames(buf):
    frames = []
    while len(buf) >= 6:
        start = buf.find(b"\xff\xff")
        if start < 0:
            del buf[:-1]
            break
        if start:
            del buf[:start]
        if len(buf) < 6:
            break
        if buf[2] != 255:  # ESP32 forwards its responses using the host ID.
            del buf[0]
            continue
        length = buf[3]
        if not 2 <= length <= 250:
            del buf[0]
            continue
        total = length + 4
        if len(buf) < total:
            break
        frame = bytes(buf[:total])
        if ((~sum(frame[2:-1])) & 255) != frame[-1]:
            del buf[0]
            continue
        frames.append((frame[2], frame[4], frame[5:-1]))
        del buf[:total]
    return frames


def describe_feedback(cmd, data):
    if cmd == 65 and len(data) == 24:
        pairs = struct.unpack("<12h", data)
        pulses = list(pairs[::2])
        return {"source": "cmd65_actual", "pulses": pulses,
                "angles_deg": [v / 10 for v in pairs[1::2]],
                "range_ok": all(0 <= v <= 4095 for v in pulses)}
    if cmd == 11 and len(data) == 24:
        status = struct.unpack("<12h", data)
        return {"source": "cmd11_status", "pose_raw": list(status[:6]),
                "pulses": list(status[6:])}
    return None


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("seconds must be 1..300")
    port = serial.Serial(port=None, baudrate=1000000, timeout=0.05)
    port.port = args.port
    port.dtr = False
    port.rts = False
    raw = bytearray()
    buf = bytearray()
    records = []
    try:
        port.open()
        start = time.monotonic()
        next_query = start + 3
        while time.monotonic() - start < args.seconds:
            if time.monotonic() >= next_query:
                body = bytes([255, 2, 65])
                port.write(b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
                next_query = time.monotonic() + 0.25
            chunk = port.read(max(1, port.in_waiting))
            raw.extend(chunk)
            buf.extend(chunk)
            for identity, command, data in take_frames(buf):
                if identity != 255:
                    continue
                record = describe_feedback(command, data)
                if record is not None:
                    record["t_seconds"] = round(time.monotonic() - start, 3)
                    records.append(record)
    finally:
        port.close()
        args.output.with_suffix(".bin").write_bytes(raw)
        args.output.with_suffix(".log").write_text(raw.decode("utf-8", errors="replace"), encoding="utf-8")
    args.output.write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    actual = [r for r in records if r["source"] == "cmd65_actual"]
    status = [r for r in records if r["source"] == "cmd11_status"]
    print(json.dumps({"actual_frames": len(actual), "status_frames": len(status),
                      "latest_actual": actual[-1] if actual else None,
                      "latest_status": status[-1] if status else None}, indent=2))
    raise SystemExit(0 if len(actual) >= 5 and all(r["range_ok"] for r in actual) else 1)


if __name__ == "__main__":
    main()
