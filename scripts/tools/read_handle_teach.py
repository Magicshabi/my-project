"""Read r4 taught endpoints without sending motion, torque, or reset commands.

Opening a USB serial device can still cause a driver-level reset: support the arm
before connecting. The firmware releases torque on startup. --decode needs no
device; --port reads command 105. It does not start playback.
"""
import argparse
import json
import pathlib
import struct
import time
import zlib

STAGES = ["red_above_open", "red_down_open", "red_down_grip", "red_lift_grip",
          "t0_above_grip", "t0_down_grip", "t0_down_open", "t0_clear_open"]


def decode_record(data):
    if len(data) != 108:
        raise ValueError("Expected exactly 108 record bytes")
    magic, version, mask, reserved = struct.unpack_from("<IHBB", data)
    crc, = struct.unpack_from("<I", data, 104)
    if magic != 0x484E4431 or version != 1 or reserved != 0:
        raise ValueError("Unsupported record format")
    if zlib.crc32(data[:104]) != crc:
        raise ValueError("Record CRC mismatch")
    if mask & (mask + 1):
        raise ValueError("Nonsequential teaching mask")
    poses = []
    for index, name in enumerate(STAGES):
        if not mask & (1 << index):
            continue
        joints = struct.unpack_from("<6h", data, 8 + index * 12)
        if any(not 0 <= p <= 4095 for p in joints):
            raise ValueError("Encoder position outside 0..4095")
        poses.append({"slot": index, "stage": name, "joint_ids": [1, 2, 3, 4, 5, 6],
                      "encoder_positions": list(joints)})
    return {"version": version, "valid_mask": mask, "complete": mask == 255,
            "playback_validated": False, "poses": poses}


def extract_record(stream):
    # Search every possible header; mixed ASCII and partial/corrupt frames are normal.
    for i in range(max(0, len(stream) - 5)):
        if stream[i:i+3] != b"\xff\xff\xff":
            continue
        length = stream[i+3]
        if length != 110 or stream[i+4] != 105 or len(stream) < i + 114:
            continue
        packet = stream[i:i+114]
        if ((~sum(packet[2:-1])) & 255) == packet[-1]:
            return bytes(packet[5:-1])
    return None


def read_device(port_name):
    import serial
    port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
    port.port = port_name
    port.dtr = False
    port.rts = False
    received = bytearray()
    try:
        port.open()
        # Wait for normal startup if the USB driver toggled reset on open.
        deadline = time.monotonic() + 10
        next_request = time.monotonic() + 3
        while time.monotonic() < deadline:
            if time.monotonic() >= next_request:
                body = bytes([255, 2, 105])
                port.write(b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
                next_request = time.monotonic() + 1
            received.extend(port.read(max(1, port.in_waiting)))
            payload = extract_record(received)
            if payload is not None:
                decode_record(payload)
                return payload
        raise TimeoutError("No valid r4 command-105 reply; no motion was requested")
    finally:
        port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--decode", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    payload = args.decode.read_bytes() if args.decode else read_device(args.port)
    result = decode_record(payload)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
        args.output.with_suffix(".bin").write_bytes(payload)
    print(text)


if __name__ == "__main__":
    main()
