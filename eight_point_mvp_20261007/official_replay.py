"""Eight-point runner using the official host CMD51 joint-slider API.

plan is offline; inspect sends only version/actual-position queries.
Motion requires an explicit subcommand AND --execute. No flashing, torque,
offset, ID, mode, OTA, factory reset, or NVS write command is implemented.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from read_handle_teach import decode_record

HERE = Path(__file__).resolve().parent
SOURCE = ROOT / 'validation_runs/readout_20261005_180008/eight_points.bin'
SOURCE_SHA = '1f5d59c95b96d6346eb3244547a0aa847e4adb74b029b92486e0f6c456744c22'
STEP = 8                 # maximum raw encoder increment per commanded axis
TOL = 3                  # raw encoder counts; not degrees/mm
SPAN = 2
FRESH = 0.6
STEP_TIMEOUT = 6.0
SETTLE_WINDOW = 0.3


class Refused(RuntimeError):
    pass


def points():
    blob = SOURCE.read_bytes()
    if hashlib.sha256(blob).hexdigest() != SOURCE_SHA:
        raise Refused('Teaching backup fingerprint mismatch')
    record = decode_record(blob)
    if not record['complete']:
        raise Refused('Incomplete teaching record')
    return record['poses']


def valid_positions(p):
    return len(p) == 6 and all(type(x) is int and 0 <= x <= 4095 for x in p)


def frame(cmd, payload=b''):
    # Dedicated host interface: only read-only queries and CMD51.
    if cmd not in (1, 65, 51):
        raise Refused('Command outside replay whitelist')
    if cmd in (1, 65) and payload:
        raise Refused('Query must have no arguments')
    if cmd == 51:
        if len(payload) != 5:
            raise Refused('CMD51 needs ID, position and duration')
        sid, pos, duration = struct.unpack('<BhH', payload)
        if not 1 <= sid <= 6 or not 0 <= pos <= 4095 or duration != 1000:
            raise Refused('Invalid slider target or duration')
    body = bytes((255, len(payload)+2, cmd)) + payload
    return b'\xff\xff' + body + bytes(((~sum(body)) & 255,))


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf.extend(data)
        found = []
        while True:
            i = self.buf.find(b'\xff\xff')
            if i < 0:
                self.buf[:] = self.buf[-1:] if self.buf[-1:] == b'\xff' else b''
                break
            del self.buf[:i]
            if len(self.buf) < 4:
                break
            # Only the ESP32 host reply and AT32-forwarded reply are relevant.
            # Reject a shifted FF FF FF header before trusting its length.
            if self.buf[2] not in (255, 90):
                del self.buf[0]
                self.bad += 1
                continue
            n = self.buf[3]
            if not 2 <= n <= 252:
                del self.buf[0]
                self.bad += 1
                continue
            if len(self.buf) < n+4:
                break
            b = bytes(self.buf[:n+4])
            if sum(b[2:]) & 255 != 255:
                del self.buf[0]
                self.bad += 1
                continue
            del self.buf[:n+4]
            found.append((b[2], b[4], b[5:-1]))
        return found


def actual_positions(payload):
    if len(payload) != 24:
        raise Refused('CMD65 expected six (position, angle) pairs')
    p = tuple(struct.unpack_from('<h', payload, 4*i)[0] for i in range(6))
    if not valid_positions(p):
        raise Refused('Invalid actual encoder feedback')
    return p


def path_steps(start, target):
    if not valid_positions(start) or not valid_positions(target):
        raise Refused('Invalid endpoint')
    count = max(1, math.ceil(max(abs(b-a) for a, b in zip(start, target))/STEP))
    return [tuple(a + round((b-a)*i/count) for a, b in zip(start, target))
            for i in range(1, count+1)]


def stable_tail(samples, target=None, after=0):
    tail = [s for s in samples if s[0] > after][-5:]
    if len(tail) != 5 or tail[-1][0]-tail[0][0] < SETTLE_WINDOW:
        return False
    for j in range(6):
        vals = [p[j] for _, p in tail]
        if max(vals)-min(vals) > SPAN:
            return False
        if target is not None and any(abs(v-target[j]) > TOL for v in vals):
            return False
    return True


class Device:
    def __init__(self, port, logdir):
        import serial
        logdir.mkdir(parents=True, exist_ok=False)
        self.logdir = logdir
        self.events = (logdir/'events.jsonl').open('w', encoding='utf-8')
        self.raw = (logdir/'rx.bin').open('wb')
        self.parser = Parser()
        self.samples = deque(maxlen=40)
        self.versions = []
        self.s = serial.Serial(port=None, baudrate=1000000, timeout=0.03, write_timeout=0.5)
        self.s.port = port
        self.s.dtr = False
        self.s.rts = False
        try:
            self.s.open()  # no intentional RTS reset; driver may still reset
        except BaseException:
            self.raw.close()
            self.events.close()
            raise
        self.last_query = 0
        self.touched = set()
        self.motion_sent = False
        self.tx_failed = False
        self.stop_hold_attempted = False
        self.emit('session', port=port, source_sha256=SOURCE_SHA)

    def emit(self, kind, **data):
        row = dict(type=kind, monotonic=time.monotonic(), **data)
        self.events.write(json.dumps(row, ensure_ascii=False)+'\n')
        self.events.flush()
        if kind not in ('actual65','tx_attempt','tx_complete') and (
                kind != 'step_reached' or data['step'] % 10 == 0 or data['step'] == data['total']):
            print(json.dumps(row, ensure_ascii=False), flush=True)

    def send(self, cmd, payload=b''):
        b = frame(cmd, payload)
        # Log intent before write: a write exception may follow partial TX.
        if cmd == 51:
            self.motion_sent = True
            self.touched.add(payload[0])
            self.emit('tx_attempt', cmd=cmd, hex=b.hex())
        try:
            n = self.s.write(b)
            if n != len(b):
                raise Refused('Short UART write; no retry')
        except BaseException:
            self.tx_failed = True
            raise
        if cmd == 51:
            self.emit('tx_complete', cmd=cmd, bytes=n, acknowledged=False)

    def poll(self):
        if (self.logdir/'STOP').exists():
            raise Refused('Operator STOP file')
        if sys.platform == 'win32':
            import msvcrt
            if msvcrt.kbhit() and msvcrt.getwch() in ('q', 'Q', '\x1b'):
                raise Refused('Operator keyboard stop')
        now = time.monotonic()
        if now-self.last_query >= 0.1:
            self.send(65)
            self.last_query = now
        b = self.s.read(max(1, min(self.s.in_waiting, 8192)))
        self.raw.write(b)
        self.raw.flush()
        for sid, cmd, payload in self.parser.feed(b):
            if sid not in (255, 90):
                continue
            if cmd == 65:
                p = actual_positions(payload)
                now = time.monotonic()
                self.samples.append((now, p))
                self.emit('actual65', pulses=p)
            elif cmd == 1:
                self.versions.append(payload.hex())
                self.emit('version', payload_hex=payload.hex())

    def current(self):
        if not self.samples or time.monotonic()-self.samples[-1][0] > FRESH:
            raise Refused('Actual position feedback stale or absent')
        return self.samples[-1][1]

    def baseline(self):
        self.send(1)
        until = time.monotonic()+10
        while time.monotonic() < until:
            self.poll()
            if stable_tail(self.samples):
                return self.current()
        raise Refused('No stable five-frame CMD65 baseline')

    def settle(self, target, before, sent_at, changed):
        until = sent_at+STEP_TIMEOUT
        while time.monotonic() < until:
            self.poll()
            p = self.current()
            for i in range(6):
                if not min(before[i], target[i])-12 <= p[i] <= max(before[i], target[i])+12:
                    raise Refused(f'Joint {i+1} outside step envelope')
            if stable_tail(self.samples, target, sent_at):
                if any(abs(target[j]-before[j]) >= 6 and abs(p[j]-before[j]) < 2 for j in changed):
                    continue
                return p
        raise Refused('Target did not settle; no automatic retry')

    def move_step(self, target):
        before = self.current()
        if not valid_positions(target) or max(abs(a-b) for a,b in zip(before,target)) > STEP+TOL:
            raise Refused('Refusing a target jump larger than one verified step')
        changed = [j for j in range(6) if abs(before[j]-target[j]) > TOL]
        started = time.monotonic()
        for j in changed:
            # Official GUI move_single_servo packs exactly <BhH> for CMD51.
            # Fixed 1000ms avoids the source's 20000/time_ms reaching zero.
            self.send(51, struct.pack('<BhH', j+1, target[j], 1000))
        return self.settle(target, before, started, changed)

    def goto(self, target, label, max_approach):
        start = self.current()
        if max(abs(a-b) for a,b in zip(start,target)) > max_approach:
            raise Refused('Current pose too far from permitted segment start')
        steps = path_steps(start, target)
        self.emit('segment_start', label=label, start=start, target=target, steps=len(steps))
        for k, p in enumerate(steps):
            self.move_step(p)
            self.emit('step_reached', label=label, step=k+1, total=len(steps), actual=self.current())
        self.emit('point_reached', label=label, target=target, actual=self.current())

    def hold_after_abort(self):
        # Best effort target=current for axes touched in THIS session only.
        # Never call a stale sample or a UART submission a verified stop.
        if not self.motion_sent:
            return
        if self.tx_failed or not self.samples or time.monotonic()-self.samples[-1][0] > 0.25:
            self.emit('stop_unverified', action='Support arm and cut servo power if moving; no fresh hold sent')
            return
        self.stop_hold_attempted = True
        p = self.samples[-1][1]
        try:
            for sid in sorted(self.touched):
                self.send(51, struct.pack('<BhH', sid, p[sid-1], 1000))
            self.emit('hold_requested', actual=p, physical_stop_verified=False)
        except Exception as e:
            self.emit('stop_unverified', error=str(e), action='Support arm and cut servo power')

    def close(self):
        self.s.close()
        self.raw.close()
        self.events.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=['plan', 'inspect', 'probe', 'point', 'run'])
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--point', type=int, choices=range(1,9))
    ap.add_argument('--execute', action='store_true')
    a = ap.parse_args()
    taught = points()
    if a.mode == 'plan':
        print(json.dumps(taught, indent=2))
        return
    if a.mode in ('probe','point','run') and not a.execute:
        ap.error('Motion requires --execute and on-site observation')
    if a.mode == 'point' and a.point is None:
        ap.error('point needs --point 1..8')
    run_dir = HERE/'runs'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    print('Log directory: '+str(run_dir), flush=True)
    d = Device(a.port, run_dir)
    result = {'mode':a.mode,'success':False,'full_playback_validated':False,
              'operator_observation':'pending','source_sha256':SOURCE_SHA}
    try:
        baseline = d.baseline()
        result['baseline'] = baseline
        if a.mode == 'inspect':
            d.emit('inspection', actual=baseline, versions=d.versions)
        elif a.mode == 'probe':
            target = list(baseline)
            target[5] -= STEP
            if not 1100 <= target[5] <= 1900:
                raise Refused('ID6 outside conservative taught interior; no probe')
            d.goto(target, 'id6_open_probe', STEP)
            if baseline[5]-d.current()[5] < 4:
                raise Refused('Probe movement not resolved')
        elif a.mode == 'point':
            idx = a.point-1
            # Point1 approach is bounded; later points require the previous
            # taught endpoint, rather than an arbitrary all-joint jump.
            if idx and max(abs(x-y) for x,y in zip(baseline,taught[idx-1]['encoder_positions'])) > 8:
                raise Refused('Start must be within 8 counts of preceding taught point')
            d.goto(taught[idx]['encoder_positions'], taught[idx]['stage'], 350 if idx==0 else 4095)
        else:
            if max(abs(x-y) for x,y in zip(baseline,taught[0]['encoder_positions'])) > 8:
                raise Refused('Full replay must start near verified point1')
            for p in taught:
                d.goto(p['encoder_positions'], p['stage'], 4095)
            result['full_playback_validated'] = False  # needs operator/path acceptance too
            result['eight_targets_reached_by_feedback'] = True
        result['success'] = True
        result['final'] = d.current()
    except (Exception, KeyboardInterrupt) as e:
        result['error'] = str(e) or 'Keyboard interrupt'
        d.emit('aborted', error=result['error'])
        d.hold_after_abort()
    finally:
        result['motion_attempted'] = d.motion_sent
        result['hold_attempted'] = d.stop_hold_attempted
        result['versions'] = d.versions
        result['parser_bad_frames'] = d.parser.bad
        (run_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n', encoding='utf-8')
        d.close()
    print(json.dumps(result, indent=2), flush=True)
    if not result['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
