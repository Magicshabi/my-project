"""Re-enable servo torque without moving the arm (supervised, empty load).

Background (2026-10-11 18:06): after a power cycle ID2 could be turned by hand
while the other joints held, and a CMD3 goal write to ID2 produced no motion.
HX-30HM servos drive to their Goal Position register when torque is enabled,
so this tool first writes every servo's goal = its current reading at a low
speed (zero-motion writes), then sends the vendor CMD61 SET_TORQUE(1), which the
factory firmware turns into one broadcast REG_TORQUE_ENABLE=1 on the servo bus.
It then re-reads and reports any joint change. The port is closed before the
operator is asked whether ID2 now resists by hand.

Not a cause diagnosis: protection flags/temperature are not read here.
"""
import argparse
from collections import deque
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import threading
import time

import replay_joint_points as r
import reposition_empty as setup
from nexarm import ArmController
from nexarm.eight_point import configuration, same_configuration

HOLD_ACC = 20
HOLD_SPEED = 300           # slow, in case a goal register differs from the reading
MAX_DRIFT = 12             # counts a joint may change across the whole procedure
TORQUE_ON = b'\x01'


class TorqueTransport(r.ReplayTransport):
    """ReplayTransport plus exactly one CMD61 SET_TORQUE(1); never torque off."""

    def __init__(self, emit, cancelled, limits=None):
        super().__init__(emit, cancelled, limits)
        self.torque_allowed = False
        self.torque_sent = 0

    def send(self, cmd, args=b'', target_id=255):
        if cmd == 61:
            if target_id != 255 or bytes(args) != TORQUE_ON or not self.torque_allowed:
                raise r.SafetyError('只允许一次CMD61力矩开启')
            if self.failed or self.cancelled():
                raise r.SafetyError('已请求停止或通信失败，不再下发')
            self.torque_allowed = False
            self.emit('tx_attempt', cmd=cmd, id=target_id, wire_hex=r.build_frame(cmd, args, target_id).hex())
            r.Transport.send(self, cmd, args, target_id)
            self.torque_sent += 1
            self.emit('tx_complete', cmd=cmd, id=target_id)
            return
        super().send(cmd, args, target_id)


def hold_frames(counts):
    return [(sid, r.build_frame(3, r.pack_joint_counts(v, HOLD_ACC, HOLD_SPEED), sid))
            for sid, v in enumerate(counts, 1)]


def drift(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def restore(arm, transport, emit, limits):
    before, _ = r.capture_stable(arm, emit)
    r.check_counts(before, limits, '恢复前姿态')
    frames = hold_frames(before)
    transport.pending = deque(f for _, f in frames)
    emit('hold_goals', goals=before, acc=HOLD_ACC, speed=HOLD_SPEED)
    for sid, value in zip(range(1, 7), before):
        arm.move_joint_counts(sid, value, HOLD_ACC, HOLD_SPEED)
    if transport.pending:
        raise r.SafetyError('存在未发送的保持帧')
    time.sleep(0.5)
    mid, _ = r.capture_stable(arm, emit)
    if drift(before, mid) > MAX_DRIFT:
        raise r.SafetyError('写入保持目标后姿态变化%d刻度；不开力矩' % drift(before, mid))
    transport.torque_allowed = True
    arm.set_torque(True)
    time.sleep(1.0)
    after, _ = r.capture_stable(arm, emit)
    change = drift(before, after)
    if change > MAX_DRIFT:
        raise r.SafetyError('开力矩后姿态变化%d刻度' % change)
    return dict(before=before, after_hold_goals=mid, after_torque=after, max_change=change,
                torque_command_sent=transport.torque_sent == 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--empty-load', action='store_true')
    a = ap.parse_args()
    if not (a.go and a.empty_load):
        ap.error('需现场确认空载、有人扶住大臂并显式--go --empty-load')
    raw = r.SOURCE.read_bytes()
    route = json.loads(raw.decode('utf-8'))
    limits = r.load_limits()
    lease_path, lease_sha, expires = r.require_device_lease(a.port)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('torque_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    (run / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    result = dict(started_at=now.isoformat(), port=a.port, empty_load=True, complete=False,
                  operator_confirmed=False, cause_diagnosed=False)

    def save():
        (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def cancelled():
        try:
            return ((run / 'STOP').exists() or datetime.now(timezone.utc) >= expires
                    or r.sha(lease_path) != lease_sha)
        except OSError:
            return True
    save()
    print('RUN_DIR=' + str(run), flush=True)
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        log_lock = threading.Lock()

        def emit(kind, **data):
            with log_lock:
                log.write(json.dumps(dict(event=kind, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = TorqueTransport(emit, cancelled, limits)
        arm = ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        try:
            arm.connect_serial(a.port)
            live = configuration(arm)
            if not same_configuration(route['configuration'], live):
                raise r.SafetyError('设备配置改变')
            battery = arm.get_battery_mv()
            if battery is None:
                raise r.SafetyError('缺少供电报告')
            result.update(battery_mv=battery)
            arm.enable_motion(True)
            result['torque'] = restore(arm, transport, emit, limits)
            save()
            print('TORQUE_DONE=' + json.dumps(result['torque'], ensure_ascii=False), flush=True)
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
            emit('aborted', reason=result['error'])
            print('STOP: 停止后续写入；无重试。', flush=True)
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            result.update(motion_attempts=transport.motion_attempts, torque_commands=transport.torque_sent,
                          serial_closed=True, stats=transport.stats())
            save()
    if 'torque' in result and 'error' not in result:
        answer = setup.ask_observation('OBSERVE: 串口已关闭。请用户轻推大臂：yes=ID2已掰不动且整臂无异动；'
                                       '否则原样输入用户描述（含夹爪开/合）\n')
        result.update(operator_answer=answer, operator_confirmed=answer == 'yes', complete=answer == 'yes')
        save()
    print('RESULT=' + json.dumps(dict(complete=result['complete'], error=result.get('error'),
                                      torque=result.get('torque'), result_file=str(run / 'result.json')),
                                 ensure_ascii=False), flush=True)
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
