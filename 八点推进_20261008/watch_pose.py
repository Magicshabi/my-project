"""Read-only feedback liveness check while the operator moves a joint by hand.

Polls CMD11/CMD65 for a fixed window (queries only, no motion/torque/config
writes) and reports, per joint, the min/max and how many distinct values were
seen. A joint the operator visibly moves but whose reading never changes has a
frozen (cached) report, i.e. the controller is not getting fresh data from it.
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import threading
import time

import replay_joint_points as r
from capture_joint_points import ReadOnlyTransport
from nexarm import ArmController


def summarize(rows):
    out = []
    for i in range(6):
        values = [row[i] for row in rows]
        out.append(dict(joint=i + 1, min=min(values), max=max(values), span=max(values) - min(values),
                        distinct=len(set(values))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--note', default='', help='what the operator moved by hand')
    a = ap.parse_args()
    if not 5 <= a.seconds <= 60:
        ap.error('--seconds 5..60')
    r.require_device_lease(a.port)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('watch_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    result = dict(started_at=now.isoformat(), port=a.port, read_only=True, motion_commands=0,
                  seconds=a.seconds, operator_note=a.note)
    rows11, rows65, missing = [], [], 0
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        lock = threading.Lock()

        def emit(event, **data):
            with lock:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = ReadOnlyTransport(emit)
        arm = ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        try:
            arm.connect_serial(a.port)
            print('WATCHING %.0fs: 现在请用户用手缓慢推动要检查的关节' % a.seconds, flush=True)
            end = time.monotonic() + a.seconds
            while time.monotonic() < end:
                pose, real = arm.get_pose(), arm.get_real_joints()
                if pose is None or real is None or len(real) != 6:
                    missing += 1
                else:
                    a11, a65 = list(pose.servos), [x[0] for x in real]
                    rows11.append(a11)
                    rows65.append(a65)
                    emit('sample', cmd11=a11, cmd65=a65)
                time.sleep(0.15)
            result.update(samples=len(rows11), missing_replies=missing,
                          cmd11=summarize(rows11) if rows11 else None,
                          cmd65=summarize(rows65) if rows65 else None,
                          checksum_errors=transport.stats()['checksum_errors'])
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            arm.close()
            result.update(serial_closed=True, stats=transport.stats())
            (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    spans = [(j['joint'], j['min'], j['max'], j['distinct']) for j in (result.get('cmd65') or [])]
    print('WATCH=' + json.dumps(dict(samples=result.get('samples'), missing=missing, error=result.get('error'),
                                     joint_min_max_distinct=spans), ensure_ascii=False), flush=True)
    print('RESULT_FILE=' + str(run / 'result.json'), flush=True)
    return 0 if 'error' not in result else 2


if __name__ == '__main__':
    raise SystemExit(main())
