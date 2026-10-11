"""Read-only pose snapshot for recovery planning.

Queries only (no motion, torque, configuration or NVS writes). Reports the live
joint counts and their distance to each taught point of the powered route. The
controller report is not proof of the physical pose; the operator describes it.
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import threading
import time

import replay_joint_points as r
from capture_joint_points import ReadOnlyTransport, capture_stable
from nexarm import ArmController
from nexarm.eight_point import configuration, same_configuration


def distances(current, targets):
    """Max per-joint distance to each taught point, nearest first."""
    rows = [dict(point=i + 1, max_delta=max(abs(a - b) for a, b in zip(current, t)),
                 delta=[b - a for a, b in zip(current, t)]) for i, t in enumerate(targets)]
    return sorted(rows, key=lambda x: x['max_delta'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--note', default='', help='operator description of the physical pose')
    a = ap.parse_args()
    raw = r.SOURCE.read_bytes()
    route = json.loads(raw.decode('utf-8'))
    limits = r.load_limits()
    targets = r.resolve_route(route, limits)
    r.require_device_lease(a.port)
    now = datetime.now(timezone(timedelta(hours=8)))
    run = r.HERE / 'logs' / ('pose_' + now.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)
    result = dict(started_at=now.isoformat(), port=a.port, read_only=True, motion_commands=0,
                  operator_note=a.note, source_sha256=r.hashlib.sha256(raw).hexdigest(),
                  limits_sha256=r.sha(r.DEFAULT_LIMITS))
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        lock = threading.Lock()

        def emit(event, **data):
            with lock:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = ReadOnlyTransport(emit)
        transport.on_any(lambda f: emit('rx', id=f.id, cmd=f.cmd, args=f.args.hex()))
        arm = ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        try:
            arm.connect_serial(a.port)
            config = configuration(arm)
            result.update(configuration=config,
                          configuration_matches_route=same_configuration(route['configuration'], config),
                          battery_mv=arm.get_battery_mv())
            counts, samples = capture_stable(arm, emit)
            result.update(counts=counts, controller_pose=samples[-1]['controller_pose'],
                          within_software_limits=all(lo <= v <= hi for v, (lo, hi) in zip(counts, limits)),
                          nearest_points=distances(counts, targets),
                          checksum_errors=transport.stats()['checksum_errors'])
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            arm.close()
            result.update(serial_closed=True, stats=transport.stats())
            (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    brief = {k: result.get(k) for k in ('counts', 'battery_mv', 'configuration_matches_route',
                                         'within_software_limits', 'error')}
    brief['nearest'] = [(x['point'], x['max_delta']) for x in result.get('nearest_points', [])[:3]]
    print('POSE=' + json.dumps(brief, ensure_ascii=False), flush=True)
    print('RESULT_FILE=' + str(run / 'result.json'), flush=True)
    return 0 if 'error' not in result else 2


if __name__ == '__main__':
    raise SystemExit(main())
