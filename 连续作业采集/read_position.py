"""Read-only position snapshot for the continuous-collection task.

Queries only; never sends motion, torque, configuration or NVS writes.
Reuses the 八点推进_20261008 query protocol and stable-sampling logic, so the
counts it reports follow the same definition as a would-be capture.

Prerequisites: host serial released, and a valid device lease acquired first:

    python handoff/lock.py acquire COM11 --holder claw --minutes 30 --purpose '位置读取'
    python 连续作业采集/read_position.py --port COM11 --holder claw --note '现场说明（可选）'
    python handoff/lock.py release COM11 --holder claw

Runtime env (same as capture): PYTHONPATH=仓库/.runtime_deps/serial_capture，PYTHONIOENCODING=utf-8。
Prints one 'POSITION=' JSON line and 'RUN_DIR='; raw evidence stays under 连续作业采集/logs/read_*.
"""
import argparse
from datetime import datetime
import json
import sys
import threading
import time
from pathlib import Path

import collection as c


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, choices=('COM11',))
    ap.add_argument('--holder', required=True, choices=('claw', 'codex', 'claude'))
    ap.add_argument('--note', default='', help='本次读取的现场说明（可选）')
    a = ap.parse_args(argv)

    # Import existing protocol/query primitives only after CLI arguments pass.
    sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
    import replay_joint_points as r
    import capture_joint_points as old

    try:
        lease, lease_hash, _ = r.require_device_lease(a.port, holder=a.holder)
    except Exception as exc:
        print('POSITION=' + json.dumps({'error': str(exc) or type(exc).__name__}, ensure_ascii=False), flush=True)
        return 1

    run = c.ROOT / 'logs' / ('read_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns()))
    run.mkdir(parents=True)
    result = dict(started_at=c.now(), port=a.port, holder=a.holder, read_only=True, motion_commands=0,
                  operator_note=a.note, lease=str(lease), lease_sha256=lease_hash,
                  sources={str(p): c.sha(p) for p in (Path(__file__), Path(old.__file__), Path(r.__file__))})
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        mutex = threading.Lock()

        def emit(event, **data):
            with mutex:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()

        transport = old.ReadOnlyTransport(emit)
        transport.on_any(lambda f: emit('rx', id=f.id, cmd=f.cmd, args=f.args.hex()))
        arm = old.ArmController(transport=transport, log=lambda message: emit('library', message=message))
        try:
            arm.connect_serial(a.port)
            battery = arm.get_battery_mv()
            counts, samples = old.capture_stable(arm, emit)
            if transport.stats()['checksum_errors']:
                raise old.SafetyError('读取出现坏帧；结果不可用')
            span = max(max(s['cmd65_counts'][i] for s in samples)
                       - min(s['cmd65_counts'][i] for s in samples) for i in range(6))
            limits = c.limits()
            violations = [dict(servo_id=i + 1, value=v, lower=lo, upper=hi)
                          for i, (v, (lo, hi)) in enumerate(zip(counts, limits)) if not lo <= v <= hi]
            result.update(counts=counts, controller_pose=samples[-1]['controller_pose'], battery_mv=battery,
                          span_max=span, within_limits=not violations, violations=violations,
                          samples=len(samples), checksum_errors=transport.stats()['checksum_errors'])
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            arm.close()
            result.update(serial_closed=True, stats=transport.stats())
            (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    if 'error' in result:
        brief = {'error': result['error']}
    else:
        brief = {'counts': result['counts'], 'pose': result['controller_pose'], 'battery_mv': result['battery_mv'],
                 'span_max': result['span_max'], 'within_limits': result['within_limits']}
        if result['violations']:
            brief['violations'] = result['violations']
    print('POSITION=' + json.dumps(brief, ensure_ascii=False), flush=True)
    print('RUN_DIR=' + str(run), flush=True)
    return 0 if 'error' not in result else 1


if __name__ == '__main__':
    sys.exit(main())
