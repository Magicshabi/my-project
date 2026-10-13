"""Read-only: compare the live device configuration with the one recorded at P0 capture.

Query-only transport (refuses motion/torque/config writes). Needs a valid COM11 lease.
  python check_config.py --port COM11 --holder claude
"""
import argparse
import json
import sys
import threading
import time

import collection as c
import executor as ex

r = ex.r
sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
import capture_joint_points as old  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, choices=('COM11',))
    ap.add_argument('--holder', required=True, choices=('claw', 'codex', 'claude'))
    a = ap.parse_args(argv)
    r.require_device_lease(a.port, holder=a.holder)
    recorded = c.read(c.route_path(c.ROOT))['configuration']
    mutex = threading.Lock()
    transport = old.ReadOnlyTransport(lambda *x, **k: None)
    arm = old.ArmController(transport=transport, log=lambda m: None)
    try:
        arm.connect_serial(a.port)
        live = json.loads(json.dumps(old.configuration(arm)))
        pose, _ = old.capture_stable(arm, lambda *x, **k: None)
    finally:
        arm.close()
    same = True
    for k in sorted(set(live) | set(recorded)):
        if json.dumps(live.get(k), sort_keys=True) != json.dumps(recorded.get(k), sort_keys=True):
            same = False
            print(f'{k}:\n  P0采集时={recorded.get(k)}\n  当前    ={live.get(k)}')
    print('配置一致' if same else '配置不同（见上）')
    print('当前姿态=' + json.dumps(pose))
    return 0 if same else 2


if __name__ == '__main__':
    sys.exit(main())
