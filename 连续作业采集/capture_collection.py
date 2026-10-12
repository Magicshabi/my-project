"""Read-only powered teaching for this collection. Never sends movement commands."""
import argparse
import copy
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
import collection as c


def add_point(doc, root, color, target, point, measured, samples, config, grip_target, observed, evidence):
    config = json.loads(json.dumps(config))
    c.validate(doc, root, color, target)
    c.require(point == len(doc['points']) + 1 and 1 <= point <= (1 if color is None else 8),
              '只能追加下一个点，不覆盖、不跳号')
    if doc['configuration'] is not None:
        c.require(doc['configuration'] == config, '设备配置改变')
    carrying = color is not None and 4 <= point <= 6
    if carrying:
        inherited = doc['points'][2]['gripper_target_counts']
        c.require(grip_target is None or grip_target == inherited, '搬运途中不能改夹紧目标')
        grip_target = inherited
    c.require(type(grip_target) is int, '必须明确输入上位机实际设置的ID6目标刻度')
    out = copy.deepcopy(doc)
    out['configuration'] = config
    out['points'].append(dict(point=point, stage='home' if color is None else c.STAGES[point-1],
        encoder_positions=measured, gripper_target_counts=grip_target, keep_gripper_goal=carrying,
        samples=samples, captured_at=c.now(), evidence_dir=str(evidence),
        observation=observed, physical_pose_confirmed=True, motor_driven_pose_confirmed_by_operator=True))
    out['complete'] = len(out['points']) == (1 if color is None else 8)
    c.validate(out, root, color, target)
    return out


def capture(a):
    # Import existing protocol/query primitives only after CLI arguments pass.
    sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
    import replay_joint_points as r
    import capture_joint_points as old
    path = c.route_path(a.root, a.color, a.target)
    original = path.read_bytes()
    doc = c.read(path)
    c.validate(doc, a.root, a.color, a.target)
    expected = 1 if a.home else 8
    c.require(1 <= a.point <= expected and a.point == len(doc['points']) + 1, '只能依次采集未保存的点')
    c.require(a.powered_sliders_confirmed and a.observed.strip(), '需要本次带电摆位完成依据')
    carrying = not a.home and 4 <= a.point <= 6
    grip = doc['points'][2]['gripper_target_counts'] if carrying else a.grip_target
    c.require(type(grip) is int and c.limits()[5][0] <= grip <= c.limits()[5][1], '需要有效ID6目标')
    c.require(not carrying or a.grip_target in (None, grip), '搬运点必须继承夹紧目标')
    lease, lease_hash, expires = r.require_device_lease(a.port, holder=a.holder)
    scene_hash, limits_hash = c.sha(a.root / 'scene.json'), c.sha(c.LIMITS_FILE)
    run = a.root / 'logs' / ('capture_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns()))
    run.mkdir(parents=True)
    (run / 'input_snapshot.json').write_bytes(original)
    result = dict(point=a.point, color=a.color, destination=a.target, saved=False,
                  started_at=c.now(), motion_commands=0, serial_closed=False, file=str(path),
                  scene_sha256=scene_hash, limits_sha256=limits_hash,
                  sources={str(p): c.sha(p) for p in (Path(__file__), Path(c.__file__),
                           Path(old.__file__), Path(r.__file__))})

    def check_context():
        c.require(datetime.now(timezone.utc) < expires and c.sha(lease) == lease_hash, '设备锁过期或改变')
        c.require(c.sha(a.root / 'scene.json') == scene_hash and c.sha(c.LIMITS_FILE) == limits_hash,
                  '采集期间场地或软件限位改变')
        c.require(path.read_bytes() == original, '采集文件已由其他进程修改')

    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        mutex = threading.Lock()
        def emit(event, **data):
            with mutex:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False)+'\n')
                log.flush()

        class LeasedReadOnlyTransport(old.ReadOnlyTransport):
            def send(self, cmd, args=b'', target_id=255):
                check_context()
                return super().send(cmd, args, target_id)

        transport = LeasedReadOnlyTransport(emit)
        arm = old.ArmController(transport=transport, log=lambda message: emit('library', message=message))
        try:
            check_context()
            arm.connect_serial(a.port)
            config = json.loads(json.dumps(old.configuration(arm)))
            for other_path, color, target in c.paths(a.root):
                other = c.read(other_path)
                c.validate(other, a.root, color, target)
                if other['points']:
                    c.require(other['configuration'] == config, '跨路线设备配置改变')
            battery = arm.get_battery_mv()
            c.require(battery is not None, '缺少供电报告')
            measured, samples = old.capture_stable(arm, emit)
            c.require(transport.stats()['checksum_errors'] == 0, '采集有坏帧')
            updated = add_point(doc, a.root, a.color, a.target, a.point, measured, samples, config,
                                a.grip_target, a.observed, run)
            check_context()
            old.save_revision(path, updated, original)
            result.update(saved=True, encoder_positions=measured, battery_mv=battery,
                          sha256=c.sha(path), configuration=config)
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            arm.close()
            result.update(serial_closed=True, stats=transport.stats())
            (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('RUN_DIR=' + str(run))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['saved'] else 1


def batch_capture(a):
    """Batch mode: capture a route or home point with interactive prompts."""
    sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
    import replay_joint_points as r

    # Acquire lock once for entire batch session
    lease, lease_hash, expires = r.require_device_lease(a.port, holder=a.holder)
    print(f'已取得 {a.port} 锁（{a.holder}），有效期至 {expires.strftime("%H:%M:%S")}')

    if a.home:
        points_to_capture = [1]
        route_name = 'P0初始位'
    else:
        path = c.route_path(a.root, a.color, a.target)
        doc = c.read(path)
        c.validate(doc, a.root, a.color, a.target)
        existing = len(doc['points'])
        points_to_capture = list(range(existing + 1, 9))
        route_name = f'{a.color}→{a.target}'
        if not points_to_capture:
            print(f'✓ {route_name} 已完成（8/8）')
            return 0
        print(f'\n开始批量采集：{route_name}（当前 {existing}/8）')
        print(f'  张开目标（S1/S2/S7/S8）: {a.grip_open}')
        print(f'  夹紧目标（S3及S4/S5/S6继承）: {a.grip_close}')
        input('\n按回车开始... ')

    try:
        for point in points_to_capture:
            while True:
                stage = 'home' if a.home else c.STAGES[point - 1]
                carrying = not a.home and 4 <= point <= 6

                # Determine grip target for this point
                if a.home or point in (1, 2, 7, 8):
                    grip = a.grip_open if not a.home else a.grip_target
                elif point == 3:
                    grip = a.grip_close
                else:  # points 4-6
                    grip = None  # Will inherit from S3

                print(f'\n→ S{point} {stage}', end='')
                if carrying:
                    print(f'（继承S3夹紧目标）')
                elif grip:
                    print(f'（夹爪目标={grip}）')
                else:
                    print()

                obs = input('  摆好位置、断开上位机后按回车（或输入本点观察依据）: ').strip()
                if not obs:
                    obs = a.observed or f'{route_name} S{point} 用户确认摆位完成'

                # Build args for single capture
                cap_args = argparse.Namespace(
                    root=a.root, home=a.home, color=a.color, target=a.target,
                    point=point, port=a.port, holder=a.holder,
                    powered_sliders_confirmed=True, observed=obs, grip_target=grip
                )

                exit_code = capture(cap_args)

                if exit_code == 0:
                    # Show captured data
                    path_to_check = c.route_path(a.root, a.color, a.target)
                    doc_check = c.read(path_to_check)
                    saved_point = doc_check['points'][-1]
                    pos = saved_point['encoder_positions']
                    grip_tgt = saved_point['gripper_target_counts']
                    print(f'\n✓ 采集成功：{route_name} S{point}')
                    print(f'  ID1:{pos[0]:5d}  ID2:{pos[1]:5d}  ID3:{pos[2]:5d}  ID4:{pos[3]:5d}  ID5:{pos[4]:5d}  ID6:{pos[5]:5d}')
                    print(f'  夹爪目标: {grip_tgt}')

                    action = input('\n  [回车]继续  [r]重采此点  [s]跳过  [q]退出: ').strip().lower()
                    if action == 'r':
                        print('  → 准备重新采集此点，请重新摆位')
                        continue  # Re-capture this point
                    elif action == 's':
                        print(f'  → 跳过S{point}，稍后需单独补采')
                        break  # Move to next point without saving
                    elif action == 'q':
                        print('\n已退出批量采集')
                        return 0
                    else:  # Enter or any other key
                        break  # Move to next point
                else:
                    print(f'\n✗ 采集失败')
                    action = input('  [r]重采  [s]跳过  [q]退出: ').strip().lower()
                    if action == 'r':
                        continue
                    elif action == 's':
                        break
                    else:
                        return exit_code

        # Report completion
        if a.home:
            print(f'\n✓ {route_name} 完成')
        else:
            path = c.route_path(a.root, a.color, a.target)
            doc = c.read(path)
            print(f'\n✓ {route_name} 完成（{len(doc["points"])}/8）')

        return 0
    finally:
        # Release lock
        try:
            import subprocess
            subprocess.run([sys.executable, str(c.ROOT.parent / 'handoff' / 'lock.py'),
                          'release', a.port, '--holder', a.holder], check=False)
            print(f'\n已释放 {a.port} 锁')
        except Exception as e:
            print(f'\n警告：释放锁失败 - {e}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=c.ROOT, help='独立采集目录；默认本目录')
    sub = ap.add_subparsers(dest='mode', required=True)

    init = sub.add_parser('init', help='仅创建独立主文件，不连接串口')
    init.add_argument('--layout-id', required=True)

    cap = sub.add_parser('capture', help='单点采集')
    cap.add_argument('--home', action='store_true')
    cap.add_argument('--color', choices=c.COLORS)
    cap.add_argument('--target', choices=c.TARGETS)
    cap.add_argument('--point', type=int, required=True, choices=range(1, 9))
    cap.add_argument('--port', required=True, choices=('COM11',))
    cap.add_argument('--holder', required=True, choices=('claw', 'codex', 'claude'))
    cap.add_argument('--powered-sliders-confirmed', action='store_true')
    cap.add_argument('--observed', required=True)
    cap.add_argument('--grip-target', type=int, help='上位机实际ID6目标刻度；第4~6点自动继承第3点')

    batch = sub.add_parser('batch', help='批量连续采集（整条路线或P0）')
    batch.add_argument('--home', action='store_true')
    batch.add_argument('--color', choices=c.COLORS)
    batch.add_argument('--target', choices=c.TARGETS)
    batch.add_argument('--port', required=True, choices=('COM11',))
    batch.add_argument('--holder', required=True, choices=('claw', 'codex', 'claude'))
    batch.add_argument('--grip-open', type=int, help='张开目标（S1/S2/S7/S8及P0）')
    batch.add_argument('--grip-close', type=int, help='夹紧目标（S3，S4/S5/S6自动继承）')
    batch.add_argument('--grip-target', type=int, help='P0模式的夹爪目标')
    batch.add_argument('--observed', help='默认观察依据（可在每点覆盖）')

    a = ap.parse_args(argv)
    try:
        if a.mode == 'init':
            c.init(a.root, a.layout_id)
            print('已初始化P0和12条路线；当前0/97点，不代表已采集')
            return 0

        if a.mode == 'batch':
            c.require((a.home and a.color is None and a.target is None)
                      or (not a.home and a.color in c.COLORS and a.target in c.TARGETS), '请选择P0或颜色+目标')
            if a.home:
                c.require(a.grip_target is not None, 'P0需要--grip-target')
            else:
                c.require(a.grip_open is not None and a.grip_close is not None,
                          '路线需要--grip-open和--grip-close')
            return batch_capture(a)

        # capture mode
        c.require((a.home and a.color is None and a.target is None)
                  or (not a.home and a.color in c.COLORS and a.target in c.TARGETS), '请选择P0或颜色+目标')
        return capture(a)
    except (ValueError, OSError, RuntimeError) as exc:
        print('ERROR=' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
