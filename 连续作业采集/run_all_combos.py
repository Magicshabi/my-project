"""Run every colour order (24 batches) with baskets, one after another, over COM11.

Each batch: P0 -> three routes S1..S8 (executor.run_batch), then the script pauses so
the operator resets the three baskets to their source positions, then it returns the
arm from the last S8 to P0 and starts the next batch.

Return to P0 reuses the staging of 回P0_4帧.d6a (made 2026-10-12), but starts from the
live S8 instead of that file's fixed first frame:
  1. ID4 to its P0 value;
  2. ID1 and ID3 to their P0 values (base turns back);
  3. remaining joints (ID2, ID5, ID6) to P0.
Not collision checked: the first return is watched and must be confirmed.

Results go to logs/combos_*/summary.json (per batch: order, steps, total_s, error,
operator answer). Re-running with --resume <summary.json> skips confirmed batches.

  python run_all_combos.py --dry-run                         # list the 24 orders + return plans
  python run_all_combos.py --port COM11 --go --loaded        # full run with baskets
  python run_all_combos.py --port COM11 --go --loaded --only 红-蓝-黄 绿-黄-红
"""
import argparse
from datetime import datetime, timezone, timedelta
from itertools import permutations
import json
import sys
import threading
import time
from pathlib import Path

import collection as c
import sequence as seq
import executor as ex

r, sr = ex.r, ex.sr


def return_stages(start, p0):
    """S8 -> P0 in the three stages of 回P0_4帧.d6a. Each stage only moves the listed joints."""
    stages = []
    cur = start[:]
    for name, ids in (('回P0①ID4', (4,)), ('回P0②ID1+ID3', (1, 3)), ('回P0③收尾', (2, 5, 6))):
        target = [p0[i] if i + 1 in ids else cur[i] for i in range(6)]
        stages.append(dict(name=name, route=None, point=0, kind='move', axes=ids, carry=False, target=target))
        cur = target
    return stages


def return_to_p0(a, start_pose, p0, emit_name):
    """Drive the three return stages with the smooth_route motion layer; returns final feedback."""
    lease, lease_sha, expires = r.require_device_lease(a.port, holder=a.holder)
    limits = r.load_limits()
    run = a.logs / (emit_name + '_' + str(time.time_ns())[-6:])
    run.mkdir(parents=True)

    def cancelled():
        try:
            return ((run / 'STOP').exists() or datetime.now(timezone.utc) >= expires
                    or r.sha(lease) != lease_sha)
        except OSError:
            return True
    result = dict(start=start_pose, target=p0, steps=[], complete=False)
    with (run / 'events.jsonl').open('w', encoding='utf-8') as log:
        mutex = threading.Lock()

        def emit(event, **data):
            with mutex:
                log.write(json.dumps(dict(event=event, t=time.monotonic(), **data), ensure_ascii=False) + '\n')
                log.flush()
        transport = r.ReplayTransport(emit, cancelled, limits)
        arm = r.ArmController(transport=transport, log=lambda msg: emit('library', message=msg))
        arm.gate.min_period_s = sr.PACING_S
        try:
            arm.connect_serial(a.port)
            start, _ = r.capture_stable(arm, emit)
            if max(abs(x - y) for x, y in zip(start, start_pose)) > ex.START_TOL:
                raise r.SafetyError(f'回位前姿态{start}与预期S8{start_pose}不符；不自动追赶')
            arm.enable_motion(True)
            ex.drive(arm, transport, return_stages(start, p0), start, 'gentle', emit, cancelled, limits,
                     result['steps'].append)
            result['complete'] = True
            result['final'], _ = r.capture_stable(arm, emit)
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            transport.pending.clear()
            arm.enable_motion(False)
            arm.close()
            (run / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f"RETURN={'完成' if result['complete'] else '失败：' + result.get('error', '')}  RUN_DIR={run}", flush=True)
    return result


def ask(prompt, choices):
    while True:
        ans = input(prompt).strip().lower()
        if ans in choices:
            return ans
        print('请输入：' + ' / '.join(choices))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--root', type=Path, default=c.ROOT)
    ap.add_argument('--port', choices=('COM11',))
    ap.add_argument('--holder', default='claude', choices=('claude', 'codex'))
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--loaded', action='store_true', help='带篮运行（gentle）')
    ap.add_argument('--empty-load', action='store_true')
    ap.add_argument('--only', nargs='+', help='只跑指定顺序，例如 红-蓝-黄')
    ap.add_argument('--resume', type=Path, help='上次的summary.json，跳过已确认通过的组')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)
    try:
        home, routes, _ = seq.load_routes(a.root)
        p0 = home['points'][0]['encoder_positions'][:]
        orders = [list(p) for p in permutations(c.COLORS, 3)]
        if a.only:
            orders = [seq.parse_command(t) for t in a.only]
        done_before = set()
        if a.resume:
            prev = json.loads(a.resume.read_text(encoding='utf-8'))
            done_before = {b['command'] for b in prev['batches'] if b.get('operator_confirmed')}
        if a.dry_run:
            for cols in orders:
                end = seq.goal_of(routes[(cols[2], 'P3')]['points'][7])
                stages = ' → '.join(f"{s['name']}{s['target']}" for s in return_stages(end, p0))
                print(f"{'-'.join(cols)}：25帧，停在{cols[2]}→P3 S8 {end}\n    回位：{stages}")
            print(f'共{len(orders)}组；每组约42s运动+复位篮子时间')
            return 0
        if not (a.port and a.go and (a.loaded or a.empty_load)):
            ap.error('运行需要 --port COM11 --go 以及 --loaded 或 --empty-load')
        a.profile = sr.LOADED_PROFILE if a.loaded else 'gentle'
        a.from_s8 = False
        now = datetime.now(timezone(timedelta(hours=8)))
        a.logs = a.root / 'logs' / ('combos_' + now.strftime('%Y%m%d_%H%M%S'))
        a.logs.mkdir(parents=True)
        summary = dict(started_at=now.isoformat(), loaded=a.loaded, batches=[])
        summary_file = a.logs / 'summary.json'

        def save():
            summary_file.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'SUMMARY={summary_file}')
        input('确认：机械臂在P0，四个篮子在源位，P1/P2/P3已清空。回车开始（Ctrl+C中止）')
        first_return = True
        for n, cols in enumerate(orders, 1):
            name = '-'.join(cols)
            if name in done_before:
                print(f'[{n}/{len(orders)}] {name} 上次已通过，跳过')
                continue
            print(f'\n[{n}/{len(orders)}] {name}：红黄蓝绿中的 {cols[0]}→P1 {cols[1]}→P2 {cols[2]}→P3', flush=True)
            res = ex.run_batch(a, cols)
            entry = dict(command=name, total_s=res.get('total_s'), error=res.get('error'),
                         operator_confirmed=res.get('operator_confirmed', False),
                         operator_answer=res.get('operator_answer'), steps=len(res.get('steps', [])))
            summary['batches'].append(entry)
            save()
            if res.get('error'):
                print('本组中止，脚本停止。先核对现场和姿态；修好后用 --resume 继续')
                return 2
            if not entry['operator_confirmed']:
                if ask('本组未确认通过。c=记为失败继续下一组  q=停止: ', ('c', 'q')) == 'q':
                    return 2
            if n == len(orders):
                break
            ask('复位：把三个篮子放回源位、清空P1/P2/P3，人离开工作区。完成输入 y: ', ('y',))
            end = res['steps'][-1]['feedback']      # where the arm really stopped (may be short of S8)
            if first_return:
                print('第一次回P0（S8→P0未验证过），请盯着机械臂，异常立即断电')
            ret = return_to_p0(a, end, p0, f'return_after_{name}')
            entry['return'] = dict(complete=ret['complete'], error=ret.get('error'))
            save()
            if not ret['complete']:
                print('回P0失败，脚本停止；用上位机回P0后用 --resume 继续')
                return 2
            if first_return:
                if ask('回P0过程无碰撞、姿态正确？y=继续  q=停止: ', ('y', 'q')) == 'q':
                    return 2
                first_return = False
        passed = sum(b['operator_confirmed'] for b in summary['batches'])
        summary.update(finished_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                       passed=passed, total=len(summary['batches']))
        save()
        print(f'\n完成：{passed}/{len(summary["batches"])}组通过。SUMMARY={summary_file}')
        return 0 if passed == len(summary['batches']) else 2
    except KeyboardInterrupt:
        print('\n已中止（Ctrl+C）。已发出的那一段运动会走到目标；串口已关闭。结果见SUMMARY')
        return 130
    except (ValueError, OSError, r.SafetyError) as exc:
        print('ERROR=' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
