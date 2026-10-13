import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import collection as c
import sequence as seq
import executor as ex


class ParseTests(unittest.TestCase):
    def test_formats(self):
        for text in ('红-蓝-黄', '红 蓝 黄', '红蓝黄', '红色，蓝色，黄色', 'red,blue,yellow', '红→蓝→黄'):
            with self.subTest(text=text):
                self.assertEqual(seq.parse_command(text), ['红', '蓝', '黄'])

    def test_rejects(self):
        for text in ('', '红-蓝', '红-蓝-黄-绿', '红-红-黄', '红-紫-黄'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                seq.parse_command(text)


class CompileTests(unittest.TestCase):
    """Uses the real collected routes (97/97) read-only."""

    @classmethod
    def setUpClass(cls):
        cls.home, cls.routes, cls.pins = seq.load_routes()

    def test_all_24_orders_compile(self):
        from itertools import permutations
        for colors in permutations(c.COLORS, 3):
            steps = seq.compile_plan(list(colors), self.home, self.routes)
            self.assertEqual(len(steps), 25)
            self.assertEqual(steps[0]['name'], 'P0')
            self.assertEqual([s['route'] for s in steps[1::8]], list(zip(colors, c.TARGETS)))
            self.assertEqual(steps[-1]['point'], 8)

    def test_gripper_semantics(self):
        steps = seq.compile_plan(['红', '蓝', '黄'], self.home, self.routes)
        for s in steps[1:]:
            row = self.routes[s['route']]['points'][s['point'] - 1]
            self.assertEqual(s['target'][5], row['gripper_target_counts'])
            if s['point'] == 3:
                self.assertEqual((s['kind'], s['axes']), ('close', (6,)))
                self.assertEqual(s['target'][5], min(row['encoder_positions'][5] + 200, 2731))
            if 4 <= s['point'] <= 6:
                self.assertEqual(s['axes'], (1, 2, 3, 4, 5))
                self.assertTrue(s['carry'])
        # 红→P2 S7 carries the recorded body-motion waiver: moves every joint
        s7 = seq.compile_plan(['蓝', '红', '黄'], self.home, self.routes)[1 + 8 + 6]
        self.assertEqual((s7['name'], s7['kind'], s7['axes']), ('红→P2 S7', 'open', None))

    def test_d6a_output(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / 'x.d6a'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(seq.main(['绿-黄-红', '--d6a', str(out)]), 0)
            frames = json.loads(out.read_text(encoding='utf-8'))
            self.assertEqual(len(frames), 25)
            self.assertTrue(all(len(f['servos']) == 6 and f['time'] == 1000 for f in frames))

    def test_motion_plan_keeps_squeezed_grip_while_carrying(self):
        steps = seq.compile_plan(['红', '蓝', '黄'], self.home, self.routes)
        planned = ex.plan_motion(steps, steps[0]['target'], 'gentle')
        for p in planned:
            if p['carry']:
                self.assertEqual(p['start'][5], 2409)
                self.assertNotIn(6, [j['id'] for j in p['profile']['joints']])

    def test_start_guard(self):
        steps = seq.compile_plan(['红', '蓝', '黄'], self.home, self.routes)
        p0 = steps[0]['target']
        self.assertIs(ex.start_steps(steps, p0, self.routes, False), steps)
        s8 = seq.goal_of(self.routes[('绿', 'P3')]['points'][7])
        with self.assertRaisesRegex(Exception, 'from-s8'):
            ex.start_steps(steps, s8, self.routes, False)
        got = ex.start_steps(steps, s8, self.routes, True)
        self.assertEqual((got[0]['name'], got[0]['target']), ('绿→P3 S8→P0', p0))
        far = [v + 200 for v in p0[:5]] + [p0[5]]
        with self.assertRaisesRegex(Exception, '不自动追赶'):
            ex.start_steps(steps, far, self.routes, True)


class FakeArm:
    """Joints jump to their last commanded goal; ID6 stalls short of a squeezed close goal."""

    def __init__(self, start):
        self.pos = start[:]
        self.writes = []
        self.gate = type('G', (), {'min_period_s': 0})()
        self.closed = False

    def connect_serial(self, port):
        pass

    def enable_motion(self, on):
        pass

    def close(self):
        self.closed = True

    def get_battery_mv(self):
        return 12300

    def move_joint_counts(self, sid, value, acc, speed):
        self.writes.append((sid, value))
        self.transport.pending.popleft()
        self.pos[sid - 1] = min(value, 2209) if sid == 6 and value > 2209 else value

    def get_real_joints(self):
        return [(v, 0) for v in self.pos]


class RunBatchTests(unittest.TestCase):
    def test_run_batch_with_fake_arm(self):
        from unittest.mock import patch
        home, routes, _ = seq.load_routes()
        p0 = home['points'][0]['encoder_positions']
        arm = FakeArm(p0)

        def make_arm(transport, log):
            arm.transport = transport
            return arm
        with tempfile.TemporaryDirectory() as d:
            a = type('A', (), dict(root=c.ROOT, logs=Path(d), port='COM11', holder='claude',
                                   profile='gentle', loaded=True, from_s8=False))()
            lease = Path(d) / 'lease'
            lease.write_text('x')
            from datetime import datetime, timezone, timedelta
            exp = datetime.now(timezone.utc) + timedelta(minutes=5)
            with patch.object(ex.r, 'require_device_lease', return_value=(lease, c.sha(lease), exp)), \
                 patch.object(ex.r, 'ArmController', side_effect=make_arm), \
                 patch.object(ex.r, 'configuration', return_value=home['configuration']), \
                 patch.object(ex.r, 'capture_stable', return_value=(p0[:], [])), \
                 patch('time.sleep'), patch('builtins.input', return_value='yes'), \
                 contextlib.redirect_stdout(io.StringIO()):
                res = ex.run_batch(a, ['红', '蓝', '黄'])
        self.assertTrue(res['controller_complete'], res.get('error'))
        self.assertTrue(res['operator_confirmed'])
        self.assertTrue(arm.closed)
        self.assertEqual(len(res['steps']), 24)   # P0 skipped: already there
        # ID6 is written only for open/close/plain moves, never while carrying
        grip_writes = [v for sid, v in arm.writes if sid == 6]
        self.assertEqual(grip_writes.count(2409), 3)
        self.assertEqual(arm.pos[:5], seq.goal_of(routes[('黄', 'P3')]['points'][7])[:5])


class RunAllCombosTests(unittest.TestCase):
    def test_return_stages_end_at_p0_and_move_named_joints_only(self):
        import run_all_combos as rc
        home, routes, _ = seq.load_routes()
        p0 = home['points'][0]['encoder_positions']
        s8 = seq.goal_of(routes[('绿', 'P3')]['points'][7])
        stages = rc.return_stages(s8, p0)
        self.assertEqual(stages[-1]['target'], p0)
        prev = s8
        for st in stages:
            moved = {i + 1 for i in range(6) if st['target'][i] != prev[i]}
            self.assertLessEqual(moved, set(st['axes']))
            prev = st['target']

    def test_two_batches_with_return_on_fake_arm(self):
        from unittest.mock import patch
        from datetime import datetime, timezone, timedelta
        import run_all_combos as rc
        home, _, _ = seq.load_routes()
        p0 = home['points'][0]['encoder_positions']
        arm = FakeArm(p0)

        def make_arm(transport, log):
            arm.transport = transport
            return arm

        def stable(*_):
            return arm.pos[:], []
        import shutil
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / 'data'
            root.mkdir()
            shutil.copy(c.ROOT / 'scene.json', root)
            for path, _, _ in c.paths(c.ROOT):
                dst = root / path.relative_to(c.ROOT)
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(path, dst)
            lease = Path(d) / 'lease'
            lease.write_text('x')
            exp = datetime.now(timezone.utc) + timedelta(minutes=5)
            answers = iter(['', 'yes', 'y', 'y', 'yes'])
            with patch.object(ex.r, 'require_device_lease', return_value=(lease, c.sha(lease), exp)), \
                 patch.object(ex.r, 'ArmController', side_effect=make_arm), \
                 patch.object(ex.r, 'configuration', return_value=home['configuration']), \
                 patch.object(ex.r, 'capture_stable', side_effect=stable), \
                 patch('time.sleep'), patch('builtins.input', side_effect=lambda *_: next(answers)), \
                 contextlib.redirect_stdout(io.StringIO()):
                code = rc.main(['--root', str(root), '--port', 'COM11', '--go', '--loaded',
                                '--only', '红-蓝-黄', '绿-黄-红'])
            summaries = sorted((root / 'logs').glob('combos_*/summary.json'))
            s = json.loads(summaries[-1].read_text(encoding='utf-8'))
        self.assertEqual(code, 0)
        self.assertEqual([b['command'] for b in s['batches']], ['红-蓝-黄', '绿-黄-红'])
        self.assertTrue(s['batches'][0]['return']['complete'])
        self.assertEqual(s['passed'], 2)


if __name__ == '__main__':
    unittest.main()
