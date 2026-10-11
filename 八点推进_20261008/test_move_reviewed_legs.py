import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock

import move_reviewed_legs as m
import read_pose
import reposition_empty


def plan_for(legs):
    return dict(schema=m.SCHEMA, source_sha256='r', limits_sha256='l', historical_hold_sha256='h',
                empty_load_only=True, legs=legs)


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.limits = m.r.load_limits()
        self.a = [2407, 1607, 2430, 3087, 2454, 2016]
        self.b = [2407, 2093, 2421, 3087, 2453, 2016]
        self.c = [1607, 2093, 2421, 3087, 2453, 2016]

    def test_valid_chain_accepted(self):
        legs = m.validate_plan(plan_for([dict(name='lift', expected_start=self.a, target=self.b),
                                         dict(name='turn', expected_start=self.b, target=self.c)]),
                               'r', 'l', 'h', self.limits)
        self.assertEqual(m.find_leg(legs, 'turn')[0], 1)
        with self.assertRaises(m.r.SafetyError):
            m.find_leg(legs, 'missing')

    def test_mismatched_bindings_rejected(self):
        plan = plan_for([dict(name='lift', expected_start=self.a, target=self.b)])
        for args in (('x', 'l', 'h'), ('r', 'x', 'h'), ('r', 'l', 'x')):
            with self.assertRaises(m.r.SafetyError):
                m.validate_plan(plan, *args, self.limits)
        plan['empty_load_only'] = False
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(plan, 'r', 'l', 'h', self.limits)

    def test_discontinuous_or_loose_legs_rejected(self):
        broken = plan_for([dict(name='lift', expected_start=self.a, target=self.b),
                           dict(name='turn', expected_start=self.a, target=self.c)])
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(broken, 'r', 'l', 'h', self.limits)
        loose = plan_for([dict(name='lift', expected_start=self.a, target=self.b, start_tolerance=40)])
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(loose, 'r', 'l', 'h', self.limits)
        odd = plan_for([dict(name='lift', expected_start=self.a, target=self.b, settle_tolerances=[5] * 6)])
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(odd, 'r', 'l', 'h', self.limits)
        out = self.b[:]
        out[1] = 3000
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(plan_for([dict(name='lift', expected_start=self.a, target=out)]),
                            'r', 'l', 'h', self.limits)

    def test_chunks_bounded_and_end_on_target(self):
        goals = m.chunk_goals(self.b, self.c)
        self.assertEqual(goals[-1], self.c)
        previous = self.b
        for g in goals:
            self.assertLessEqual(max(abs(x - y) for x, y in zip(previous, g)), m.MAX_CHUNK)
            previous = g

    def test_leg_refuses_start_gap_without_motion(self):
        arm = Mock()
        live = self.a[:]
        live[0] += 13
        leg = dict(name='lift', expected_start=self.a, target=self.b)
        with patch.object(m.r, 'capture_stable', return_value=(live, [])), \
                patch.object(m.setup, 'execute_direct_p1') as direct:
            with self.assertRaises(m.r.SafetyError):
                m.execute_leg(arm, leg, Mock(), lambda: False, 'r', self.limits, 'l')
            direct.assert_not_called()

    def test_leg_runs_chunks_from_live_feedback(self):
        seen = []

        def fake(arm, start, goal, *args):
            seen.append((start, goal))
            return dict(returned_report=goal[:])
        leg = dict(name='turn', expected_start=self.b, target=self.c)
        with patch.object(m.r, 'capture_stable', return_value=(self.b, [])), \
                patch.object(m.setup, 'execute_direct_p1', side_effect=fake):
            out = m.execute_leg(Mock(), leg, Mock(), lambda: False, 'r', self.limits, 'l')
        self.assertEqual(seen[0][0], self.b)
        self.assertEqual(out['returned_report'], self.c)
        self.assertFalse(out['operator_confirmed'])


class ObservationAfterCloseTests(unittest.TestCase):
    def test_prompt_happens_after_serial_close(self):
        order = []
        b = [2407, 2093, 2421, 3087, 2453, 2016]
        c = [1607, 2093, 2421, 3087, 2453, 2016]
        with tempfile.TemporaryDirectory() as tmp:
            plan_path = Path(tmp) / 'plan.json'
            plan_path.write_text(json.dumps(plan_for([dict(name='turn', expected_start=b, target=c)])),
                                 encoding='utf-8')
            arm = Mock()
            arm.close.side_effect = lambda: order.append('close')
            arm.get_battery_mv.return_value = 12300
            fake_exp = m.datetime.now(m.timezone.utc) + m.timedelta(minutes=5)

            def shas(path):
                return 'h' if Path(path) == m.r.DIAGNOSTIC_HOLD else ('l' if Path(path) == m.r.DEFAULT_LIMITS else 'p')
            argv = ['x', '--port', 'COM11', '--plan', str(plan_path), '--leg', 'turn', '--go', '--empty-load']
            with patch.object(sys, 'argv', argv), patch.object(m.r, 'HERE', Path(tmp)), \
                    patch.object(m.r.hashlib, 'sha256', return_value=Mock(hexdigest=lambda: 'r')), \
                    patch.object(m.r, 'sha', side_effect=shas), \
                    patch.object(m, 'validate_plan', return_value=json.loads(plan_path.read_text())['legs']), \
                    patch.object(m.r, 'require_device_lease', return_value=(plan_path, 'p', fake_exp)), \
                    patch.object(m, 'ArmController', return_value=arm), \
                    patch.object(m, 'configuration', return_value={}), \
                    patch.object(m, 'same_configuration', return_value=True), \
                    patch.object(m, 'execute_leg', return_value=dict(returned_report=c)), \
                    patch.object(m.setup, 'ask_observation', side_effect=lambda: order.append('ask') or 'yes'), \
                    patch.object(m.r.SOURCE.__class__, 'read_bytes', return_value=b'{"configuration":{}}'):
                code = m.main()
            self.assertEqual(code, 0)
            self.assertEqual(order, ['close', 'ask'])
            run = next(Path(tmp, 'logs').iterdir())
            result = json.loads((run / 'result.json').read_text(encoding='utf-8'))
            self.assertTrue(result['serial_closed'] and result['complete'])

    def test_eof_is_not_confirmation(self):
        with patch('builtins.input', side_effect=EOFError):
            self.assertEqual(reposition_empty.ask_observation(), 'no observation')


class BoxStartTests(unittest.TestCase):
    def setUp(self):
        self.limits = m.r.load_limits()
        self.box = [[2380, 2410], [1800, 2000], [1980, 2220], [3020, 3090], [2440, 2465], [2000, 2030]]
        self.lift = [2403, 2093, 2421, 3087, 2453, 2016]

    def test_box_only_on_first_leg(self):
        ok = plan_for([dict(name='1-lift', start_box=self.box, target=self.lift),
                       dict(name='2-open', expected_start=self.lift, target=self.lift[:5] + [1070])])
        m.validate_plan(ok, 'r', 'l', 'h', self.limits)
        bad = plan_for([dict(name='a', expected_start=self.lift, target=self.lift),
                        dict(name='b', start_box=self.box, target=self.lift)])
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(bad, 'r', 'l', 'h', self.limits)
        wide = [b[:] for b in self.box]
        wide[1] = [1500, 2000]
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(plan_for([dict(name='a', start_box=wide, target=self.lift)]), 'r', 'l', 'h', self.limits)

    def test_live_outside_box_refused_without_motion(self):
        leg = dict(name='1-lift', start_box=self.box, target=self.lift)
        live = [2394, 1790, 2194, 3064, 2450, 2016]
        with patch.object(m.r, 'capture_stable', return_value=(live, [])), \
                patch.object(m.setup, 'execute_direct_p1') as direct:
            with self.assertRaises(m.r.SafetyError):
                m.execute_leg(Mock(), leg, Mock(), lambda: False, 'r', self.limits, 'l')
            direct.assert_not_called()

    def test_live_inside_box_chunks_from_live(self):
        leg = dict(name='1-lift', start_box=self.box, target=self.lift)
        live = [2394, 1898, 2194, 3064, 2450, 2016]
        seen = []

        def fake(arm, start, goal, *args):
            seen.append(start)
            return dict(returned_report=goal[:])
        with patch.object(m.r, 'capture_stable', return_value=(live, [])), \
                patch.object(m.setup, 'execute_direct_p1', side_effect=fake):
            out = m.execute_leg(Mock(), leg, Mock(), lambda: False, 'r', self.limits, 'l')
        self.assertEqual(seen[0], live)
        self.assertEqual(out['returned_report'], self.lift)


class TorqueTests(unittest.TestCase):
    def test_hold_frames_target_current_counts_slowly(self):
        import restore_torque as t
        counts = [2394, 1898, 2194, 3064, 2450, 2016]
        frames = t.hold_frames(counts)
        self.assertEqual([sid for sid, _ in frames], [1, 2, 3, 4, 5, 6])
        for (sid, frame), v in zip(frames, counts):
            self.assertEqual(frame, m.r.build_frame(3, m.r.pack_joint_counts(v, t.HOLD_ACC, t.HOLD_SPEED), sid))

    def test_only_one_torque_on_allowed(self):
        import restore_torque as t
        tr = t.TorqueTransport(lambda *a, **k: None, lambda: False)
        with patch.object(m.r.Transport, 'send') as raw:
            with self.assertRaises(m.r.SafetyError):
                tr.send(61, b'\x01')            # not yet allowed
            tr.torque_allowed = True
            with self.assertRaises(m.r.SafetyError):
                tr.send(61, b'\x00')            # torque off never allowed
            tr.send(61, b'\x01')
            with self.assertRaises(m.r.SafetyError):
                tr.send(61, b'\x01')            # second time refused
            self.assertEqual(raw.call_count, 1)
            self.assertEqual(tr.torque_sent, 1)

    def test_restore_aborts_before_torque_if_hold_goals_move_arm(self):
        import restore_torque as t
        before = [2394, 1898, 2194, 3064, 2450, 2016]
        moved = before[:]
        moved[1] += 30
        tr = t.TorqueTransport(lambda *a, **k: None, lambda: False)
        arm = Mock()
        arm.move_joint_counts.side_effect = lambda *a: tr.pending.popleft()
        with patch.object(m.r, 'capture_stable', side_effect=[(before, []), (moved, [])]), \
                patch.object(t.time, 'sleep'):
            with self.assertRaises(m.r.SafetyError):
                t.restore(arm, tr, lambda *a, **k: None, m.r.load_limits())
        arm.set_torque.assert_not_called()

    def test_restore_reports_no_change(self):
        import restore_torque as t
        before = [2394, 1898, 2194, 3064, 2450, 2016]
        tr = t.TorqueTransport(lambda *a, **k: None, lambda: False)
        arm = Mock()
        arm.move_joint_counts.side_effect = lambda *a: tr.pending.popleft()
        arm.set_torque.side_effect = lambda on: setattr(tr, 'torque_sent', 1)
        with patch.object(m.r, 'capture_stable', return_value=(before, [])), patch.object(t.time, 'sleep'):
            out = t.restore(arm, tr, lambda *a, **k: None, m.r.load_limits())
        self.assertEqual(out['max_change'], 0)
        self.assertTrue(out['torque_command_sent'])
        arm.set_torque.assert_called_once_with(True)


class GateTests(unittest.TestCase):
    def _run(self, tmp, **extra):
        run = Path(tmp, 'logs', 'legs_x')
        run.mkdir(parents=True)
        res = dict(plan_sha256='p', leg='1-lift', serial_closed=True, segment={}, **extra)
        (run / 'result.json').write_text(json.dumps(res), encoding='utf-8')
        (run / 'events.jsonl').write_text('', encoding='utf-8')
        return run

    def test_next_leg_requires_recorded_yes(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self._run(tmp, operator_answer='no observation', operator_confirmed=False)
            logs = Path(tmp, 'logs')
            self.assertFalse(m.leg_confirmed(logs, 'p', '1-lift'))
            m.record_observation(run, 'yes')
            self.assertTrue(m.leg_confirmed(logs, 'p', '1-lift'))
            self.assertFalse(m.leg_confirmed(logs, 'other-plan', '1-lift'))
            with self.assertRaises(m.r.SafetyError):
                m.record_observation(run, 'no')

    def test_errored_run_cannot_be_confirmed(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self._run(tmp, error='timeout')
            with self.assertRaises(m.r.SafetyError):
                m.record_observation(run, 'yes')
            self.assertFalse(m.leg_confirmed(Path(tmp, 'logs'), 'p', '1-lift'))

    def test_non_yes_is_not_confirmation(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self._run(tmp)
            res = m.record_observation(run, 'ok')
            self.assertFalse(res['operator_confirmed'])


class ReadPoseTests(unittest.TestCase):
    def test_nearest_point_first(self):
        targets = m.r.resolve_route(json.loads(m.r.SOURCE.read_text(encoding='utf-8')))
        live = targets[5][:]
        live[1] += 7
        rows = read_pose.distances(live, targets)
        self.assertEqual(rows[0]['point'], 6)
        self.assertEqual(rows[0]['max_delta'], 7)



class WatchTests(unittest.TestCase):
    def test_frozen_joint_has_one_distinct_value(self):
        import watch_pose
        rows = [[2394, 1898, 2067 + k, 3052, 2259, 2021] for k in range(5)]
        s = watch_pose.summarize(rows)
        self.assertEqual(s[1]['distinct'], 1)
        self.assertEqual(s[1]['span'], 0)
        self.assertEqual(s[2]['span'], 4)

    def test_watch_uses_query_only_transport(self):
        import watch_pose
        tr = watch_pose.ReadOnlyTransport(lambda *a, **k: None)
        with self.assertRaises(m.r.SafetyError):
            tr.send(3, m.r.pack_joint_counts(1898, 20, 300), 2)
        with self.assertRaises(m.r.SafetyError):
            tr.send(61, b'\x01')


if __name__ == '__main__':
    unittest.main()
