import json
import unittest
from unittest.mock import patch

import smooth_route as m

r = m.r


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class FakeArm:
    """Simulates servos following the commanded synchronized trapezoid."""

    def __init__(self, transport, start, clock, behaviour=None):
        self.transport, self.pos, self.clock = transport, start[:], clock
        self.moves = {}            # id -> (t0, p0, target, v, a)
        self.behaviour = behaviour or {}
        self.writes = []

    def move_joint_counts(self, sid, pos, acc, speed):
        self.transport.send(3, r.pack_joint_counts(pos, acc, speed), sid)
        self.writes.append((sid, pos, acc, speed))
        self.moves[sid] = (self.clock(), self._at(sid), pos, speed, acc * m.ACC_UNIT)

    def _at(self, sid):
        if sid not in self.moves:
            return self.pos[sid - 1]
        t0, p0, goal, v, a = self.moves[sid]
        d = goal - p0
        dist = abs(d)
        t = self.clock() - t0
        ta = v / a
        if dist < v * ta:          # triangular
            ta = (dist / a) ** 0.5
            v = a * ta
        T = dist / v + ta if dist else 0
        if t >= T:
            s = dist
        elif t < ta:
            s = 0.5 * a * t * t
        elif t < T - ta:
            s = 0.5 * a * ta * ta + v * (t - ta)
        else:
            s = dist - 0.5 * a * (T - t) ** 2
        return round(p0 + (s if d > 0 else -s))

    def get_real_joints(self):
        out = []
        for sid in range(1, 7):
            v = self._at(sid)
            f = self.behaviour.get(sid)
            out.append((f(v) if f else v, 0))
        return out


def transport():
    tr = r.ReplayTransport(lambda *a, **k: None, lambda: False)
    return tr


class ProfileTests(unittest.TestCase):
    def test_all_joints_share_duration(self):
        start = [1615, 1542, 2612, 1981, 2673, 1999]
        target = [2425, 1680, 2431, 3083, 2485, 2020]
        p = m.segment_profile(start, target, 1000, 2500)
        for j in p['joints']:
            covered = j['speed'] * (p['T'] - p['ta'])
            self.assertLess(abs(covered - abs(j['delta'])), max(3, 0.03 * abs(j['delta'])), j)
            self.assertGreaterEqual(j['acc'], 1)
        lead = max(p['joints'], key=lambda j: abs(j['delta']))
        self.assertEqual(lead['speed'], 1000)

    def test_triangular_and_minimum_duration(self):
        p = m.segment_profile([0, 0, 0, 0, 0, 1000], [0, 0, 0, 0, 0, 1100], 1000, 2500)
        self.assertAlmostEqual(p['T'], 2 * (100 / 2500) ** 0.5, places=6)
        q = m.segment_profile([0, 0, 0, 0, 0, 1000], [0, 0, 0, 0, 0, 1100], 1000, 2500, min_duration=1.0)
        self.assertAlmostEqual(q['T'], 1.0)
        self.assertLess(q['joints'][0]['speed'], p['joints'][0]['speed'])

    def test_small_and_excluded_axes_not_sent(self):
        p = m.segment_profile([10, 10, 10, 10, 10, 10], [12, 10, 400, 10, 10, 900], 1000, 2500, axes=(6,))
        self.assertEqual([j['id'] for j in p['joints']], [6])
        self.assertEqual(m.segment_profile([5] * 6, [7] * 6, 1000, 2500)['joints'], [])

    def test_never_zero_acceleration(self):
        # acc register 0 means "unlimited" on these servos; it must never be sent.
        p = m.segment_profile([2000] * 6, [2005, 2010, 2020, 2040, 2080, 3000], 1400, 4000)
        self.assertTrue(all(1 <= j['acc'] <= 254 and 1 <= j['speed'] <= m.SPEED_REG_MAX for j in p['joints']))


class StepTests(unittest.TestCase):
    def setUp(self):
        self.targets = r.resolve_route(json.loads(r.SOURCE.read_text(encoding='utf-8')))

    def test_p1_sequence_and_grip_holds_body(self):
        steps = m.build_steps(self.targets, 'p1')
        self.assertEqual([s['name'] for s in steps],
                         ['p2', 'p3-grip-close', 'p4', 'p5', 'p6', 'p7-grip-open', 'p8'])
        grip = steps[1]
        self.assertEqual(grip['axes'], (6,))
        self.assertEqual(grip['target'][:5], self.targets[1][:5])
        self.assertEqual(steps[-1]['target'], self.targets[7])
        self.assertEqual(steps[5]['target'][:5], self.targets[5][:5])

    def test_basket_squeeze_holds_id6_through_carry(self):
        plain = m.plan_steps(m.build_steps(self.targets, 'p8'), self.targets[7], 'gentle')
        planned = {s['name']: s for s in m.plan_steps(
            m.build_steps(self.targets, 'p8', m.GRIP_SQUEEZE), self.targets[7], 'gentle')}
        close = planned['p3-grip-close']
        self.assertEqual(close['target'][5], self.targets[2][5] + m.GRIP_SQUEEZE)
        for name in ('p4', 'p5', 'p6'):
            s = planned[name]
            self.assertNotIn(6, [j['id'] for j in s['profile']['joints']])   # goal stays squeezed
            self.assertEqual(s['start'][5], close['target'][5])
            # contact short of the squeezed goal (jaws reached ~1997 freely) stays inside the box
            self.assertGreaterEqual(min(s['start'][5], s['target'][5]) - m.BOX_MARGIN, 1900)
        self.assertEqual(planned['p7-grip-open']['start'][5], close['target'][5])
        self.assertEqual(planned['p7-grip-open']['target'][5], self.targets[6][5])
        # empty runs unchanged
        self.assertEqual([s['target'] for s in plain],
                         [s['target'] for s in m.plan_steps(m.build_steps(self.targets, 'p8'), self.targets[7], 'gentle')])
        self.assertIsNone(next(s for s in plain if s['name'] == 'p4')['axes'])

    def test_home_start_turns_base_then_descends(self):
        home = m.load_home(r.load_limits())
        steps = m.steps_for_start(self.targets, 'home', home, 0, home)
        self.assertEqual([s['name'] for s in steps[:3]], ['home-base-to-p1-azimuth', 'to-p1', 'p2'])
        self.assertEqual(steps[0]['target'][1:], home[1:])
        self.assertEqual(steps[0]['target'][0], self.targets[0][0])
        self.assertEqual(steps[1]['target'], self.targets[0])
        self.assertEqual(steps[-1]['target'], self.targets[7])

    def test_home_mode_from_p8_returns_home_first(self):
        home = m.load_home(r.load_limits())
        near_p8 = [v + 10 for v in self.targets[7]]
        steps = m.steps_for_start(self.targets, 'home', near_p8, m.GRIP_SQUEEZE, home)
        self.assertEqual(steps[0]['name'], 'p8-to-home')
        self.assertEqual(steps[0]['target'], home)
        self.assertEqual(steps[1]['name'], 'home-base-to-p1-azimuth')
        planned = m.plan_steps(steps, near_p8, 'gentle')
        limits = r.load_limits()
        for s in planned:
            r.check_counts(s['target'], limits)

    def test_start_elsewhere_is_refused(self):
        home = m.load_home(r.load_limits())
        for mode, start in (('home', self.targets[4]), ('p8', home), ('p1', self.targets[7])):
            with self.assertRaises(r.SafetyError):
                m.steps_for_start(self.targets, mode, start, 0, home)

    def test_p8_mode_adds_reviewed_transit(self):
        steps = m.build_steps(self.targets, 'p8')
        transit = steps[0]['target']
        self.assertEqual(transit[1:], self.targets[7][1:])
        self.assertEqual(transit[0], self.targets[0][0])
        self.assertEqual(steps[1]['target'], self.targets[0])

    def test_planned_chain_and_targets_inside_limits(self):
        planned = m.plan_steps(m.build_steps(self.targets, 'p8'), self.targets[7], 'normal')
        limits = r.load_limits()
        for prev, nxt in zip(planned, planned[1:]):
            r.check_counts(prev['target'], limits)
            if prev['axes'] is None:
                self.assertEqual(nxt['start'], prev['target'])
        total = sum(s['profile']['T'] for s in planned)
        self.assertLess(total, 15.0)     # "not too slow": whole loop well under the old ~40 s

    def test_careful_segments_are_slower(self):
        planned = {s['name']: s for s in m.plan_steps(m.build_steps(self.targets, 'p1'), self.targets[0], 'normal')}
        self.assertGreaterEqual(planned['p2']['profile']['T'], 1.0)
        self.assertGreaterEqual(planned['p6']['profile']['T'], 1.0)


class RunStepTests(unittest.TestCase):
    def setUp(self):
        self.targets = r.resolve_route(json.loads(r.SOURCE.read_text(encoding='utf-8')))
        self.limits = r.load_limits()
        self.raw = patch.object(r.Transport, 'send').start()
        self.addCleanup(patch.stopall)

    def _run(self, step, start, behaviour=None):
        clock = FakeClock()
        tr = transport()
        arm = FakeArm(tr, start, clock, behaviour)
        planned = m.plan_steps([step], start, 'normal')[0]
        # Mirror main()'s emit, which adds its own 't' key (2026-10-11 20:05 crash).
        emit = lambda event, **data: dict(event=event, t=0.0, **data)
        out = m.run_step(arm, tr, planned, emit, lambda: False, self.limits,
                         clock=clock, sleep=clock.sleep)
        return out, arm, clock, planned

    def test_transfer_arrives_once_without_chunks(self):
        step = m.build_steps(self.targets, 'p1')[3]                       # p4 -> p5
        start = self.targets[3][:]
        out, arm, clock, planned = self._run(step, start)
        self.assertEqual(len(arm.writes), len(planned['profile']['joints']))   # one write per joint
        self.assertLessEqual(max(abs(a - b) for a, b in zip(out['feedback'], step['target'])), m.ARRIVE_TOL)
        self.assertLess(out['elapsed'], planned['profile']['T'] + 0.3)

    def test_gravity_droop_accepted_after_stop(self):
        step = m.build_steps(self.targets, 'p1')[3]
        start = self.targets[3][:]
        out, *_ = self._run(step, start, {2: lambda v: v - 30})            # ID2 sags 30 counts
        self.assertEqual(out['name'], 'p5')

    def test_blocked_joint_times_out_and_holds_here(self):
        step = m.build_steps(self.targets, 'p1')[3]
        start = self.targets[3][:]
        stuck = start[3]
        with self.assertRaises(m.SmoothAbort):
            self._run(step, start, {4: lambda v: stuck if v != stuck else v})
        hold = [c for c in self.raw.call_args_list][-6:]
        self.assertEqual(len(hold), 6)

    def test_runaway_outside_box_holds(self):
        step = m.build_steps(self.targets, 'p1')[3]
        start = self.targets[3][:]
        with self.assertRaises(m.SmoothAbort):
            self._run(step, start, {5: lambda v: v + 200})

    def test_fault_while_carrying_keeps_grip_squeezed(self):
        step = next(s for s in m.build_steps(self.targets, 'p1', m.GRIP_SQUEEZE) if s['name'] == 'p5')
        start = self.targets[3][:5] + [self.targets[2][5] + m.GRIP_SQUEEZE]   # ID6 goal = squeezed close goal
        clock, tr = FakeClock(), transport()
        arm = FakeArm(tr, start, clock, {6: lambda v: 2060, 5: lambda v: v + 200})   # jaws on handle; ID5 runs away
        planned = m.plan_steps([step], start, 'gentle')[0]
        with self.assertRaises(m.SmoothAbort):
            m.run_step(arm, tr, planned, lambda event, **d: None, lambda: False, self.limits,
                       clock=clock, sleep=clock.sleep)
        hold = dict((sid, pos) for sid, pos, *_ in arm.writes[-6:])
        self.assertEqual(hold[6], start[5])            # not 2060: that would relax the jaws
        self.assertEqual(sorted(hold), [1, 2, 3, 4, 5, 6])

    def test_fault_without_carry_holds_feedback(self):
        step = m.build_steps(self.targets, 'p1')[3]
        start = self.targets[3][:]
        clock, tr = FakeClock(), transport()
        arm = FakeArm(tr, start, clock, {5: lambda v: v + 200})
        with self.assertRaises(m.SmoothAbort):
            m.run_step(arm, tr, m.plan_steps([step], start, 'gentle')[0], lambda event, **d: None,
                       lambda: False, self.limits, clock=clock, sleep=clock.sleep)
        self.assertEqual(arm.writes[-1][0], 6)
        self.assertEqual(arm.writes[-1][1], arm._at(6))

    def test_gripper_close_stall_is_accepted(self):
        steps = m.build_steps(self.targets, 'p1')
        start = self.targets[1][:]
        stall = start[5] + 600                                             # handle stops the fingers
        out, *_ = self._run(steps[1], start, {6: lambda v: min(v, stall)})
        self.assertEqual(out['feedback'][5], stall)

    def test_frames_pass_the_cmd3_whitelist(self):
        step = m.build_steps(self.targets, 'p1')[0]
        self._run(step, self.targets[0][:])
        for call in self.raw.call_args_list:
            cmd, args = call.args[0], call.args[1]
            if cmd == 3:
                self.assertEqual(args[0], 41)


class PlanTests(unittest.TestCase):
    def test_plan_binding(self):
        pins = dict(route='a', limits='b')
        plan = dict(schema=m.SCHEMA, empty_load_only=True, start_mode='p8', profile='normal', cycles=1, pins=pins)
        m.validate_plan(plan, pins)
        for key, bad in (('profile', 'ludicrous'), ('cycles', 3), ('start_mode', 'p4'), ('pins', {})):
            p = dict(plan, **{key: bad})
            with self.assertRaises(r.SafetyError):
                m.validate_plan(p, pins)
        with self.assertRaises(r.SafetyError):            # empty plan refused under --loaded
            m.validate_plan(plan, pins, loaded=True)

    def test_loaded_plan_binding(self):
        pins = dict(route='a', limits='b')
        plan = dict(schema=m.SCHEMA, load='basket', empty_load_only=False, start_mode='p8',
                    profile='gentle', cycles=1, pins=pins)
        m.validate_plan(plan, pins, loaded=True)
        for bad in (dict(plan, profile='normal'), dict(plan, empty_load_only=True), dict(plan, load='brick')):
            with self.assertRaises(r.SafetyError):
                m.validate_plan(bad, pins, loaded=True)
        with self.assertRaises(r.SafetyError):            # basket plan refused without --loaded
            m.validate_plan(plan, pins)
        with self.assertRaises(r.SafetyError):            # empty_load_only=false needs load=basket
            m.validate_plan(dict(plan, load='empty'), pins)


if __name__ == '__main__':
    unittest.main()
