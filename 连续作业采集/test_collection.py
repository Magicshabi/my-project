import contextlib
import copy
import io
import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone, timedelta
from pathlib import Path
from unittest.mock import patch

import collection as c
import capture_collection as cap
import progress_tracker as tracker

CONFIG = {'version': {'esp_str': '1.0.2', 'at32_str': '1.0.42'}, 'fixture': 'offline',
          'limits': [0]*12, 'kinematics': [0]*9, 'offsets': {str(i): 0 for i in range(1, 7)}}
COUNTS = [2045, 2025, 2075, 2135, 2100, 1079]


def samples(values):
    return [dict(cmd11_counts=values[:], cmd65_counts=values[:],
                 controller_pose=dict(x=0, y=0, z=150, pitch=0, roll=0, claw=0)) for _ in range(8)]


class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        c.init(self.root, 'offline-fixture')

    def route(self, n=8, color='红', target='P1'):
        doc = c.template(self.root, color, target)
        for point in range(1, n+1):
            values = COUNTS[:]
            carrying = 3 <= point <= 6
            values[5] = 2050 if carrying else 1079
            goal = None if 4 <= point <= 6 else (2193 if point == 3 else 1079)
            doc = cap.add_point(doc, self.root, color, target, point, values, samples(values),
                                CONFIG, goal, 'offline fixture only', self.root / 'fixture')
        return doc

    def save(self, doc, color='红', target='P1'):
        c.route_path(self.root, color, target).write_text(json.dumps(doc), encoding='utf-8')

    def test_init_refuses_overwrite(self):
        before = c.sha(self.root / 'scene.json')
        with self.assertRaises(ValueError):
            c.init(self.root, 'another')
        self.assertEqual(c.sha(self.root / 'scene.json'), before)

    def test_real_encoder_format_and_stalled_grip(self):
        doc = self.route()
        self.assertEqual(c.validate(doc, self.root, '红', 'P1', complete=True), 8)
        self.assertEqual(doc['points'][2]['encoder_positions'][5], 2050)
        self.assertEqual(doc['points'][2]['gripper_target_counts'], 2193)
        self.assertEqual([r['gripper_target_counts'] for r in doc['points'][3:6]], [2193]*3)
        self.assertFalse(doc['execution_verified'])

    def test_empty_fake_payload_is_rejected(self):
        self.save({'servos': []})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(tracker.verify_path('红', 'P1', self.root))

    def test_duplicate_exports_not_counted(self):
        path = c.route_path(self.root, '红', 'P1')
        for i in range(9):
            path.with_name(f'retake_{i}.json').write_text('{"servos":[]}', encoding='utf-8')
        with contextlib.redirect_stdout(io.StringIO()):
            total, errors = tracker.check_progress(self.root)
        self.assertEqual(total, 0)
        self.assertFalse(errors)

    def test_duplicate_point_and_wrong_identity_rejected(self):
        doc = self.route()
        doc['points'][1]['point'] = 1
        with self.assertRaises(ValueError):
            c.validate(doc, self.root, '红', 'P1')
        with self.assertRaises(ValueError):
            c.validate(self.route(), self.root, '蓝', 'P1')

    def test_numeric_and_sample_corruption_rejected(self):
        for mutate in (
            lambda d: d['points'][0].update(encoder_positions=[True]*6),
            lambda d: d['points'][0].update(encoder_positions=[9999]*6),
            lambda d: d['points'][0].update(samples=[]),
            lambda d: d['points'][0]['samples'][0]['cmd11_counts'].__setitem__(0, 2060),
            lambda d: d['points'][0]['samples'][0]['controller_pose'].update(z=float('nan')),
            lambda d: d.update(execution_verified=True),
            lambda d: d.update(configuration={}),
        ):
            doc = self.route()
            mutate(doc)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                c.validate(doc, self.root, '红', 'P1')

    def test_stability_even_when_both_channels_agree(self):
        doc = self.route()
        for key in ('cmd11_counts', 'cmd65_counts'):
            doc['points'][0]['samples'][0][key][0] += 6
        with self.assertRaisesRegex(ValueError, '不稳定'):
            c.validate(doc, self.root, '红', 'P1')

    def test_grip_event_body_motion_rejected(self):
        doc = self.route(2)
        values = COUNTS[:]
        values[0] += 13
        with self.assertRaisesRegex(ValueError, '臂体'):
            cap.add_point(doc, self.root, '红', 'P1', 3, values, samples(values), CONFIG,
                          2193, 'fixture', 'fixture')

    def test_carry_cannot_overwrite_grip(self):
        doc = self.route(3)
        with self.assertRaisesRegex(ValueError, '夹紧目标'):
            cap.add_point(doc, self.root, '红', 'P1', 4, COUNTS, samples(COUNTS), CONFIG,
                          2000, 'fixture', 'fixture')

    def test_skip_and_config_changes_rejected(self):
        doc = self.route(1)
        for point, config in ((3, CONFIG), (2, dict(CONFIG, fixture='changed'))):
            with self.assertRaises(ValueError):
                cap.add_point(doc, self.root, '红', 'P1', point, COUNTS, samples(COUNTS),
                              config, 1079, 'fixture', 'fixture')

    def test_layout_binding(self):
        doc = self.route()
        scene = c.read(self.root / 'scene.json')
        scene['layout_id'] = 'moved'
        (self.root / 'scene.json').write_text(json.dumps(scene), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '场地'):
            c.validate(doc, self.root, '红', 'P1')

    def test_cross_route_configuration_mismatch(self):
        self.save(self.route())
        other = self.route(1, '蓝', 'P2')
        other['configuration'] = dict(CONFIG, fixture='different')
        self.save(other, '蓝', 'P2')
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(tracker.verify_path('红', 'P1', self.root))

    def test_all_97_and_p0_are_counted_once(self):
        home = cap.add_point(c.template(self.root), self.root, None, None, 1, COUNTS,
                             samples(COUNTS), CONFIG, 1079, 'fixture', 'fixture')
        self.save(home, None, None)
        for color in c.COLORS:
            for target in c.TARGETS:
                self.save(self.route(8, color, target), color, target)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tracker.check_progress(self.root), (97, []))
            self.assertEqual(tracker.main(['--root', str(self.root), '--verify-all']), 0)

    def test_failed_cli_verification_exits_nonzero(self):
        result = subprocess.run([sys.executable, str(c.ROOT / 'progress_tracker.py'),
                                 '--root', str(self.root), '--verify', '红', 'P1'], capture_output=True)
        self.assertEqual(result.returncode, 1)

    def test_capture_cli_requires_port_and_identity(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            cap.main(['capture', '--home', '--point', '1', '--observed', 'fixture'])
        self.assertEqual(error.exception.code, 2)

    def test_capture_mocked_serial_saves_atomically_and_closes(self):
        sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
        import replay_joint_points as r
        import capture_joint_points as old
        lease = self.root / 'lease'
        lease.write_text('fixture', encoding='utf-8')
        fake = unittest.mock.MagicMock()
        fake.get_battery_mv.return_value = 12000
        with patch.object(r, 'require_device_lease', return_value=(lease, c.sha(lease),
                datetime.now(timezone.utc)+timedelta(minutes=5))) as lock_check, \
             patch.object(old, 'ArmController', return_value=fake), \
             patch.object(old, 'configuration', return_value=CONFIG), \
             patch.object(old, 'capture_stable', return_value=(COUNTS[:], samples(COUNTS))), \
             contextlib.redirect_stdout(io.StringIO()):
            code = cap.main(['--root', str(self.root), 'capture', '--home', '--point', '1',
                '--port', 'COM11', '--holder', 'claw', '--powered-sliders-confirmed',
                '--observed', 'offline fixture', '--grip-target', '1079'])
        self.assertEqual(code, 0)
        lock_check.assert_called_once_with('COM11', holder='claw')
        fake.close.assert_called_once()
        self.assertTrue(list(c.route_path(self.root).parent.glob('*.bak')))
        self.assertEqual(c.validate(c.read(c.route_path(self.root)), self.root, complete=True), 1)

    def test_capture_rejects_missing_lock_before_connecting(self):
        sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
        import replay_joint_points as r
        import capture_joint_points as old
        with patch.object(r, 'require_device_lease', side_effect=ValueError('no lease')), \
             patch.object(old, 'ArmController') as arm, contextlib.redirect_stderr(io.StringIO()):
            code = cap.main(['--root', str(self.root), 'capture', '--home', '--point', '1',
                '--port', 'COM11', '--holder', 'claw', '--powered-sliders-confirmed',
                '--observed', 'fixture', '--grip-target', '1079'])
        self.assertEqual(code, 1)
        arm.assert_not_called()

    def test_expired_lock_and_sampling_failure_preserve_file(self):
        sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
        import replay_joint_points as r
        import capture_joint_points as old
        lease = self.root / 'lease'
        lease.write_text('fixture', encoding='utf-8')
        before = c.route_path(self.root).read_bytes()
        for expired in (True, False):
            fake = unittest.mock.MagicMock()
            fake.get_battery_mv.return_value = 12000
            expiry = datetime.now(timezone.utc) + timedelta(minutes=-1 if expired else 5)
            with self.subTest(expired=expired), \
                 patch.object(r, 'require_device_lease', return_value=(lease, c.sha(lease), expiry)), \
                 patch.object(old, 'ArmController', return_value=fake), \
                 patch.object(old, 'configuration', return_value=CONFIG), \
                 patch.object(old, 'capture_stable', side_effect=ValueError('unstable')), \
                 contextlib.redirect_stdout(io.StringIO()):
                code = cap.main(['--root', str(self.root), 'capture', '--home', '--point', '1',
                    '--port', 'COM11', '--holder', 'claw', '--powered-sliders-confirmed',
                    '--observed', 'fixture', '--grip-target', '1079'])
            self.assertEqual(code, 1)
            self.assertEqual(c.route_path(self.root).read_bytes(), before)
            fake.close.assert_called_once()
            if expired:
                fake.connect_serial.assert_not_called()

    def test_transport_rejects_motion_torque_and_configuration(self):
        sys.path.insert(0, str(c.ROOT.parent / '八点推进_20261008'))
        import capture_joint_points as old
        transport = old.ReadOnlyTransport(lambda *args, **kwargs: None)
        for cmd in (3, 8, 61):
            with self.subTest(cmd=cmd), self.assertRaises(old.SafetyError):
                transport.send(cmd, b'\x01')


if __name__ == '__main__':
    unittest.main()
