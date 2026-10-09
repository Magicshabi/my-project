import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import capture_joint_points as m
from nexarm.commands import Pose


def route():
    return dict(schema=m.SCHEMA, units='raw_encoder_counts', servo_ids=list(range(1,7)),
                configuration=None, complete=False, execution_verified=False, points=[])


def arm_at(counts):
    arm=Mock()
    arm.get_pose.return_value=Pose(1,2,3,4,5,6,tuple(counts))
    arm.get_real_joints.return_value=[(v,0) for v in counts]
    return arm


class CaptureTests(unittest.TestCase):
    def test_only_exact_queries_can_be_transmitted(self):
        t=m.ReadOnlyTransport(Mock())
        with patch.object(m.Transport,'send') as send:
            for cmd,args,sid in ((3,b'\x29\x64',1),(3,b'',255),(61,b'\x00',255),
                                 (8,b'',255),(58,b'\x00',255),(11,b'\x01',255),(65,b'',1)):
                with self.assertRaises(m.SafetyError):t.send(cmd,args,sid)
            send.assert_not_called()
            t.send(65)
            send.assert_called_once_with(65,b'',255)

    def test_stable_capture_reads_only_and_preserves_raw_counts(self):
        arm=arm_at([2000]*6)
        with patch.object(m.time,'sleep'):
            counts,samples=m.capture_stable(arm,Mock())
        self.assertEqual(counts,[2000]*6)
        self.assertEqual(len(samples),8)
        self.assertEqual(set(c[0] for c in arm.mock_calls),{'get_pose','get_real_joints'})

    def test_missing_or_disagreeing_feedback_rejected(self):
        arm=arm_at([2000]*6)
        for feedback in (None,[(2050,0)]*6):
            arm.get_real_joints.return_value=feedback
            with self.assertRaises(m.SafetyError):m.capture_stable(arm,Mock())

    def test_unstable_readings_rejected(self):
        arm=arm_at([2000]*6)
        arm.get_pose.side_effect=[Pose(1,2,3,4,5,6,tuple([2000+n]*6)) for n in range(8)]
        arm.get_real_joints.side_effect=[[(2000+n,0)]*6 for n in range(8)]
        with patch.object(m.time,'sleep'),self.assertRaises(m.SafetyError):
            m.capture_stable(arm,Mock())

    def test_order_overwrite_config_and_grip_pose_guard(self):
        r=route()
        with self.assertRaises(m.SafetyError):m.check_route(r,2)
        for point in (1,2):
            r=m.add_capture(r,point,[2000]*6,[],{'v':1},'observed',Path('.'))
        with self.assertRaises(m.SafetyError):m.check_route(r,2)
        with self.assertRaises(m.SafetyError):m.add_capture(r,3,[2000]*6,[],{'v':2},'observed',Path('.'))
        with self.assertRaises(m.SafetyError):m.add_capture(r,3,[2050]*6,[],{'v':1},'observed',Path('.'))
        updated=m.add_capture(r,3,[2000]*5+[2100],[],{'v':1},'observed',Path('.'))
        self.assertEqual(len(r['points']),2)
        self.assertFalse(updated['execution_verified'])

    def test_backup_readback_and_concurrent_change(self):
        with tempfile.TemporaryDirectory(prefix='capture_test_', dir=m.HERE) as folder:
            self.assertEqual(Path(folder).resolve().parent, m.HERE.resolve())
            p=Path(folder)/'points.json'
            original=json.dumps(route()).encode()
            p.write_bytes(original)
            updated=m.add_capture(route(),1,[2000]*6,[],{'v':1},'observed',Path(folder))
            m.save_revision(p,updated,original)
            self.assertEqual(json.loads(p.read_text(encoding='utf-8')),updated)
            self.assertEqual(list(p.parent.glob('*.bak'))[0].read_bytes(),original)
            with self.assertRaises(m.SafetyError):m.save_revision(p,route(),original)
            self.assertEqual(json.loads(p.read_text(encoding='utf-8')),updated)


if __name__=='__main__':unittest.main()
