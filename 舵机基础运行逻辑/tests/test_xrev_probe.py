import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

LIB=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(LIB))
spec=importlib.util.spec_from_file_location('xrev_probe',LIB/'examples/10_claw_strategy_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from nexarm import ArmController, SafetyError, Transport, build_frame
from nexarm.commands import pack_joint_counts


class ProbeTests(unittest.TestCase):
    def test_logged_gui_frame_exact(self):
        self.assertEqual(build_frame(3,pack_joint_counts(1988,100,2000),5).hex(),
                         'ffff050a032964c4070000d007be')
    def test_library_preserves_servo_id(self):
        t=Mock();a=ArmController(transport=t,log=lambda s:None);a.enable_motion(True)
        a.move_joint_counts(5,1988,100,2000)
        t.send.assert_called_once_with(3,pack_joint_counts(1988,100,2000),target_id=5)
    def test_only_delta50_and_correct_cmd51_duration(self):
        p=m.plan_request([[2000]*5+[1875]]*5,-15,(-100,83),'cmd51')
        self.assertEqual((p['id'],p['cmd'],p['target']),(255,51,1825))
        self.assertEqual(p['args'],'062107e803')
    def test_unstable_or_boundary_baseline_refused(self):
        for s in ([[2000]*5+[1240]]*5,[[2000]*5+[1875]]*4+[[2000]*5+[1880]]):
            with self.assertRaises(SafetyError):m.plan_request(s,-15,(-100,83),'cmd51')
    def test_transport_refuses_torque_and_second_motion(self):
        t=m.StrategyTransport(Mock());t.allowed=(255,51,bytes.fromhex('062107e803'))
        with patch.object(Transport,'send') as sent:
            with self.assertRaises(SafetyError):t.send(61,b'\x01')
            t.send(51,bytes.fromhex('062107e803'))
            with self.assertRaises(SafetyError):t.send(51,bytes.fromhex('062107e803'))
            self.assertEqual(sent.call_count,1)
    def test_write_failure_does_not_enable_retry(self):
        t=m.StrategyTransport(Mock());t.allowed=(255,51,b'test')
        with patch.object(Transport,'send',side_effect=RuntimeError('write failed')) as sent:
            with self.assertRaises(RuntimeError):t.send(51,b'test')
            with self.assertRaises(SafetyError):t.send(51,b'test')
            self.assertEqual(sent.call_count,1)


if __name__=='__main__':unittest.main()
