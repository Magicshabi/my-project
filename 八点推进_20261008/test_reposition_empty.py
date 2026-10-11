import json
import unittest
from unittest.mock import patch,Mock
import reposition_empty as m


class SetupTests(unittest.TestCase):
    def setUp(self):
        self.targets=m.r.resolve_route(json.loads(m.r.SOURCE.read_text(encoding='utf-8')))

    def test_base_only_preserves_five_other_joints_and_input(self):
        old=[v[:] for v in self.targets]
        start,end=m.targets_for_leg(self.targets,'base')
        self.assertEqual(start,self.targets[7]);self.assertEqual(end[0],self.targets[0][0])
        self.assertEqual(end[1:],start[1:]);self.assertEqual(self.targets,old)

    def test_second_leg_starts_exactly_at_first_leg_end(self):
        _,transit=m.targets_for_leg(self.targets,'base')
        start,end=m.targets_for_leg(self.targets,'approach')
        self.assertEqual(start,transit);self.assertEqual(end,self.targets[0])

    def test_plan_does_not_allow_second_leg_implicitly(self):
        plan=dict(schema='nexarm.empty-setup-trial/1',source_sha256='r',limits_sha256='l',
                  empty_load_only=True,allowed_legs=['base'],historical_hold_sha256='h')
        with patch.object(m.r,'sha',return_value='h'):
            m.validate_setup(plan,'r','l','base')
            with self.assertRaises(m.r.SafetyError):m.validate_setup(plan,'r','l','approach')
            with self.assertRaises(m.r.SafetyError):m.validate_setup(plan,'wrong','l','base')

    def test_direct_setup_sends_once_per_changed_joint_and_checks_start(self):
        limits=m.r.load_limits();before=[1616,2055,2420,3088,2471,1101]
        target=self.targets[0];arm=Mock()
        arm.move_joint_counts.side_effect=lambda *args:arm.transport.pending.popleft()
        with patch.object(m.r,'capture_stable',return_value=(before,[])),patch.object(m.r,'sha',return_value='pin'),patch.object(m.r,'wait_step',return_value=target):
            result=m.execute_direct_p1(arm,before,target,Mock(),lambda:False,'pin',limits,'pin')
            self.assertTrue(result['controller_converged'])
            self.assertEqual([c.args[0] for c in arm.move_joint_counts.call_args_list],[2,3,5,6])
            arm.reset_mock();wrong=before[:];wrong[0]+=13
            with self.assertRaises(m.r.SafetyError):
                m.execute_direct_p1(arm,wrong,target,Mock(),lambda:False,'pin',limits,'pin')
            arm.move_joint_counts.assert_not_called()

    def test_direct_setup_timeout_never_retries(self):
        before=[1616,2055,2420,3088,2471,1101];arm=Mock()
        arm.move_joint_counts.side_effect=lambda *args:arm.transport.pending.popleft()
        with patch.object(m.r,'capture_stable',return_value=(before,[])),patch.object(m.r,'sha',return_value='pin'),patch.object(m.r,'wait_step',side_effect=TimeoutError):
            with self.assertRaises(TimeoutError):
                m.execute_direct_p1(arm,before,self.targets[0],Mock(),lambda:False,'pin',m.r.load_limits(),'pin')
        self.assertEqual(arm.move_joint_counts.call_count,4)

    def test_forward_segment_holds_body_during_grip(self):
        before=[1616,1556,2613,3088,2679,1080];goals=[]
        def fake(arm,start,target,*args):
            goals.append(target);return dict(returned_report=target[:])
        with patch.object(m,'execute_direct_p1',side_effect=fake):
            report=m.execute_forward_segment(Mock(),before,self.targets[2],Mock(),lambda:False,
                    'r',m.r.load_limits(),'l',[10,10,20,10,10,10],True)
        self.assertTrue(all(g[:5]==before[:5] for g in goals))
        self.assertEqual(report['returned_report'][5],self.targets[2][5])
        previous=before
        for g in goals:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous,g)),450);previous=g

    def test_segment_timeout_prevents_remaining_batches(self):
        with patch.object(m,'execute_direct_p1',side_effect=TimeoutError) as send:
            with self.assertRaises(TimeoutError):
                m.execute_forward_segment(Mock(),self.targets[2],self.targets[3],Mock(),lambda:False,
                    'r',m.r.load_limits(),'l',[10]*6,False)
            self.assertEqual(send.call_count,1)

    def test_segment_scope_is_forward_and_explicit(self):
        for p in ({},{'from_point':8,'to_point':7},{'from_point':1,'to_point':3}):
            with self.assertRaises(m.r.SafetyError):m.segment_targets(self.targets,p)
        target,grip=m.segment_targets(self.targets,dict(from_point=2,to_point=3))
        self.assertTrue(grip);self.assertEqual(target,self.targets[2])


if __name__=='__main__':unittest.main()
