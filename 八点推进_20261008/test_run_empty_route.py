import unittest
from unittest.mock import Mock, patch
import json
import run_empty_route as m


class CycleTests(unittest.TestCase):
    def setUp(self):
        self.limits=m.r.load_limits()
        self.targets=m.r.resolve_route(json.loads(m.r.SOURCE.read_text(encoding='utf-8')),self.limits)
        self.plan=dict(schema='nexarm.empty-cycle/1',empty_load_only=True,
            preparation='base-return',
            sequence=list(range(1,9)),cycles=1,settle_tolerances=[20]*6,pins={'a':'b'},
            expected_start=self.targets[7][:])

    def test_reject_scope_pin_and_start_changes(self):
        m.validate_plan(self.plan,{'a':'b'},self.targets,self.limits)
        for changes in (dict(cycles=2),dict(sequence=[8,1]),dict(pins={}),
                        dict(expected_start=self.targets[0]),dict(empty_load_only=False)):
            with self.assertRaises(m.r.SafetyError):
                m.validate_plan(dict(self.plan,**changes),{'a':'b'},self.targets,self.limits)

    def test_single_cycle_and_only_two_gripper_events(self):
        expected=self.targets[7][:]; called=[]
        def move(arm,current,target,*args):
            called.append((current[:],target[:],args[-1]))
            return {'returned_report':target[:]}
        with patch.object(m.r,'execute_segment',return_value={'returned_report':expected}) as base, \
             patch.object(m.setup,'execute_forward_segment',side_effect=move):
            result=m.run_cycle(Mock(),expected,self.targets,Mock(),lambda:False,'r',self.limits,'l',Mock())
        self.assertEqual(result,self.targets[7]);self.assertEqual(len(called),8)
        self.assertEqual([i+1 for i,c in enumerate(called) if c[2]],[3,7])
        self.assertEqual([c[1] for c in called],self.targets)
        self.assertEqual(base.call_args.kwargs['active_axes'],(1,))
        self.assertEqual(base.call_args.kwargs['tolerances'],[20]*6)
        self.assertEqual(base.call_args.args[2][1:],expected[1:])

    def test_resume_does_not_repeat_base_or_accept_wrong_side(self):
        plan=dict(self.plan,preparation='approach-p1')
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(plan,{'a':'b'},self.targets,self.limits)
        start=self.targets[7][:];start[0]=self.targets[0][0]
        plan['expected_start']=start
        m.validate_plan(plan,{'a':'b'},self.targets,self.limits)
        with patch.object(m.r,'execute_segment') as base, \
             patch.object(m.setup,'execute_forward_segment',side_effect=TimeoutError) as move:
            with self.assertRaises(TimeoutError):
                m.run_cycle(Mock(),start,self.targets,Mock(),lambda:False,'r',self.limits,'l',Mock(),
                            preparation='approach-p1')
            base.assert_not_called();self.assertEqual(move.call_count,1)

    def test_timeout_and_cancel_prevent_following_commands(self):
        for cancel in (True,False):
            with patch.object(m.r,'execute_segment',return_value={'returned_report':self.targets[7]}) as base, \
                 patch.object(m.setup,'execute_forward_segment',side_effect=TimeoutError) as move:
                with self.assertRaises((m.r.SafetyError,TimeoutError)):
                    m.run_cycle(Mock(),self.targets[7],self.targets,Mock(),lambda:cancel,'r',self.limits,'l',Mock())
                self.assertEqual(base.call_count,0 if cancel else 1)
                self.assertEqual(move.call_count,0 if cancel else 1)

    def test_accepted_p1_requires_confirmation_and_does_not_reissue_p1(self):
        start=self.targets[0][:];start[2]-=22
        plan=dict(self.plan,preparation='accepted-p1',expected_start=start,start_evidence='failed/result.json')
        with self.assertRaises(m.r.SafetyError):
            m.validate_plan(plan,{'a':'b'},self.targets,self.limits)
        plan['operator_accepted_p1']=True
        m.validate_plan(plan,{'a':'b'},self.targets,self.limits)
        with patch.object(m.r,'execute_segment') as base, \
             patch.object(m.r,'capture_stable',return_value=(start,[])), \
             patch.object(m.setup,'execute_forward_segment',side_effect=lambda *a:dict(returned_report=a[2])) as move:
            m.run_cycle(Mock(),start,self.targets,Mock(),lambda:False,'r',self.limits,'l',Mock(),preparation='accepted-p1')
        base.assert_not_called();self.assertEqual(move.call_count,7)
        self.assertEqual(move.call_args_list[0].args[2],self.targets[1])
        self.assertTrue(all(c.args[-2]==[20]*6 for c in move.call_args_list))

    def test_accepted_p1_drift_aborts_without_writes(self):
        start=self.targets[0][:];drift=start[:];drift[0]+=13
        with patch.object(m.r,'capture_stable',return_value=(drift,[])), \
             patch.object(m.setup,'execute_forward_segment') as move:
            with self.assertRaises(m.r.SafetyError):
                m.run_cycle(Mock(),start,self.targets,Mock(),lambda:False,'r',self.limits,'l',Mock(),preparation='accepted-p1')
            move.assert_not_called()


if __name__=='__main__':unittest.main()
