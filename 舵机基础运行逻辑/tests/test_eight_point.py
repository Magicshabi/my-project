import copy
from dataclasses import asdict
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nexarm import ArmController
from nexarm.commands import Pose
from nexarm.eight_point import *
from nexarm.transport import Transport, TransportError
from nexarm.safety import WorkspaceLimits


def fixture():
    # Synthetic geometry for offline tests only, never exported as user's calibration.
    data=[(200,0,180,-90,0,-60),(200,0,120,-90,0,-60),
          (200,0,120,-90,0,0),(200,0,180,-90,0,0),
          (220,80,180,-90,0,0),(220,80,120,-90,0,0),
          (220,80,120,-90,0,-60),(220,80,180,-90,0,-60)]
    return dict(schema='nexarm.cartesian-eight/1',units='mm,deg',configuration={},
                path_observed=False,points=[dict(stage=s,pose=dict(zip(AXES,p)),
                  physical_confirmed=True,observation='offline fixture') for s,p in zip(STAGES,data)])


class Clock:
    def __init__(self): self.now=0.0
    def monotonic(self): return self.now
    def sleep(self,n): self.now+=n


class EightPointTests(unittest.TestCase):
    def setUp(self):
        self.route=fixture(); self.limits=WorkspaceLimits()

    def test_order_exact_endpoints_contact_speed_and_grip_dwell(self):
        plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        self.assertEqual([s.stage for s in plan],list(STAGES))
        self.assertEqual([s.pose for s in plan],[r['pose'] for r in self.route['points']])
        self.assertEqual(plan[1].duration_ms,6000)
        self.assertEqual(plan[2].dwell_ms,400)
        self.assertEqual(plan[6].dwell_ms,400)
        self.assertEqual(plan[2].pose['x'],plan[1].pose['x'])

    def test_missing_or_unobserved_pose_rejected(self):
        for change in ({'pose':None},{'physical_confirmed':False},{'observation':''}):
            r=copy.deepcopy(self.route); r['points'][0].update(change)
            with self.assertRaises(SafetyError): validate_route(r,self.limits)

    def test_old_encoder_only_points_cannot_be_sent_as_xyz(self):
        original=Path(__file__).resolve().parents[2]/'validation_runs/readout_20261005_180008/eight_points.json'
        r=template(original)
        self.assertEqual(r['points'][0]['legacy_encoders'],[2043,2169,1798,2076,2106,1399])
        self.assertIsNone(r['points'][0]['pose'])
        with self.assertRaises(SafetyError): validate_route(r,self.limits)

    def test_grip_cannot_move_arm_and_approach_cannot_move_sideways(self):
        for idx,key in ((2,'x'),(1,'y'),(6,'pitch')):
            r=copy.deepcopy(self.route); r['points'][idx]['pose'][key]+=10
            with self.assertRaises(SafetyError): validate_route(r,self.limits)

    def test_clearance_and_far_start_refused(self):
        r=copy.deepcopy(self.route); r['points'][3]['pose']['z']=130
        with self.assertRaises(SafetyError): validate_route(r,self.limits)
        with self.assertRaises(SafetyError): make_plan(self.route,self.limits,dict(x=400,y=0,z=180,pitch=-90,roll=0,claw=-60))

    def test_protocol_quantization_checked_before_limits(self):
        self.assertEqual(checked_pose(dict(x=200.8,y=0,z=180,pitch=-89.96,roll=.1,claw=-60))['x'],201)
        with self.assertRaises(SafetyError): checked_pose(dict(x=float('nan'),y=0,z=180,pitch=-90,roll=0,claw=-60))

    def test_timeout_stops_route_before_second_command(self):
        arm=Mock(); arm.get_pose.return_value=Pose(200,0,180,-90,0,-60,())
        arm.wait_for.return_value=False
        plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        clock=Clock()
        with patch('nexarm.eight_point.configuration',return_value={}),patch('nexarm.eight_point.time',clock):
            with self.assertRaises(TimeoutError): execute(arm,self.route,plan,lambda s:True,Mock())
        self.assertEqual(arm.move_to.call_count,1)

    def test_continuous_needs_path_review_and_never_fabricates_observation(self):
        arm=Mock(); plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        with self.assertRaises(SafetyError): execute(arm,self.route,plan,None,Mock())
        arm.move_to.assert_not_called()

    def test_config_change_rejected_before_motion(self):
        arm=Mock(); plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        with patch('nexarm.eight_point.configuration',return_value={'offsets':{'1':10}}):
            with self.assertRaises(SafetyError): execute(arm,self.route,plan,lambda s:True,Mock())
        arm.move_to.assert_not_called()

    def test_complete_route_keeps_eight_full_targets_without_fake_physical_proof(self):
        self.route['path_observed']=True
        plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        arm=Mock(); state=dict(plan[0].pose)
        arm.get_pose.side_effect=lambda: Pose(**state,servos=())
        def moved(**kwargs):
            state.update({k:kwargs[k] for k in AXES})
        arm.move_to.side_effect=moved; arm.wait_for.return_value=True
        emit=Mock()
        with patch('nexarm.eight_point.configuration',return_value={}),patch('nexarm.eight_point.time',Clock()):
            result=execute(arm,self.route,plan,None,emit)
        self.assertEqual(arm.move_to.call_count,8)
        self.assertEqual([c.kwargs['claw'] for c in arm.move_to.call_args_list],[-60,-60,0,0,0,0,-60,-60])
        self.assertEqual(result['physical_evidence'],'pending observation')
        self.assertFalse(result['measured_accuracy_verified'])
        self.assertNotIn('operator_confirmed',[c.args[0] for c in emit.call_args_list])

    def test_drift_after_confirmation_blocks_next_segment(self):
        plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        arm=Mock(); state=dict(plan[0].pose)
        arm.get_pose.side_effect=lambda: Pose(**state,servos=())
        arm.wait_for.return_value=True
        def confirm(segment):
            state['x']+=30
            return True
        with patch('nexarm.eight_point.configuration',return_value={}),patch('nexarm.eight_point.time',Clock()):
            with self.assertRaises(SafetyError): execute(arm,self.route,plan,confirm,Mock())
        self.assertEqual(arm.move_to.call_count,1)

    def test_cancel_does_not_send_a_return_or_torque_command(self):
        arm=Mock(); arm.get_pose.return_value=Pose(200,0,180,-90,0,-60,())
        plan=make_plan(self.route,self.limits,self.route['points'][0]['pose'])
        with patch('nexarm.eight_point.configuration',return_value={}):
            with self.assertRaises(SafetyError): execute(arm,self.route,plan,lambda s:True,Mock(),lambda:True)
        arm.move_to.assert_not_called(); arm.set_torque.assert_not_called()

    def test_existing_move_to_propagates_failed_wait(self):
        arm=ArmController(transport=Mock(),log=lambda m:None)
        arm.enable_motion(True); arm.wait_for=Mock(return_value=False)
        with self.assertRaises(TimeoutError): arm.move_to(200,0,180,pitch=-90,claw=-60)

    def test_claw_report_must_match_even_when_xyz_already_matches(self):
        arm=ArmController(transport=Mock(),log=lambda m:None)
        arm.get_pose=Mock(return_value=Pose(200,0,180,-90,0,-60,()))
        with patch('nexarm.arm.time',Clock()):
            self.assertFalse(arm.wait_for(200,0,180,claw=0,timeout=.5))

    def test_missing_report_resets_stability(self):
        arm=ArmController(transport=Mock(),log=lambda m:None)
        p=Pose(200,0,180,-90,0,-60,())
        arm.get_pose=Mock(side_effect=[p,p,None,p,p,None,p,p,None])
        with patch('nexarm.arm.time',Clock()):
            self.assertFalse(arm.wait_for(200,0,180,timeout=.8))

    def test_success_wait_respects_duration(self):
        arm=ArmController(transport=Mock(),log=lambda m:None)
        arm.get_pose=Mock(return_value=Pose(200,0,180,-90,0,-60,()))
        c=Clock()
        with patch('nexarm.arm.time',c):
            self.assertTrue(arm.wait_for(200,0,180,timeout=2,min_duration=1,claw=-60))
        self.assertGreaterEqual(c.now,1.2)

    def test_short_serial_write_fails_without_retry(self):
        t=Transport(); t._ser=Mock(); t._ser.write.return_value=2
        with self.assertRaises(TransportError): t.send(11)
        t._ser.write.assert_called_once(); self.assertFalse(t.connected)


if __name__=='__main__': unittest.main()
