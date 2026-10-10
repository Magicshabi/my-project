import copy
import json
from collections import deque
import unittest
from unittest.mock import Mock,patch

import replay_joint_points as m
from nexarm.commands import Pose,pack_joint_counts


class Clock:
    def __init__(self):self.now=0
    def monotonic(self):return self.now
    def sleep(self,n):self.now+=n


class ReplayTests(unittest.TestCase):
    def setUp(self):
        # Synthetic, in-range route. Never rewrite or clip the actual teaching data.
        poses=[[1800,1700,2400,3100,2400,1200],
               [1800,1600,2450,3150,2400,1200],
               [1800,1600,2450,3150,2400,1966],
               [1820,1730,2350,3050,2400,2029],
               [2300,1730,2350,3050,2400,2029],
               [2300,1600,2450,3150,2400,2029],
               [2300,1600,2450,3150,2400,1200],
               [2300,1700,2390,3020,2400,1200]]
        self.route=dict(schema='nexarm.joint-eight/1',units='raw_encoder_counts',
                        servo_ids=list(range(1,7)),complete=True,
                        host_joint_limits=m.snapshot(m.load_limits()),points=[
                            dict(point=i+1,stage=m.STAGES[i],encoder_positions=p,
                                 physical_pose_confirmed=True) for i,p in enumerate(poses)])
        self.route['points'][2]['target_encoder_positions']=poses[2][:5]+[2029]
        self.targets=m.resolve_route(self.route)
        source_pin=patch.object(m,'sha',return_value=m.EXPECTED_SHA)
        source_pin.start();self.addCleanup(source_pin.stop)

    def arm(self,state,move=True):
        arm=Mock()
        arm.get_pose.side_effect=lambda:Pose(0,0,0,0,0,0,tuple(state))
        arm.get_real_joints.side_effect=lambda:[(v,0) for v in state]
        arm.transport.stats.return_value={'checksum_errors':0}
        def update(sid,p,acc,speed):
            expected=m.build_frame(3,pack_joint_counts(p,acc,speed),sid)
            self.assertEqual(arm.transport.pending.popleft(),expected)
            if move:state[sid-1]=p
        arm.move_joint_counts.side_effect=update
        return arm

    def test_override_and_reject_unapproved_override(self):
        self.assertEqual(self.targets[2],[1800,1600,2450,3150,2400,2029])
        bad=copy.deepcopy(self.route)
        bad['points'][2]['target_encoder_positions'][0]+=100
        with self.assertRaises(m.SafetyError):m.resolve_route(bad)

    def test_diagnostic_hold_blocks_before_connection(self):
        hold=Mock();hold.exists.return_value=True
        with patch.object(m,'DIAGNOSTIC_HOLD',hold),patch.object(m.sys,'argv',['probe','--go','--empty-load']),patch.object(m,'ArmController') as controller:
            with self.assertRaisesRegex(m.SafetyError,'ID4'):
                m.main()
            controller.assert_not_called()

    def test_all_steps_bounded_and_finish_exactly(self):
        for a,b in zip(m.DEFAULT_POINTS,m.DEFAULT_POINTS[1:]):
            previous=self.targets[a-1]
            goals=m.interpolate(previous,self.targets[b-1])
            for goal in goals:
                self.assertLessEqual(max(abs(x-y) for x,y in zip(previous,goal)),40)
                previous=goal
            self.assertEqual(previous,self.targets[b-1])
        with self.assertRaises(m.SafetyError):m.check_sequence([8,1])

    def test_whitelist_and_failed_write_are_terminal(self):
        t=m.ReplayTransport(Mock(),lambda:False)
        p=pack_joint_counts(2800,100,2000)
        t.pending=deque([m.build_frame(3,p,3)])
        with patch.object(m.Transport,'send',side_effect=IOError('short write')) as send:
            for sid in (255,2):
                with self.assertRaises(m.SafetyError):t.send(3,p,sid)
            send.assert_not_called()
            with self.assertRaises(IOError):t.send(3,p,3)
            with self.assertRaises(m.SafetyError):t.send(3,p,3)
            self.assertEqual(send.call_count,1)

    def test_stop_blocks_even_preapproved_frame(self):
        t=m.ReplayTransport(Mock(),lambda:True)
        p=pack_joint_counts(2800,100,2000)
        t.pending=deque([m.build_frame(3,p,3)])
        with patch.object(m.Transport,'send') as send:
            with self.assertRaises(m.SafetyError):t.send(3,p,3)
            send.assert_not_called()

    def test_complete_segment_remains_operator_unconfirmed(self):
        base=self.targets[7][:];state=base[:];arm=self.arm(state)
        with patch.object(m,'capture_stable',return_value=(base,[])),patch.object(m,'time',Clock()):
            result=m.execute_segment(arm,base,self.targets[6],Mock(),lambda:False)
        self.assertEqual(state,self.targets[6])
        self.assertFalse(result['operator_confirmed'])
        self.assertEqual(result['steps'],4)

    def test_old_real_route_is_rejected_before_serial_connection(self):
        hold=Mock();hold.exists.return_value=False
        pin=m.hashlib.sha256(m.SOURCE.read_bytes()).hexdigest()
        with patch.object(m,'DIAGNOSTIC_HOLD',hold),patch.object(m.sys,'argv',['probe','--go','--empty-load','--expected-sha',pin]),patch.object(m,'ArmController') as controller:
            with self.assertRaisesRegex(m.SafetyError,'第1点执行目标.*ID4=3448'):
                m.main()
            controller.assert_not_called()

    def test_source_bytes_must_match_reviewed_fingerprint_before_serial(self):
        hold=Mock();hold.exists.return_value=False
        with patch.object(m,'DIAGNOSTIC_HOLD',hold),patch.object(m.sys,'argv',['probe','--go','--empty-load','--expected-sha','incorrect']),patch.object(m,'ArmController') as controller:
            with self.assertRaisesRegex(m.SafetyError,'指纹改变'):m.main()
            controller.assert_not_called()

    def test_out_of_range_target_blocks_even_direct_segment_call(self):
        bad=self.targets[7][:];bad[3]=3414
        arm=Mock()
        with patch.object(m,'capture_stable') as capture:
            with self.assertRaisesRegex(m.SafetyError,'ID4=3414'):
                m.execute_segment(arm,self.targets[6],bad,Mock(),lambda:False)
            capture.assert_not_called()
        arm.move_joint_counts.assert_not_called()

    def test_whitelisted_out_of_range_frame_is_not_transmitted(self):
        t=m.ReplayTransport(Mock(),lambda:False)
        payload=pack_joint_counts(3414,100,2000)
        t.pending=deque([m.build_frame(3,payload,4)])
        with patch.object(m.Transport,'send') as send:
            with self.assertRaisesRegex(m.SafetyError,'ID4=3414'):
                t.send(3,payload,4)
            send.assert_not_called()
        self.assertEqual(t.motion_attempts,0)

    def test_limit_file_change_aborts_before_motion(self):
        base=self.targets[7][:];arm=self.arm(base[:])
        with patch.object(m,'capture_stable',return_value=(base,[])):
            with self.assertRaisesRegex(m.SafetyError,'限位文件被更改'):
                m.execute_segment(arm,base,self.targets[6],Mock(),lambda:False,
                                  limits_sha='different-policy-fingerprint')
        arm.move_joint_counts.assert_not_called()

    def test_wrong_units_and_unconfirmed_powered_pose_rejected(self):
        r=copy.deepcopy(self.route);r['units']='degrees'
        with self.assertRaises(m.SafetyError):m.resolve_route(r)
        r=copy.deepcopy(self.route);r['require_powered_sliders']=True
        with self.assertRaisesRegex(m.SafetyError,'电机主动到达确认'):m.resolve_route(r)

    def test_start_mismatch_no_motion(self):
        base=self.targets[7][:];wrong=base[:];wrong[0]+=13
        arm=Mock()
        with patch.object(m,'capture_stable',return_value=(wrong,[])):
            with self.assertRaises(m.SafetyError):m.execute_segment(arm,base,self.targets[6],Mock(),lambda:False)
        arm.move_joint_counts.assert_not_called()

    def test_no_follow_timeout_no_later_steps(self):
        base=self.targets[7][:];state=base[:];arm=self.arm(state,move=False)
        with patch.object(m,'capture_stable',return_value=(base,[])),patch.object(m,'time',Clock()):
            with self.assertRaises(TimeoutError):m.execute_segment(arm,base,self.targets[6],Mock(),lambda:False)
        first=m.interpolate(base,self.targets[6])[0]
        self.assertEqual(arm.move_joint_counts.call_count,sum(x!=y for x,y in zip(base,first)))

    def test_drift_or_missing_feedback_aborts(self):
        base=self.targets[7][:];bad=base[:];bad[0]+=30
        arm=self.arm(bad)
        with patch.object(m,'time',Clock()),self.assertRaises(m.SafetyError):
            m.wait_step(arm,base,base,Mock(),lambda:False)
        arm.get_pose.side_effect=None;arm.get_pose.return_value=None
        with self.assertRaises(m.SafetyError):m.reads(arm)


if __name__=='__main__':unittest.main()
