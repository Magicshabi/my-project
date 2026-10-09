import unittest
from unittest.mock import Mock,patch
import verify_six_joints as m
from nexarm.commands import Pose,pack_joint_counts


class Clock:
    def __init__(self):self.now=0
    def monotonic(self):return self.now
    def sleep(self,n):self.now+=n


class SixTests(unittest.TestCase):
    def test_exact_golden_frame_and_six_distinct_joints(self):
        self.assertEqual(m.build_frame(3,pack_joint_counts(1299,100,2000),6).hex(),
                         'ffff060a03296413050000d00770')
        p=m.make_plan(list(m.EXPECTED))
        self.assertEqual([x['id'] for x in p],[6,5,4,3,2,1])
        self.assertTrue(all(x['start']==m.EXPECTED[x['id']-1] for x in p))
    def test_changed_start_refused(self):
        base=list(m.EXPECTED);base[3]+=13
        with self.assertRaises(m.SafetyError):m.make_plan(base)
    def test_wrong_axis_or_broadcast_or_extra_write_refused(self):
        t=m.GuardedTransport(Mock());p=pack_joint_counts(1302,100,2000)
        t.pending=[m.build_frame(3,p,6)]
        with patch.object(m.Transport,'send') as send:
            for sid in (5,255):
                with self.assertRaises(m.SafetyError):t.send(3,p,sid)
            t.send(3,p,6)
            with self.assertRaises(m.SafetyError):t.send(3,p,6)
            send.assert_called_once()
    def test_complete_round_sends_out_and_original_only(self):
        arm=Mock();base=list(m.EXPECTED);state=base[:]
        def move(sid,p,acc,speed):state[sid-1]=p
        arm.move_joint_counts.side_effect=move
        arm.get_pose.side_effect=lambda:Pose(264,31,159,-41.6,-5,-22,tuple(state))
        with patch.object(m,'time',Clock()):
            result=m.run_round(arm,m.make_plan(base)[0],base,Mock(),lambda:False)
        self.assertEqual([c.args[1] for c in arm.move_joint_counts.call_args_list],[1302,1802])
        self.assertEqual(result['return_error'],0)
        self.assertFalse(result['operator_confirmed'])
    def test_no_following_no_return_or_retry(self):
        arm=Mock();base=list(m.EXPECTED)
        arm.get_pose.return_value=Pose(264,31,159,-41.6,-5,-22,tuple(base))
        with patch.object(m,'time',Clock()):
            with self.assertRaises(TimeoutError):m.run_round(arm,m.make_plan(base)[0],base,Mock(),lambda:False)
        arm.move_joint_counts.assert_called_once()
    def test_other_axis_drift_aborts(self):
        arm=Mock();bad=list(m.EXPECTED);bad[0]+=13
        arm.get_pose.return_value=Pose(264,31,159,-41.6,-5,-22,tuple(bad))
        with patch.object(m,'time',Clock()):
            with self.assertRaises(m.SafetyError):m.wait_report(arm,6,1302,list(m.EXPECTED),Mock(),lambda:False)


if __name__=='__main__':unittest.main()
