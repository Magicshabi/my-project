"""Offline tests of the actual host adapter, parser and feedback decisions."""
from collections import deque
import struct
import unittest
from unittest.mock import patch
import official_replay as r


def reply(cmd, payload, sid=255):
    b = bytes([sid,len(payload)+2,cmd])+payload
    return b'\xff\xff'+b+bytes([(~sum(b)) & 255])


class AdapterTests(unittest.TestCase):
    def test_official_slider_wire(self):
        b = r.frame(51,struct.pack('<BhH',6,1409,1000))
        self.assertEqual(b.hex(),'ffffff0733068105e8034f')
        self.assertEqual(sum(b[2:]) & 255,255)

    def test_no_dangerous_configuration_commands(self):
        for cmd in range(256):
            if cmd not in (1,51,65):
                with self.assertRaises(r.Refused):
                    r.frame(cmd)
        for sid,pos,t in [(0,1000,1000),(7,1000,1000),(1,-1,1000),
                          (1,4096,1000),(1,1000,0),(1,1000,30000)]:
            with self.assertRaises(r.Refused):
                r.frame(51,struct.pack('<BhH',sid,pos,t))

    def test_parser_fragmented_mixed_log_and_bad_checksum(self):
        good = reply(65,struct.pack('<12h',*[x for p in range(1000,1006) for x in (p,-200)]))
        bad = good[:-1]+bytes([good[-1]^1])
        stream = b'log line\n'+bad+b'noise'+good
        parser = r.Parser()
        frames=[]
        for b in stream:
            frames.extend(parser.feed(bytes([b])))
        self.assertEqual(len(frames),1)
        self.assertEqual(r.actual_positions(frames[0][2]),tuple(range(1000,1006)))
        self.assertGreater(parser.bad,0)

    def test_feedback_decoder_uses_stride_not_cached_xyz(self):
        p=struct.pack('<12h',1000,123,1100,124,1200,125,1300,126,1400,127,1500,128)
        self.assertEqual(r.actual_positions(p),(1000,1100,1200,1300,1400,1500))
        for bad in [p[:12],p+b'\x00',struct.pack('<12h',4096,*([0]*11))]:
            with self.assertRaises(r.Refused):r.actual_positions(bad)

    def test_all_eight_paths_preserve_endpoints_and_bound_each_step(self):
        taught=r.points()
        for a,b in zip(taught,taught[1:]):
            prev=tuple(a['encoder_positions'])
            steps=r.path_steps(prev,b['encoder_positions'])
            self.assertEqual(steps[-1],tuple(b['encoder_positions']))
            for s in steps:
                self.assertLessEqual(max(abs(x-y) for x,y in zip(prev,s)),r.STEP)
                prev=s

    def test_five_fresh_stable_samples_required(self):
        p=(1000,)*6
        good=[(1+i*.11,p) for i in range(5)]
        self.assertTrue(r.stable_tail(good,p,after=.9))
        self.assertFalse(r.stable_tail(good[:4],p))
        self.assertFalse(r.stable_tail(good,p,after=1))
        self.assertFalse(r.stable_tail([(1+i*.001,p) for i in range(5)],p))
        unstable=good[:-1]+[(1.5,(1003,1000,1000,1000,1000,1000))]
        self.assertFalse(r.stable_tail(unstable,p))
        self.assertFalse(r.stable_tail(good,(1010,)*6))

    def test_current_rejects_stale_feedback(self):
        d=r.Device.__new__(r.Device)
        d.samples=deque([(1,(1000,)*6)])
        with patch.object(r.time,'monotonic',return_value=2):
            with self.assertRaises(r.Refused):d.current()

    def test_position_jump_sends_nothing(self):
        d=r.Device.__new__(r.Device)
        d.current=lambda:(1000,)*6
        sent=[]
        d.send=lambda *a:sent.append(a)
        with self.assertRaises(r.Refused):d.move_step((1500,)*6)
        self.assertEqual(sent,[])

    def test_gripper_probe_writes_only_id6(self):
        d=r.Device.__new__(r.Device)
        d.current=lambda:(1500,)*6
        sent=[]
        d.send=lambda cmd,payload:sent.append((cmd,struct.unpack('<BhH',payload)))
        d.settle=lambda target,*args:target
        target=(1500,1500,1500,1500,1500,1492)
        d.move_step(target)
        self.assertEqual(sent,[(51,(6,1492,1000))])

    def test_abort_without_motion_never_writes(self):
        d=r.Device.__new__(r.Device)
        d.motion_sent=False
        d.send=lambda *a:self.fail('Unexpected abort write')
        d.hold_after_abort()

    def simulated_device(self, feedback):
        d=r.Device.__new__(r.Device)
        clock=[0.0]
        d.samples=deque([(0.0,(1500,)*6)],maxlen=40)
        def poll():
            clock[0]+=.11
            if feedback is not None:d.samples.append((clock[0],feedback))
        d.poll=poll
        return d,clock

    def test_no_follow_times_out_instead_of_accepting_stable_old_position(self):
        d,clock=self.simulated_device((1500,)*6)
        target=(1500,1500,1500,1500,1500,1492)
        with patch.object(r.time,'monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(r.Refused,'did not settle'):
                d.settle(target,(1500,)*6,0,[5])

    def test_other_axis_excursion_aborts(self):
        d,clock=self.simulated_device((1520,1500,1500,1500,1500,1492))
        with patch.object(r.time,'monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(r.Refused,'Joint 1 outside'):
                d.settle((1500,1500,1500,1500,1500,1492),(1500,)*6,0,[5])

    def test_feedback_loss_aborts_before_next_step(self):
        d,clock=self.simulated_device(None)
        with patch.object(r.time,'monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(r.Refused,'stale'):
                d.settle((1500,1500,1500,1500,1500,1492),(1500,)*6,0,[5])

    def test_failed_step_does_not_advance_segment(self):
        d=r.Device.__new__(r.Device)
        d.current=lambda:(1500,)*6
        d.emit=lambda *a,**kw:None
        targets=[]
        def fail(target):
            targets.append(target)
            raise r.Refused('mock feedback loss')
        d.move_step=fail
        with self.assertRaises(r.Refused):d.goto((1600,)*6,'test',350)
        self.assertEqual(len(targets),1)

    def test_abort_hold_only_touched_axes_and_requires_freshness(self):
        d=r.Device.__new__(r.Device)
        d.motion_sent=True;d.tx_failed=False;d.touched={6}
        d.samples=deque([(1,(1500,)*6)])
        d.emit=lambda *a,**kw:None
        sent=[]
        d.send=lambda cmd,payload:sent.append((cmd,struct.unpack('<BhH',payload)))
        with patch.object(r.time,'monotonic',return_value=1.1):d.hold_after_abort()
        self.assertEqual(sent,[(51,(6,1500,1000))])
        sent.clear()
        with patch.object(r.time,'monotonic',return_value=2):d.hold_after_abort()
        self.assertEqual(sent,[])


if __name__=='__main__':unittest.main()
