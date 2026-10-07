import unittest
from unittest.mock import Mock
import struct
import nexarm_exe_probe as n


class ExeProbeTests(unittest.TestCase):
    def test_wire_matches_extracted_exe_algorithm_and_defaults(self):
        self.assertEqual(n.exe_frame(6,1359).hex(), 'ffff060a0329644f050000d00734')

    def test_other_axes_and_outside_interior_refused(self):
        for sid,pos in [(1,1359),(5,1359),(255,1359),(6,1099),(6,1901)]:
            with self.assertRaises(n.replay.Refused): n.exe_frame(sid,pos)

    def test_motion_intent_tracked_and_short_write_never_retried(self):
        d=n.ExeProbeDevice.__new__(n.ExeProbeDevice)
        d.s=Mock(); d.s.write.return_value=2
        d.emit=Mock(); d.touched=set(); d.motion_sent=False; d.tx_failed=False
        with self.assertRaises(n.replay.Refused):
            d.send(51,struct.pack('<BhH',6,1359,1000))
        self.assertTrue(d.motion_sent); self.assertTrue(d.tx_failed)
        self.assertEqual(d.touched,{6}); d.s.write.assert_called_once()


if __name__ == '__main__': unittest.main()
