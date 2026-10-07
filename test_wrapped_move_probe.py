"""Boundary checks for wrapped move probe r4: source shape + policy mirror."""
import unittest
from pathlib import Path

ROOT = Path(__file__).parent
SRC = ROOT / 'wrapped_move_probe_20261006/NexArm_Wrapped_Move_Probe'
WM_NONE, WM_UP_ONCE, WM_DOWN_RETURN = 0, 2, 3

def wrapped_tx_allowed(id_, cmd, a, n, permit):
    if id_ == 255 and cmd in (65, 11) and n == 0:
        return True
    if id_ != 255 or cmd != 50 or a is None or n != 14:
        return False
    if permit not in (WM_UP_ONCE, WM_DOWN_RETURN):
        return False
    dz = a[4] | (a[5] << 8)
    if dz >= 0x8000:
        dz -= 0x10000
    if permit == WM_UP_ONCE and dz != 30:
        return False
    if permit == WM_DOWN_RETURN and dz != -30:
        return False
    for i in range(14):
        if i in (4, 5):
            continue
        expect = 0xB8 if i == 12 else (0x0B if i == 13 else 0)
        if a[i] != expect:
            return False
    return True

UP = [0,0, 0,0, 30,0, 0,0, 0,0, 0,0, 0xB8,0x0B]
DOWN = [0,0, 0,0, 0xE2,0xFF, 0,0, 0,0, 0,0, 0xB8,0x0B]

def wire_frame(id_, cmd, args):
    body = [id_, len(args) + 2, cmd] + list(args)
    chk = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF] + body + [chk])

class WrappedMovePolicyMirrorTests(unittest.TestCase):
    def test_queries_zero_arg_only(self):
        for id_ in range(256):
            self.assertEqual(wrapped_tx_allowed(id_, 65, None, 0, WM_NONE), id_ == 255)
            self.assertEqual(wrapped_tx_allowed(id_, 11, None, 0, WM_NONE), id_ == 255)
        self.assertFalse(wrapped_tx_allowed(255, 65, UP, 1, WM_NONE))
        self.assertFalse(wrapped_tx_allowed(255, 11, UP, 14, WM_NONE))

    def test_only_wrapped_move_inc_passes(self):
        for id_ in range(256):
            for cmd in range(256):
                if cmd in (65, 11):
                    continue
                want = (id_ == 255 and cmd == 50)
                self.assertEqual(wrapped_tx_allowed(id_, cmd, UP, 14, WM_UP_ONCE), want, (id_, cmd))
                self.assertEqual(wrapped_tx_allowed(id_, cmd, DOWN, 14, WM_DOWN_RETURN), want, (id_, cmd))

    def test_permit_payload_mismatch_rejected(self):
        self.assertFalse(wrapped_tx_allowed(255, 50, UP, 14, WM_DOWN_RETURN))
        self.assertFalse(wrapped_tx_allowed(255, 50, DOWN, 14, WM_UP_ONCE))
        self.assertFalse(wrapped_tx_allowed(255, 50, UP, 14, WM_NONE))
        self.assertFalse(wrapped_tx_allowed(255, 50, UP, 13, WM_UP_ONCE))
        self.assertFalse(wrapped_tx_allowed(255, 50, None, 14, WM_UP_ONCE))

    def test_byte_flip_rejected(self):
        for i in range(14):
            bad = UP.copy(); bad[i] ^= 1
            self.assertFalse(wrapped_tx_allowed(255, 50, bad, 14, WM_UP_ONCE), i)
        for dz in (29, 31, -30, 0, 300, 5):
            bad = UP.copy(); bad[4] = dz & 0xFF; bad[5] = (dz >> 8) & 0xFF
            self.assertFalse(wrapped_tx_allowed(255, 50, bad, 14, WM_UP_ONCE), dz)

    def test_expected_up_wire_frame(self):
        self.assertEqual(wire_frame(0xFF, 50, UP).hex(),
                         'ffffff10320000' + '0000' + '1e00' + '000000000000' + 'b80b' + 'dd')

class WrappedMoveSourceShapeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = (SRC / 'NexArm_Wrapped_Move_Probe.ino').read_text(encoding='utf-8')
        cls.policy = (SRC / 'WrappedMovePolicy.h').read_text(encoding='utf-8')

    def test_single_uart_write_site_and_policy_gate(self):
        self.assertEqual(self.main.count('Serial1.write('), 1)
        self.assertIn('wrapped_tx_allowed(id,cmd,a,n,permit)', self.main)
        self.assertIn('wrapped_move_policy_test()', self.main)

    def test_no_raw_servo_frames_anywhere(self):
        for forbidden in ('write_pos_ex', 'set_torque', 'putBytes(', 'ESP.restart(',
                          'Serial.readString', 'write_frame(6,', 'write_frame(1,', 'write_frame(2,',
                          'write_frame(3,', 'write_frame(4,', 'write_frame(5,'):
            self.assertNotIn(forbidden, self.main)
        self.assertNotIn('write_pos_ex', self.policy)

    def test_one_shot_host_gating(self):
        self.assertIn('RUN_WRAPPED_ONCE', self.main)
        self.assertIn('if(!ready||phase!=IDLE||spent)return;', self.main)
        setup = self.main.split('void setup(){')[1].split('void loop()')[0]
        self.assertNotIn('start();', setup)

    def test_motion_payloads_are_fixed(self):
        self.assertIn('{0,0,0,0,30,0,0,0,0,0,0,0,0xB8,0x0B}', self.main)
        self.assertIn('{0,0,0,0,0xE2,0xFF,0,0,0,0,0,0,0xB8,0x0B}', self.main)

    def test_envelope_and_deadlines_present(self):
        for needle in ('motion_outside_envelope_cut_power', 'no_wrapped_motion', 'feedback_stale',
                       'key2_stop', 'test_deadline', 'baseline_unstable', 'STOP_WRAPPED'):
            self.assertIn(needle, self.main)

if __name__ == '__main__':
    unittest.main()
