import struct
import unittest
from capture_joint_diagnostics import take_frames, describe_feedback


def packet(cmd, payload):
    body = bytes([255, len(payload) + 2, cmd]) + payload
    return b"\xff\xff" + body + bytes([(~sum(body)) & 255])


class FeedbackTests(unittest.TestCase):
    def test_actual_pairs_do_not_mix_angles_with_positions(self):
        data = struct.pack("<12h", 0, -900, 511, 111, 1024, 222, 2048, 333, 3000, 444, 4095, 555)
        result = describe_feedback(65, data)
        self.assertEqual(result["pulses"], [0, 511, 1024, 2048, 3000, 4095])
        self.assertEqual(result["angles_deg"], [-90, 11.1, 22.2, 33.3, 44.4, 55.5])

    def test_same_bytes_have_distinct_status_layout(self):
        data = struct.pack("<12h", *range(12))
        self.assertEqual(describe_feedback(11, data)["pulses"], list(range(6, 12)))
        self.assertEqual(describe_feedback(65, data)["pulses"], list(range(0, 12, 2)))

    def test_fragmented_noise_and_corrupt_frame(self):
        good = packet(65, struct.pack("<12h", *range(12)))
        bad = good[:-1] + bytes([good[-1] ^ 1])
        buf = bytearray(b"logs\r\n" + bad + good[:10])
        self.assertEqual(take_frames(buf), [])
        buf.extend(good[10:])
        self.assertEqual(take_frames(buf), [(255, 65, struct.pack("<12h", *range(12)))])

    def test_invalid_payloads(self):
        self.assertIsNone(describe_feedback(65, b"\0" * 12))
        self.assertIsNone(describe_feedback(65, b"\0" * 25))
        self.assertIsNone(describe_feedback(96, b"\0" * 24))
        bad = struct.pack("<12h", -1, 0, 2048, 0, 2048, 0, 2048, 0, 2048, 0, 2048, 0)
        self.assertFalse(describe_feedback(65, bad)["range_ok"])


if __name__ == "__main__":
    unittest.main()
