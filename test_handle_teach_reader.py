"""Offline protocol tests; never imports pyserial or opens a port."""
import struct
import unittest
import zlib
from read_handle_teach import decode_record, extract_record


def record(mask=255, version=1, position=2048):
    body = struct.pack("<IHBB48h", 0x484E4431, version, mask, 0, *([position] * 48))
    return body + struct.pack("<I", zlib.crc32(body))


def frame(payload):
    body = bytes([255, len(payload) + 2, 105]) + payload
    return b"\xff\xff" + body + bytes([(~sum(body)) & 255])


class TeachReadTests(unittest.TestCase):
    def test_all_sequential_prefixes(self):
        for count in range(9):
            result = decode_record(record((1 << count) - 1))
            self.assertEqual(len(result["poses"]), count)
            self.assertEqual(result["complete"], count == 8)
            self.assertFalse(result["playback_validated"])

    def test_noise_and_fragmented_packet(self):
        payload = record()
        packet = frame(payload)
        noise = b"[HandleTeach] saved\r\n\xff\xff\x01garbage"
        for split in range(len(packet)):
            self.assertIsNone(extract_record(noise + packet[:split]))
        self.assertEqual(extract_record(noise + packet), payload)

    def test_bad_frame_followed_by_good_frame(self):
        payload = record()
        good = frame(payload)
        bad = good[:-1] + bytes([good[-1] ^ 1])
        self.assertIsNone(extract_record(bad))
        self.assertEqual(extract_record(bad + b"log\n" + good), payload)

    def test_corrupt_record_inside_valid_frame(self):
        bad = bytearray(record())
        bad[12] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            decode_record(extract_record(frame(bad)))

    def test_valid_crc_does_not_bypass_semantics(self):
        for data in [record(mask=5), record(version=2), record(position=-1), record(position=4096)]:
            with self.assertRaises(ValueError):
                decode_record(data)

    def test_invalid_length(self):
        for data in [b"", record()[:-1], record() + b"\0"]:
            with self.assertRaises(ValueError):
                decode_record(data)


if __name__ == "__main__":
    unittest.main()
