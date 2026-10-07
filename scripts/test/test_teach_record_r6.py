import json
from pathlib import Path
import struct
import unittest
import zlib

from read_teach_record_r6 import decode_snapshot, latest_snapshot

ROOT = Path(__file__).resolve().parent
SKETCH = ROOT / 'handle_teach_record_20260923' / 'NexArm_Teach_Record'


def snapshot(mask=3):
    values = [1000 + slot * 50 + joint for slot in range(8) for joint in range(6)]
    body = struct.pack('<IHBB48h', 0x484E4431, 1, mask, 0, *values)
    payload = body + struct.pack('<I', zlib.crc32(body))
    return dict(type='teach_record', firmware='nexarm-teach-record-r6',
                namespace='handle_free', storage_fault=False, valid_mask=mask,
                record_hex=payload.hex(), playback_enabled=False)


class RecordTests(unittest.TestCase):
    def test_roundtrip_six_joint_mapping_and_unvalidated_path(self):
        r, payload = decode_snapshot(snapshot())
        self.assertEqual(len(payload), 108)
        self.assertEqual(r['poses'][1]['encoder_positions'], [1050, 1051, 1052, 1053, 1054, 1055])
        self.assertFalse(r['complete'])
        self.assertFalse(r['playback_validated'])
        self.assertFalse(r['physical_path_validated'])

    def test_empty_and_complete(self):
        self.assertEqual(decode_snapshot(snapshot(0))[0]['poses'], [])
        self.assertTrue(decode_snapshot(snapshot(255))[0]['complete'])

    def test_wrong_origin_rejected(self):
        for field, value in [('firmware', 'r4'), ('namespace', 'handle_teach')]:
            frame = snapshot(); frame[field] = value
            with self.assertRaises(ValueError): decode_snapshot(frame)

    def test_known_key_diagnostic_version(self):
        frame = snapshot()
        for version in ('nexarm-teach-record-r6k1', 'nexarm-teach-record-r7', 'nexarm-teach-record-r7k2'):
            frame['firmware'] = version
            self.assertEqual(decode_snapshot(frame)[0]['valid_mask'], 3)
        frame['firmware'] = 'nexarm-teach-record-unknown'
        with self.assertRaises(ValueError): decode_snapshot(frame)

    def test_corruption_and_metadata_mismatch(self):
        frame = snapshot(); frame['record_hex'] = frame['record_hex'][:-2] + '00'
        with self.assertRaises(ValueError): decode_snapshot(frame)
        frame = snapshot(); frame['valid_mask'] = 255
        with self.assertRaises(ValueError): decode_snapshot(frame)

    def test_out_of_order_record_rejected(self):
        with self.assertRaises(ValueError): decode_snapshot(snapshot(5))

    def test_uncertain_storage_never_falls_back(self):
        good = snapshot(); bad = snapshot(); bad['storage_fault'] = True
        data = (json.dumps(good) + '\n' + json.dumps(bad)).encode()
        with self.assertRaises(ValueError): latest_snapshot(data)

    def test_mixed_log_and_truncated_tail(self):
        data = b'\xffboot noise\n[TeachRecord] READY\n' + json.dumps(snapshot()).encode() + b'\n{"type":'
        self.assertEqual(latest_snapshot(data)[0]['valid_mask'], 3)

    def test_no_snapshot_is_not_success(self):
        with self.assertRaises(ValueError): latest_snapshot(b'{"type":"actual65"}\n')

    def test_sketch_has_single_read_only_motor_transmit_site(self):
        # Guard the actual hardware boundary; other sources in this standalone
        # sketch only parse frames or implement pure record/capture logic.
        for sketch in (SKETCH, ROOT / 'handle_teach_keydiag_20261004' / 'NexArm_Teach_Record',
                       ROOT / 'handle_teach_r7_20261004' / 'NexArm_Teach_Record',
                       ROOT / 'handle_teach_r7k2_20261004' / 'NexArm_Teach_Record'):
            with self.subTest(sketch=sketch):
                sources = '\n'.join(p.read_text(encoding='utf-8') for p in sketch.iterdir()
                                    if p.suffix in ('.ino', '.cpp', '.h'))
                self.assertEqual(sources.count('Serial1.write('), 1)
                self.assertEqual(sources.count('at32_tx.tx_packet_complete('), 1)
                self.assertIn('at32_tx.tx_packet_complete(0xFF, 65, nullptr, 0)', sources)
                self.assertNotIn('set_torque(', sources)
                self.assertNotIn('sync_write_pos', sources)
                self.assertNotIn('Robot_Arm.h', sources)
                self.assertNotIn('AT32_OTA.h', sources)


if __name__ == '__main__':
    unittest.main()
