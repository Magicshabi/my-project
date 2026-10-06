import ast
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from contextlib import contextmanager
from uuid import uuid4

import teach_validation_r7 as v
from test_teach_record_r6 import snapshot


@contextmanager
def test_directory():
    # Python 3.13 TemporaryDirectory mode=0700 can exclude the sandbox SID on
    # this Windows host. Inherit workspace ACLs and retain tiny test artifacts.
    root = Path(__file__).resolve().parent / '.test-output-r7' / uuid4().hex
    root.mkdir(parents=True)
    yield root


def phases(axis=4, first=24, second=24, coupled=False):
    result = {}
    ms = 1000
    base = [1800, 2100, 1900, 2200, 2000, 1400]
    for name, delta in [('before', 0), ('first', first), ('second', first+second), ('returned', 0)]:
        values = list(base); values[axis-1] += delta
        if coupled and delta:
            values[0 if axis != 1 else 1] += 40
        result[name] = []
        for _ in range(6):
            result[name].append(dict(type='actual65', ms=ms, pulses=list(values)))
            ms += 250
    return result


class AxisTests(unittest.TestCase):
    def test_six_axes_both_encoder_directions(self):
        for axis in range(1, 7):
            for direction in (-1, 1):
                with self.subTest(axis=axis, direction=direction):
                    self.assertTrue(v.analyze_axis(axis, phases(axis, direction*24, direction*24))['passed'])

    def test_cached_values_fail(self):
        self.assertFalse(v.analyze_axis(4, phases(first=0, second=0))['passed'])

    def test_inconsistent_direction_fails(self):
        self.assertFalse(v.analyze_axis(4, phases(first=24, second=-24))['passed'])

    def test_other_joint_movement_needs_review(self):
        self.assertFalse(v.analyze_axis(4, phases(coupled=True))['passed'])

    def test_single_sample_spike_not_hidden_by_median(self):
        data = phases(); data['first'][1]['pulses'][3] += 900
        with self.assertRaises(ValueError): v.analyze_axis(4, data)

    def test_boundary_jump_not_hidden_between_phases(self):
        data = phases(first=200, second=200)
        with self.assertRaises(ValueError): v.analyze_axis(4, data)

    def test_reset_or_duplicate_time_rejected(self):
        for timestamp in (0, 1000):
            data = phases()['before']; data[1]['ms'] = timestamp
            with self.assertRaises(ValueError): v.summarize_positions(data)

    def test_unstable_endpoint_and_invalid_fields(self):
        data = phases()['before']; data[-1]['pulses'][5] += 9
        with self.assertRaises(ValueError): v.summarize_positions(data)
        for value in (-1, 4096, True):
            data = phases()['before']; data[-1]['pulses'][0] = value
            with self.assertRaises(ValueError): v.summarize_positions(data)

    def test_too_few_frames_and_time_gap(self):
        with self.assertRaises(ValueError): v.summarize_positions(phases()['before'][:4])
        data = phases()['before']; data[-1]['ms'] += 2000
        with self.assertRaises(ValueError): v.summarize_positions(data)

    def test_millis_wrap_is_not_reset(self):
        data = phases()['before']
        for i, frame in enumerate(data): frame['ms'] = (0xFFFFFF00 + i*250) & 0xFFFFFFFF
        self.assertEqual(v.summarize_positions(data)['count'], 6)


class EvidenceTests(unittest.TestCase):
    def test_newest_fault_not_replaced_by_old_success(self):
        good = snapshot(); bad = snapshot(); bad['storage_fault'] = True
        with self.assertRaises(ValueError): v.select_snapshot([good, bad])

    def test_immutable_archive_unique_directory_and_crc(self):
        with test_directory() as root:
            path = Path(root) / 'one'
            session = v.Session(path)
            frame = snapshot(255); frame['firmware'] = v.FIRMWARE
            decoded, binary = session.archive_record([frame], 'eight-points')
            self.assertEqual((path / 'eight-points.bin').read_bytes(), binary)
            self.assertEqual(decoded['valid_mask'], 255)
            self.assertFalse(decoded['playback_validated'])
            self.assertTrue((path / 'events.jsonl').exists())
            with self.assertRaises(FileExistsError): v.Session(path)

    def test_operator_stop_cannot_pass(self):
        with test_directory() as root:
            session = v.Session(Path(root) / 'stop')
            with patch('builtins.input', return_value='STOP'):
                with self.assertRaises(RuntimeError): v.ask(session, 'observed motion')
            self.assertIn('STOP', (session.directory / 'events.jsonl').read_text(encoding='utf-8'))

    def test_reader_has_no_write_or_reset_calls(self):
        source = Path(v.__file__).read_text(encoding='utf-8')
        tree = ast.parse(source)
        forbidden = {'write', 'writelines', 'reset_input_buffer', 'reset_output_buffer', 'send_break'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                owner = ast.unparse(node.func.value)
                if owner == 'self.port': self.assertNotIn(node.func.attr, forbidden)


if __name__ == '__main__':
    unittest.main()
