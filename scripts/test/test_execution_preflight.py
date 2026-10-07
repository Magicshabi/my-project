import unittest
from pathlib import Path
from test_teach_record_r6 import snapshot
from read_teach_record_r6 import decode_snapshot

ROOT = Path(__file__).parent
SRC = ROOT / 'execution_preflight_20261005' / 'NexArm_Execution_Preflight'

class ExecutionPreflightTests(unittest.TestCase):
    def test_preserves_record_validation(self):
        f = snapshot(255)
        f['firmware'] = 'nexarm-execution-preflight-r1'
        r, b = decode_snapshot(f)
        self.assertTrue(r['complete'])
        self.assertEqual(len(b), 108)
        f['storage_fault'] = True
        with self.assertRaises(ValueError): decode_snapshot(f)

    def test_hardware_boundary_stays_read_only(self):
        source = '\n'.join(p.read_text(encoding='utf-8') for p in SRC.iterdir() if p.suffix in ('.ino','.cpp','.h'))
        self.assertEqual(source.count('Serial1.write('), 1)
        self.assertIn('if (!allowed_probe_tx(id, cmd, args, n))', source)
        self.assertIn('storage.begin("handle_free", true)', source)
        for forbidden in ('putBytes(', 'putInt(', 'putBool(', 'set_torque(', 'write_pos_ex(', 'sync_write_pos', 'ESP.restart('):
            self.assertNotIn(forbidden, source)

    def test_no_host_command_sender(self):
        source = (ROOT / 'capture_execution_preflight.py').read_text(encoding='utf-8')
        self.assertNotIn('port.write(', source)

if __name__ == '__main__': unittest.main()
