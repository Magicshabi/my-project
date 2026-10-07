import unittest
from pathlib import Path
from test_teach_record_r6 import snapshot
from read_teach_record_r6 import decode_snapshot

ROOT=Path(__file__).parent
SRC=ROOT/'single_joint_probe_r1b_20261006/NexArm_Single_Joint_Probe'
class SingleJointBoundaryTests(unittest.TestCase):
    def test_record_import_remains_checked(self):
        f=snapshot(255); f['firmware']='nexarm-single-joint-probe-r1b'
        self.assertTrue(decode_snapshot(f)[0]['complete'])
        f['storage_fault']=True
        with self.assertRaises(ValueError): decode_snapshot(f)

    def test_one_motor_boundary_no_torque_or_store_writes(self):
        main=(SRC/'NexArm_Single_Joint_Probe.ino').read_text(encoding='utf-8')
        self.assertEqual(main.count('Serial1.write('),1)
        self.assertIn('if (!allowed_single_tx(',main)
        self.assertEqual(main.count('transmit(TEST_ID,3,'),1)
        self.assertIn('if (spent || phase != IDLE) return;',main)
        self.assertIn('if (id == TEST_ID) { spent = true;',main)
        self.assertIn('storage.begin("handle_free",true)',main)
        for forbidden in ('putBytes(', 'set_torque(', 'sync_write_pos', 'Serial.readString', 'ESP.restart('):
            self.assertNotIn(forbidden,main)
        setup=main.split('void setup() {')[1].split('void loop()')[0]
        self.assertNotIn('send_goal(',setup)
        self.assertNotIn('start();',setup)
        self.assertIn('if(click==1) start();',main)

if __name__=='__main__': unittest.main()
