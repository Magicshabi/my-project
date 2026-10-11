import copy
import json
import unittest
from unittest.mock import patch

import review_powered_route as m


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.route=json.loads(m.SOURCE.read_text(encoding='utf-8'))
        self.exports=[json.loads((m.EXPORTS/('point_%02d.json'%i)).read_text(encoding='utf-8')) for i in range(1,9)]
        self.limits=m.load_limits()

    def run_review(self):
        return m.review(self.route,self.exports,self.limits)

    def test_actual_route_preserves_targets_and_does_not_authorize_motion(self):
        original=copy.deepcopy(self.route)
        with patch('nexarm.transport.Transport.open_serial',side_effect=AssertionError('offline review opened serial')):
            report=self.run_review()
        self.assertEqual(self.route,original)
        self.assertTrue(report['data_integrity_passed'])
        self.assertFalse(report['motion_authorized'])
        self.assertFalse(report['collision_checked'])
        self.assertEqual(max(p['max_sample_spread'] for p in report['points']),1)
        self.assertEqual([s['to_point'] for s in report['segments'] if 'large_body_joint_change' in s['review_flags']],[4,5,8])

    def test_changed_export_rejected(self):
        self.exports[3]['encoder_positions'][3]+=1
        with self.assertRaisesRegex(m.SafetyError,'共享导出不一致'): self.run_review()

    def test_unstable_sample_rejected_even_with_complete_flag(self):
        self.route['points'][0]['samples'][0]['cmd11_counts'][0]+=6
        with self.assertRaisesRegex(m.SafetyError,'不稳定'): self.run_review()

    def test_channel_disagreement_rejected(self):
        self.route['points'][0]['samples'][0]['cmd11_counts'][0]+=7
        with self.assertRaisesRegex(m.SafetyError,'两通道'): self.run_review()

    def test_hidden_target_override_rejected(self):
        row=self.route['points'][2]
        row['target_encoder_positions']=row['encoder_positions'][:5]+[1999]
        with self.assertRaisesRegex(m.SafetyError,'隐藏目标'): self.run_review()

    def test_incomplete_or_unconfirmed_evidence_rejected(self):
        self.route['points'][3]['motor_driven_pose_confirmed_by_operator']=False
        with self.assertRaisesRegex(m.SafetyError,'主动到达'): self.run_review()

    def test_limits_binding_mismatch_rejected(self):
        self.route['host_joint_limits']['bounds'][0][0]+=1
        with self.assertRaisesRegex(m.SafetyError,'软件限位'): self.run_review()


if __name__=='__main__': unittest.main()
