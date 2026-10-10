import copy
import json
from pathlib import Path
import unittest

import joint_limits as m


class JointLimitTests(unittest.TestCase):
    def setUp(self):
        self.limits=m.load_limits()

    def test_all_axis_boundaries_and_one_count_outside(self):
        for sid,(lo,hi) in enumerate(self.limits,1):
            for allowed in (lo,hi):
                m.check_axis(sid,allowed,self.limits)
            for rejected in (lo-1,hi+1,True,float(lo),'2000'):
                with self.subTest(sid=sid,value=rejected),self.assertRaises(m.SafetyError):
                    m.check_axis(sid,rejected,self.limits)

    def test_invalid_or_misordered_limit_config_fails_closed(self):
        original=m.snapshot(self.limits)
        candidates=[]
        for field,value in (('units','degrees'),('servo_ids',[2,1,3,4,5,6]),('bounds',[[0,4095]])):
            changed=copy.deepcopy(original);changed[field]=value;candidates.append(changed)
        for pair in ([2000,1000],[0,4096],[True,1000],[0.0,1000]):
            changed=copy.deepcopy(original);changed['bounds'][0]=pair;candidates.append(changed)
        for document in candidates:
            with self.assertRaises(m.SafetyError):m.validate_limits(document)

    def test_real_old_route_audit_preserves_bytes_and_reports_all_six_points(self):
        path=m.HERE/'data/eight_joints_20261009.json'
        before=path.read_bytes()
        result=m.audit_route(json.loads(before),self.limits)
        self.assertEqual([p['point'] for p in result['points'] if p['violations']],[1,2,3,4,5,8])
        self.assertEqual({v['servo_id'] for p in result['points'] for v in p['violations']},{4})
        self.assertEqual(result['points'][2]['target'][5],2029)
        self.assertFalse(result['all_targets_within_limits'])
        self.assertFalse(result['hardware_test_performed'])
        self.assertEqual(path.read_bytes(),before)

    def test_empty_new_route_is_not_reported_as_success(self):
        route=dict(schema='nexarm.joint-eight/1',units='raw_encoder_counts',
                   servo_ids=list(range(1,7)),complete=False,points=[])
        result=m.audit_route(route,self.limits)
        self.assertEqual(result['captured_points'],0)
        self.assertFalse(result['all_targets_within_limits'])
        self.assertFalse(result['eight_points_recorded'])


if __name__=='__main__':unittest.main()
