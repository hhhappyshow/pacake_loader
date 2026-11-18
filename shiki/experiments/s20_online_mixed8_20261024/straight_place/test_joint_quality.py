import unittest
import numpy as np
from joint_quality import assess, retime, check_margin
from stacked_scene import twelve_box_fixture


class Array:
    def __init__(self,x):self.x=np.asarray(x)
    def numpy(self):return self.x


class Model:
    joint_limit_lower=Array([-np.pi]*6)
    joint_limit_upper=Array([np.pi]*6)


class QualityTest(unittest.TestCase):
    def test_retiming_limits_and_endpoints(self):
        q=np.array([[0.]*6,[.1,.2,.3,.4,.5,.6],[.3,.4,.5,.6,.7,.8]])
        policy=dict(margin_deg=15.,speed_deg_s=25.,acceleration_deg_s2=50.)
        path=retime(q,.04,policy)
        np.testing.assert_allclose(path[[0,-1]],q[[0,-1]])
        result=assess(path,[-np.pi]*6,[np.pi]*6)
        self.assertLessEqual(max(result['max_speed_deg_s']),25.0001)
        self.assertLessEqual(max(result['max_acceleration_deg_s2']),50.0001)

    def test_limits_not_hidden_by_wrapping(self):
        with self.assertRaisesRegex(ValueError,'joint_margin'):
            check_margin([[2*np.pi,0,0,0,0,0]],Model(),dict(margin_deg=15.))

    def test_fixture_containment(self):
        boxes=twelve_box_fixture()
        self.assertEqual(sorted(b['inventory_id'] for b in boxes),list(range(1,13)))
        for column in (1,2,3):
            chain=sorted([b for b in boxes if b['column']==column],key=lambda b:b['level'])
            for lower,upper in zip(chain,chain[1:]):
                self.assertAlmostEqual(lower['pos'][2]+lower['dims'][2],upper['pos'][2])
                for axis in (0,1):
                    self.assertGreaterEqual(upper['pos'][axis]+1e-9,lower['pos'][axis])
                    self.assertLessEqual(upper['pos'][axis]+upper['dims'][axis],lower['pos'][axis]+lower['dims'][axis]+1e-9)
            self.assertAlmostEqual(chain[-1]['pos'][2]+chain[-1]['dims'][2],.97 if column==3 else .99)


if __name__=='__main__':unittest.main()
