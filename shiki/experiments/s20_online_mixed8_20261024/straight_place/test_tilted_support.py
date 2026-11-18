import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from tilted_support import solve, bottom_face, support_height

class TiltTest(unittest.TestCase):
    def setUp(self):
        self.tool=R.from_matrix([[0,0,1],[1,0,0],[0,1,0]])
        self.supports=[dict(id='middle',pos=[.155,.5,.78],dims=[.53,.34,.21]),
                       dict(id='left',pos=[.22,1.02,.75],dims=[.4,.38,.22])]
    def test_two_distinct_height_contacts(self):
        v=solve(np.array([.54,.35,.23]),self.tool,R.identity(),R.identity(),(.42,1.02),self.supports)
        self.assertIsNotNone(v)
        self.assertLess(v['roll_rad'],0)
        self.assertLess(abs(v['roll_rad']),np.radians(6))
        self.assertLess(v['contact_height_residual_m'],1e-8)
        poly=bottom_face([.54,.35,.23],R.from_quat(v['tool_xyzw']),(.42,1.02))
        for support in self.supports:
            self.assertAlmostEqual(v['center'][2],support_height(poly,support),places=8)
        self.assertAlmostEqual(v['support_height_difference_m'],.02)
    def test_missing_second_support_rejected(self):
        v=solve(np.array([.54,.35,.23]),self.tool,R.identity(),R.identity(),(.42,.3),self.supports)
        self.assertIsNone(v)

if __name__=='__main__':unittest.main()
