"""Run with the project's Python; tests do not initialize Newton."""
import unittest
import itertools
import numpy as np
from scipy.spatial.transform import Rotation as R
from flip_geometry import horizontal_tool, rotated_extent, sample_metrics, constraint_error


class FlipGeometryTest(unittest.TestCase):
    def test_horizontal_normal_with_tilted_container_and_offset_grasp(self):
        container=R.from_euler('xyz',[-.0045,-.0314,-3.1316])
        for roll in (0.,.1,-.1):
            attachment=R.from_euler('xyz',[np.pi+roll,.02,.3])
            tool=horizontal_tool(container,attachment)
            normal=tool.apply([0,0,1])
            inward=container.apply([1,0,0]);inward[2]=0;inward/=np.linalg.norm(inward)
            np.testing.assert_allclose(normal,inward,atol=1e-12)
            self.assertAlmostEqual(float(normal[2]),0.,places=12)

    def test_rotated_extent_contains_all_eight_corners(self):
        dims=np.array([.55,.4,.25])
        container=R.from_euler('xyz',[.05,-.03,1.2])
        orientation=R.from_euler('xyz',[.2,-np.pi/2,.4])
        extent=rotated_extent(dims,orientation,container)
        corners=np.array(list(itertools.product((-1,1),repeat=3)))*dims/2
        points=(container.inv()*orientation).apply(corners)
        np.testing.assert_allclose(points.max(axis=0)-points.min(axis=0),extent)

    def test_constraints_reject_tilt_and_inside_rotation(self):
        self.assertIsNotNone(constraint_error({'horizontal_error_deg':2.}))
        self.assertIsNotNone(constraint_error({'outside_clearance_m':-.01}))
        self.assertIsNotNone(constraint_error({'inward_alignment':-1.}))
        self.assertIsNone(constraint_error({'horizontal_error_deg':.1,'outside_clearance_m':.05,'inward_alignment':1.}))

    def test_width_vertical_target(self):
        attachment=R.from_euler('x',np.pi)
        tool=horizontal_tool(R.identity(),attachment,width_vertical=True)
        box=tool*attachment
        self.assertAlmostEqual(abs(box.apply([0,1,0])[2]),1.)
        self.assertAlmostEqual(box.apply([1,0,0])[2],0.)
        np.testing.assert_allclose(rotated_extent([.55,.4,.25],box,R.identity()),[.25,.55,.4],atol=1e-12)

    def test_flat_case_quarter_turn_and_rigid_attachment(self):
        attachment=R.from_euler('x',np.pi)
        tool=horizontal_tool(R.identity(),attachment)
        np.testing.assert_allclose(rotated_extent([.55,.4,.25],tool*attachment,R.identity()),[.25,.4,.55],atol=1e-12)
        offset=np.array([.02,-.01,.125])
        box_center=np.array([-.8,.3,1.])
        tcp=box_center-tool.apply(offset)
        np.testing.assert_allclose(tcp+tool.apply(offset),box_center)
        # Separating against tool +Z moves away from the box, not upward.
        delta=-tool.apply([0,0,.12])
        self.assertAlmostEqual(delta[2],0.,places=12)
        self.assertLess(delta@tool.apply([0,0,1]),0.)

if __name__=='__main__':unittest.main()
