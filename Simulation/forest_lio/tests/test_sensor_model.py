#!/usr/bin/env python3
"""Check sensor-model invariants that materially affect LIO."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from common import POINT_DTYPE, custom_points, trajectory_comparison
from generate_mid360_bag import trajectory, trajectory_pose_samples, pack_scan
from renderer import PointRenderer

class SensorModelTests(unittest.TestCase):
    def test_motion_derivatives_and_body_gyro(self):
        h = 1e-3
        for t in [3.1, 8, 15, 25, 29.9]:
            p, v, a, r, w = trajectory(t, 30, 3, 0.5)
            prev = trajectory(t-h, 30, 3, 0.5)
            nxt = trajectory(t+h, 30, 3, 0.5)
            np.testing.assert_allclose((nxt[0]-prev[0])/(2*h), v, atol=1e-7)
            np.testing.assert_allclose((nxt[1]-prev[1])/(2*h), a, atol=1e-7)
            numeric_w = Rotation.from_matrix(prev[3].T@nxt[3]).as_rotvec()/(2*h)
            np.testing.assert_allclose(numeric_w, w, atol=1e-7)

    def test_per_point_analytic_pose_matches_scalar(self):
        times=np.linspace(0,31,333)
        positions,rotations=trajectory_pose_samples(times,30,3,.5)
        for i,t in enumerate(times):
            expected=trajectory(t,30,3,.5)
            np.testing.assert_allclose(positions[i],expected[0],atol=1e-12)
            np.testing.assert_allclose(rotations[i],expected[3],atol=1e-12)

    def test_rest_specific_force(self):
        for t in [0, 2, 3, 30, 31]:
            p, v, a, r, w = trajectory(t, 30, 3, 0.5)
            np.testing.assert_allclose(v, 0, atol=1e-12)
            np.testing.assert_allclose(a, 0, atol=1e-12)
            np.testing.assert_allclose(r.T@(a-[0, 0, -9.81])/9.81, [0, 0, 1], atol=1e-12)

    def test_custommsg_preserves_point_times(self):
        pts = np.zeros(4, dtype=POINT_DTYPE)
        pts["xyz"] = [[1, 2, 3], [4, 5, 6], [0, 0, 0], [7, 8, 9]]
        pts["offset_time"] = [0, 23000, 10000000, 99999999]
        pts["line"] = [0, 1, 2, 3]
        msg = pack_scan(17, 1700000000.125, pts)
        import io
        buf = io.BytesIO(); msg.serialize(buf)
        seq, stamp, frame, tb, count, _, decoded = custom_points(buf.getvalue())
        self.assertEqual(seq, 17); self.assertEqual(count, 4)
        self.assertEqual(tb, 1700000000125000000)
        self.assertEqual(frame, "livox_frame")
        np.testing.assert_array_equal(decoded, pts)

    def test_occlusion_and_sensor_translation(self):
        y, z = np.meshgrid(np.linspace(-1, 1, 21), np.linspace(-1, 1, 21))
        front = np.column_stack([np.full(y.size, 3.), y.ravel(), z.ravel()])
        back = np.column_stack([np.full(y.size, 6.), y.ravel(), z.ravel()])
        renderer = PointRenderer(np.vstack([front, back]), 0.5, 0.2, 10)
        rays = np.array([[1., 0, 0], [-1., 0, 0]])
        np.testing.assert_allclose(renderer.scan(rays, np.zeros(3), np.eye(3), 0.5), [3, 0], atol=1e-8)
        np.testing.assert_allclose(renderer.scan(rays, np.array([1., 0, 0]), np.eye(3), 0.5), [2, 0], atol=1e-8)

    def test_cached_scene_translation(self):
        y,z=np.meshgrid(np.linspace(-1,1,21),np.linspace(-1,1,21))
        points=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()])
        renderer=PointRenderer(points,.5,.2,10)
        scene=renderer.prepare(np.zeros(3),np.eye(3))
        # Actual ray origin advances within a scan; range must change without rebuilding raster.
        rays=np.array([[1.,0,0]])
        for x in [0.,.01,.02]:
            result=renderer.cast(rays,scene,np.array([x,0,0]),np.eye(3),.5)
            np.testing.assert_allclose(result,[3-x],atol=1e-8)

    def test_tracker_reaches_goal_with_acceleration_limit(self):
        from run_closed_loop import tracking_acceleration
        p=np.zeros(3); v=np.zeros(3); goal=np.array([-.7,0,.5])
        for _ in range(2400):
            a=tracking_acceleration(p,v,goal,np.zeros(3),np.zeros(3))
            self.assertLessEqual(np.linalg.norm(a),1.5+1e-12)
            p+=v*.005+.5*a*.005**2
            v+=a*.005
        np.testing.assert_allclose(p,goal,atol=.0001)

    def test_comparison_orientation_without_alignment(self):
        ref = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [0.1, 0, 0, 0, 0, 0, 0, 1], [0.2, 0, 0, 0, 0, 0, 0, 1]])
        est = ref.copy()
        est[:, 4:8] = Rotation.from_euler("z", 90, degrees=True).as_quat()
        result = trajectory_comparison(est, ref)
        self.assertAlmostEqual(result["orientation_rmse_deg"], 90)

    def test_comparison_rejects_dropout_interpolation(self):
        ref = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [1, 10, 0, 0, 0, 0, 0, 1]])
        result = trajectory_comparison(ref, ref, max_gap=0.2)
        self.assertEqual(result["status"], "no_timestamp_overlap")

if __name__ == "__main__":
    unittest.main()
