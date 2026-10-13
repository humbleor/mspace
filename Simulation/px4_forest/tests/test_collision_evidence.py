import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from collision_evidence import collision_acceptance, ego_acceptance


class CollisionEvidence(unittest.TestCase):
    def assess(self, points, rows, **kwargs):
        return collision_acceptance(points, rows, body_radius=.3, obstacle_radius=.2,
                                    max_gap=1, **kwargs)

    def test_segment_crossing_detected_with_clear_endpoints(self):
        r = self.assess([[0, 0, 0]], [[0, -2, 0, 0], [1, 2, 0, 0]])
        self.assertEqual(r["status"], "collision")
        self.assertFalse(r["passed"])
        self.assertAlmostEqual(r["first_collision"]["sim_time_s"], .375)
        self.assertAlmostEqual(r["minimum_clearance_m"], -.5)

    def test_clear_bypass(self):
        r = self.assess([[0, 0, 0]], [[0, -2, 1, 0], [1, 2, 1, 0]])
        self.assertTrue(r["passed"])
        self.assertAlmostEqual(r["minimum_clearance_m"], .5)

    def test_tangent_is_collision(self):
        r = self.assess([[0, 0, 0]], [[0, -2, .5, 0], [1, 2, .5, 0]])
        self.assertEqual(r["status"], "collision")
        self.assertAlmostEqual(r["first_collision"]["sim_time_s"], .5)

    def test_rotor_envelope_not_only_centerline(self):
        r = self.assess([[0, .45, 0]], [[0, -1, 0, 0], [1, 1, 0, 0]])
        self.assertEqual(r["status"], "collision")

    def test_initial_overlap_and_stationary_pose(self):
        r = self.assess([[0, .1, 0]], [[0, 0, 0, 0], [1, 0, 0, 0]])
        self.assertEqual(r["first_collision"]["sim_time_s"], 0)
        self.assertAlmostEqual(r["minimum_clearance_m"], -.4)

    def test_first_contact_is_earliest_not_nearest_point(self):
        r = self.assess([[1, 0, 0], [-1, .4, 0]], [[0, -2, 0, 0], [1, 2, 0, 0]])
        np.testing.assert_allclose(r["first_collision"]["obstacle_point"], [-1, .4, 0])
        self.assertAlmostEqual(r["first_collision"]["sim_time_s"], .175)

    def test_gaps_or_missing_invalid_truth_never_pass(self):
        cases = [[], [[0, 2, 0, 0]], [[0, 2, 0, 0], [2, 2, 0, 0]],
                 [[1, 2, 0, 0], [0, 2, 0, 0]], [[0, 2, 0, 0], [0, 2, 0, 0]],
                 [[0, 2, 0, 0], [1, np.nan, 0, 0]]]
        for rows in cases:
            with self.subTest(rows=rows):
                self.assertFalse(self.assess([[0, 0, 0]], rows)["passed"])

    def test_invalid_geometry_and_parameters_rejected(self):
        for points in ([], [[np.nan, 0, 0]]):
            with self.assertRaises(ValueError):
                self.assess(points, [[0, 1, 0, 0], [1, 1, 0, 0]])
        for radius in (0, -1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                collision_acceptance([[0, 0, 0]], [[0, 1, 0, 0], [1, 1, 0, 0]], body_radius=radius)

    def test_matches_exhaustive_segment_distance(self):
        rng = np.random.RandomState(7)
        points = rng.normal(size=(80, 3))
        positions = rng.normal(size=(25, 3))*2
        rows = np.column_stack([np.arange(25), positions])
        brute = []
        for a, b in zip(positions[:-1], positions[1:]):
            d = b-a
            t = np.clip((points-a)@d/(d@d), 0, 1)
            brute.append(np.linalg.norm(points-a-t[:, None]*d, axis=1).min()-.5)
        r = self.assess(points, rows)
        self.assertAlmostEqual(r["minimum_clearance_m"], min(brute))
        self.assertEqual(r["colliding_segments"], sum(d <= 0 for d in brute))
        self.assertEqual(r["checked_segments"], 24)

    def test_multiple_batches_keep_every_segment(self):
        rows = np.column_stack([np.arange(2051)*.01, np.linspace(-2, 2, 2051),
                                np.ones(2051), np.zeros(2051)])
        r = self.assess([[0, 0, 0]], rows)
        self.assertTrue(r["passed"])
        self.assertEqual(r["checked_segments"], 2050)
        self.assertAlmostEqual(r["minimum_clearance_m"], .5)

    def test_ego_uses_recorded_inflation_without_extra_radius(self):
        rows = [[0, -1, .3, 0], [.01, 1, .3, 0]]
        r = ego_acceptance([[0,0,0]], rows, {"obstacles_inflation":.25,"resolution":.1})
        self.assertTrue(r["passed"])
        self.assertAlmostEqual(r["minimum_clearance_m"], .05)
        self.assertEqual(r["obstacle_radius_m"], 0)
        self.assertEqual(r["ego_cloud_inflation_steps_xy"], 3)
        r = ego_acceptance([[0,0,0]], rows, {"obstacles_inflation":.4,"resolution":.1})
        self.assertFalse(r["passed"])

    def test_ego_missing_or_invalid_parameters_do_not_fall_back(self):
        for params in ({}, {"obstacles_inflation":.25,"resolution":0},
                       {"obstacles_inflation":float("nan"),"resolution":.1}):
            with self.assertRaises((KeyError, ValueError)):
                ego_acceptance([[0,0,0]], [[0,1,0,0],[.01,1,0,0]], params)

    def test_intervals_group_adjacent_segments_not_contact_count(self):
        rows = [[0,-1,0,0],[.1,0,0,0],[.2,1,0,0],[.3,2,0,0],[.4,0,0,0]]
        r = self.assess([[0,0,0]], rows)
        self.assertEqual(len(r["intersection_intervals"]), 2)
        self.assertAlmostEqual(r["minimum_clearance_location"]["body_center"][0],0)


if __name__ == "__main__":
    unittest.main()
