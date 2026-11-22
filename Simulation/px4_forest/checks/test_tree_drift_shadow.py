import sys
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Simulation/px4_forest/scripts"))
from module_launch import prepare_modules
from tree_drift_shadow import (
    Prediction, Registration, apply_planar, inverse_planar, score_predictions,
)


class TreeDriftShadowTests(unittest.TestCase):
    def test_initial_placement_and_injected_drift_are_composed_once(self):
        local = np.array([[0.0, 0.0, 3.0], [2.0, 4.0, 5.0]])
        world = apply_planar(local, (0, 5, 0))
        np.testing.assert_allclose(world[:, 1], [5, 9])
        drift = (0.4, 0.5, np.deg2rad(10))
        reported = apply_planar(world, drift)
        np.testing.assert_allclose(apply_planar(reported, inverse_planar(drift)), world,
                                   atol=1e-12)

    def test_scores_paired_predictions_and_reports_unavailable_registration(self):
        points = np.tile([1.5, 0, 2], (4, 1))
        knots = (0, 0, 0, 0, 1, 1, 1, 1)
        registration = Registration(10.0, (-0.6, 0, 0), 6, 0.01)
        predictions = [
            Prediction(1, 2, 10.0, 10.0, 1, knots, points, registration),
            Prediction(1, 2, 11.0, 11.0, 2, knots, points, None),
            Prediction(2, 1, 10.0, 10.0, 1, knots, np.tile([0, 0, 2], (4, 1)),
                       Registration(10.0, (0.6, 0, 0), 6, 0.01)),
        ]
        times = np.arange(10.0, 12.02, 0.02)
        truth = {
            1: [[t, 0, 0, 2] for t in times],
            2: [[t, 0.9, 0, 2] for t in times],
        }
        result, rows = score_predictions(predictions, truth, {2: (0.6, 0, 0)}, 1.2, 0.05)
        view = result["observers"]["1"]
        self.assertEqual(view["received_trajectories"], 2)
        self.assertEqual(view["registration_coverage"], 0.5)
        self.assertAlmostEqual(view["paired_raw_error_m"]["rmse"], 0.6)
        self.assertAlmostEqual(view["paired_corrected_error_m"]["rmse"], 0.0)
        self.assertGreater(view["raw_missed_danger_samples"], 0)
        self.assertEqual(view["corrected_missed_danger_samples"], 0)
        self.assertTrue(any(row["corrected_error_m"] is None for row in rows))
        reverse_view = result["observers"]["2"]
        self.assertAlmostEqual(reverse_view["paired_raw_error_m"]["rmse"], 0.6)
        self.assertAlmostEqual(reverse_view["paired_corrected_error_m"]["rmse"], 0.0)

    def test_zero_drift_and_missing_truth_are_not_counted_as_improvement(self):
        points = np.tile([1, 0, 2], (4, 1))
        knots = (-3, -2, -1, 0, 1, 2, 3, 4)
        prediction = Prediction(1, 2, 10.0, 10.0, 1, knots, points,
                                Registration(10.0, (0, 0, 0), 6, 0.0))
        truth = {
            1: [[10.0, 0, 0, 2], [10.2, 0, 0, 2]],
            2: [[10.0, 1, 0, 2], [10.2, 1, 0, 2]],
        }
        result, rows = score_predictions([prediction], truth, {}, 1.2, 0.05)
        self.assertEqual(result["status"], "partially_scored")
        self.assertEqual([round(r["sample_s"], 2) for r in rows], [10.0, 10.2])
        self.assertAlmostEqual(result["observers"]["1"]["paired_corrected_error_m"]["max"], 0)
        self.assertEqual(result["observers"]["1"]["raw_missed_danger_samples"], 0)
        self.assertEqual(result["observers"]["2"]["paired_samples"], 0)

    def test_registration_expires_inside_the_scoring_horizon(self):
        knots = (-3, -2, -1, 0, 1, 2, 3, 4)
        prediction = Prediction(1, 2, 10.0, 10.0, 1, knots, np.tile([1, 0, 2], (4, 1)),
                                Registration(4.05, (0, 0, 0), 6, 0.0))
        times = np.linspace(10.0, 10.5, 51)
        truth = {uid: [[t, uid - 1, 0, 2] for t in times] for uid in (1, 2)}
        _, rows = score_predictions([prediction], truth, {}, 1.2, 0.05)
        self.assertEqual(len(rows), 6)
        self.assertIsNotNone(rows[0]["corrected_error_m"])
        self.assertTrue(all(row["corrected_error_m"] is None for row in rows[1:]))

    def test_launch_is_opt_in_and_uses_only_shadow_match_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            args = (ROOT, out, [[0, 0, 1.2], [5, 0, 2]])
            prepare_modules(*args, world_frame=True)
            wrapper = ET.parse(out / "modules.launch").getroot()
            self.assertFalse(any("drone_detect_lidar" in e.get("file", "")
                                 for e in wrapper.findall("include")))
            prepare_modules(*args, world_frame=True, swarm_num=2, tree_drift_shadow=True)
            wrapper = ET.parse(out / "modules.launch").getroot()
            lio = wrapper.find("include")
            self.assertEqual(lio.find("arg[@name='tree_trunks']").get("value"), "true")
            detector = next(e for e in wrapper.findall("include")
                            if "drone_detect_lidar" in e.get("file", ""))
            self.assertEqual(detector.find("arg[@name='use_external_trunks']").get("value"), "true")
            self.assertEqual(detector.find("arg[@name='output_ns']").get("value"),
                             "/tree_drift_shadow/uav1")
            self.assertEqual(detector.find("arg[@name='enable_relay']").get("value"), "false")


if __name__ == "__main__":
    unittest.main()
