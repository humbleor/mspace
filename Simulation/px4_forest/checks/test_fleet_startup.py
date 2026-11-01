import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Simulation/px4_forest/scripts"))
from forest_geometry import DEFAULTS
from fleet_startup import StartupStages, cached_geometry, open_windows, contact_message, algorithm_fingerprint


class StartupTests(unittest.TestCase):
    def geometry(self, points):
        import hashlib
        return {"schema_version": 3, "frame": "world",
                "local_map_sha256": hashlib.sha256(np.ascontiguousarray(points, dtype="<f8").tobytes()).hexdigest(),
                "parameters": dict(DEFAULTS),
                "colliders": [{"kind": "trunk", "a": [0,0,0], "b": [0,0,2], "radius": .2}]}

    def test_stages_print_elapsed_and_persist_failed_stage(self):
        with tempfile.TemporaryDirectory() as tmp, io.StringIO() as stream:
            timer = StartupStages(Path(tmp))
            with contextlib.redirect_stdout(stream):
                with timer.stage("Read map"):
                    pass
                with self.assertRaises(ValueError):
                    with timer.stage("Build renderer"):
                        raise ValueError("bad map")
            records = json.loads((Path(tmp) / "startup_timing.json").read_text())
            self.assertEqual([r["status"] for r in records], ["ok", "failed"])
            self.assertGreaterEqual(records[0]["wall_s"], 0)
            self.assertIn("Read map", stream.getvalue())
            self.assertIn("failed", stream.getvalue())

    def test_cache_reuses_geometry_and_invalidates_map_or_algorithm(self):
        points = np.zeros((3, 3))
        calls = []
        def extract(p):
            calls.append(1)
            return self.geometry(p)
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp) / "cache"
            with patch("fleet_startup.extract_geometry", side_effect=extract):
                first, info = cached_geometry(points, cache)
                self.assertEqual(info["source"], "extracted")
                again, info = cached_geometry(points.copy(), cache)
                self.assertEqual(info["source"], "cache")
                self.assertEqual(info["algorithm_sha256"], algorithm_fingerprint())
                self.assertTrue(info["geometry_sha256"])
                self.assertEqual(first, again)
                cached_geometry(points + 1, cache)
                with patch("fleet_startup.algorithm_fingerprint", return_value="changed"):
                    cached_geometry(points, cache)
            self.assertEqual(len(calls), 3)

    def test_corrupt_cache_rebuilds_and_explicit_mismatch_fails(self):
        points = np.zeros((3, 3))
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp) / "cache"
            with patch("fleet_startup.extract_geometry", side_effect=self.geometry) as extract:
                _, info = cached_geometry(points, cache)
                Path(info["cache_path"]).write_text("broken")
                _, info = cached_geometry(points, cache)
                self.assertEqual(info["source"], "extracted")
                self.assertEqual(extract.call_count, 2)
                explicit = Path(tmp) / "wrong.json"
                explicit.write_text(json.dumps(self.geometry(points + 1)))
                with self.assertRaises(ValueError):
                    cached_geometry(points, cache, explicit=explicit)

    def test_previous_run_reuse_and_parameter_mismatch(self):
        points = np.zeros((3, 3))
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            old = runs / "previous" / "forest_geometry.json"
            old.parent.mkdir()
            old.write_text(json.dumps(self.geometry(points)))
            provenance = old.with_name("geometry_cache.json")
            import hashlib
            provenance.write_text(json.dumps({"source": "extracted", "algorithm_sha256": algorithm_fingerprint(),
                "geometry_sha256": hashlib.sha256(json.dumps(self.geometry(points), sort_keys=True).encode()).hexdigest()}))
            with patch("fleet_startup.extract_geometry", side_effect=self.geometry) as extract:
                _, info = cached_geometry(points, runs / "cache", previous_root=runs)
                self.assertEqual(info["source"], "previous_run")
                self.assertEqual(extract.call_count, 0)
                old_data = self.geometry(points)
                old_data["parameters"]["voxel"] = 999
                old.write_text(json.dumps(old_data))
                _, info = cached_geometry(points, runs / "other_cache", previous_root=runs)
                self.assertEqual(info["source"], "extracted")
                self.assertEqual(extract.call_count, 1)

    def test_legacy_or_explicit_output_does_not_seed_default_cache(self):
        points = np.zeros((3, 3))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root / "previous" / "forest_geometry.json"
            old.parent.mkdir()
            old.write_text(json.dumps(self.geometry(points)))
            for source in (None, "explicit"):
                if source:
                    old.with_name("geometry_cache.json").write_text(json.dumps({
                        "source": source, "algorithm_sha256": algorithm_fingerprint()}))
                with patch("fleet_startup.extract_geometry", side_effect=self.geometry) as extract:
                    _, info = cached_geometry(points, root / str(source), previous_root=root)
                    self.assertEqual(info["source"], "extracted")
                    self.assertEqual(extract.call_count, 1)

    def test_windows_launch_with_tracked_runner(self):
        calls = []
        def launch(args, log, critical=True):
            calls.append((list(map(str,args)), log, critical))
            return log
        windows = open_windows("both", Path("run/fleet.rviz"), launch)
        self.assertEqual(windows, ["rviz.log", "gazebo_gui.log"])
        self.assertEqual(calls, [(["rviz", "-d", "run/fleet.rviz"], "rviz.log", False),
                                 (["gzclient"], "gazebo_gui.log", False)])
        self.assertEqual(open_windows("none", Path("unused"), launch), [])

    def test_contact_message_identifies_tree_and_time(self):
        text = contact_message("uav1", {"sim_time_s":39.472,
                               "collision1":"iris::rotor_0", "collision2":"forest::trunk_0001"})
        self.assertIn("39.472", text)
        self.assertIn("iris::rotor_0", text)
        self.assertIn("forest::trunk_0001", text)


if __name__ == "__main__":
    unittest.main()
