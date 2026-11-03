import sys, unittest, tempfile, json, struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Simulation/px4_forest/scripts"))
from fleet import load_fleet, allocate_ports, gazebo_environment, bounded_call
from module_launch import prepare_modules
from runtime_model import prepare_model, compose_world
from world_frame_evidence import initial_anchor
from scan_message import pack_scan
from motion import lidar_motion

sys.path.insert(0, str(ROOT / "Simulation/forest_lio/scripts"))
from common import POINT_DTYPE


def config(n=3):
    return {
        "vehicles": [
            {
                "uav_id": i,
                "instance": 19 + i,
                "initial": [0.0, 3.0 * (i - 1), 0.0, 0.0],
                "goals": [[1.0, 3.0 * (i - 1), 1.2]],
            }
            for i in range(1, n + 1)
        ]
    }


class FleetTests(unittest.TestCase):
    def test_cli_help_without_ros_environment(self):
        import subprocess, os

        env = dict(os.environ)
        env.pop("PYTHONPATH", None)
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "Simulation/px4_forest/scripts/run_swarm.py"),
                "--help",
            ],
            env=env,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--fleet", result.stdout)

    def test_gui_defaults_and_explicit_headless(self):
        from run_swarm import parser

        args = ["--fleet", "fleet.json", "--output", "run"]
        self.assertEqual(parser().parse_args(args).gui, "both")
        for mode in ("none", "gazebo", "rviz", "both"):
            self.assertEqual(parser().parse_args(args + ["--gui", mode]).gui, mode)

    def test_rviz_displays_follow_fleet_world_topics(self):
        import yaml
        from fleet_visualization import prepare_rviz

        with tempfile.TemporaryDirectory() as tmp:
            import shutil
            root = Path(tmp)
            directory = root / "Simulation/px4_forest/config"
            directory.mkdir(parents=True)
            shutil.copyfile(ROOT / "Simulation/px4_forest/config/forest.rviz", directory / "forest.rviz")
            output = root / "run"
            output.mkdir()
            path = prepare_rviz(root, output, load_fleet(config()))
            self.assertEqual((directory / "fleet.rviz").read_text(), path.read_text())
            view = yaml.safe_load(path.read_text())["Visualization Manager"]
            self.assertEqual(view["Global Options"]["Fixed Frame"], "world")
            displays = view["Displays"]
            self.assertIn("/sim/view/forest", {d.get("Topic") for d in displays})
            groups = [d for d in displays if d["Class"] == "rviz/Group"]
            self.assertEqual([g["Name"] for g in groups], ["uav1", "uav2", "uav3"])
            children = [d for g in groups for d in g["Displays"]]
            topics = {d.get("Topic", d.get("Marker Topic")) for d in children}
            for i in range(1, 4):
                self.assertIn("/uav%d/world/odom" % i, topics)
                self.assertIn("/uav%d/world/cloud" % i, topics)
                self.assertIn("/uav_%d_ego_planner_node/optimal_list" % i, topics)
                self.assertIn("/sim/view/uav%d/goals" % i, topics)
                self.assertIn("/sim/view/uav%d/vehicle" % i, topics)
            for display in children:
                if display["Class"] == "rviz/PointCloud2":
                    self.assertFalse(display["Enabled"])
                if display["Class"] == "rviz/Odometry":
                    self.assertEqual(display["Keep"], 1)
            self.assertNotIn("/sim/view/truth_path", topics)

    def test_vehicle_label_follows_world_position_without_mutating_it(self):
        from fleet_visualization import vehicle_label
        from rospy import Time

        position = [2., 5., 1.2]
        marker = vehicle_label("uav2", position, 2, Time(12)).markers[0]
        self.assertEqual(marker.header.frame_id, "world")
        self.assertEqual(marker.header.stamp, Time(12))
        self.assertEqual(marker.type, marker.TEXT_VIEW_FACING)
        self.assertEqual(marker.text, "uav2")
        self.assertEqual([marker.pose.position.x, marker.pose.position.y], [2., 5.])
        self.assertAlmostEqual(marker.pose.position.z, 1.8)
        self.assertEqual(position, [2., 5., 1.2])
        self.assertEqual(marker.pose.orientation.w, 1.)
        self.assertGreater(marker.scale.z, 0)
        self.assertEqual(marker.color.a, 1.)

    def test_route_markers_have_distinct_colors_and_preserve_world_goals(self):
        from fleet_visualization import route_markers
        goals = [[0, 5, 2], [5, 5, 2]]
        first = route_markers(goals, 1)
        second = route_markers(goals, 2)
        self.assertNotEqual(first.markers[0].color, second.markers[0].color)
        for array in (first, second):
            spheres = [m for m in array.markers if m.type == m.SPHERE]
            self.assertEqual(len(spheres), 2)
            self.assertEqual([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                              for m in spheres], goals)
            self.assertTrue(all(m.header.frame_id == "world" for m in array.markers))
            self.assertTrue(all(m.color.a == 1 for m in array.markers))

    def test_independent_world_grids_preserve_legacy_goals(self):
        data = config(3)
        for row, y in zip(data["vehicles"][:2], (0, 5)):
            row.pop("goals")
            row["initial"] = [0, y, 0, 0]
            row["grid"] = {
                "x": {"min": 0, "max": 2, "step": 2},
                "y": {"min": y, "max": y + 1, "step": 1},
                "z": {"min": 2, "max": 2, "step": 2},
            }
        data["vehicles"][2]["initial"] = [0, 10, 0, 0]
        fleet = load_fleet(data)
        self.assertEqual(fleet[0].goals, ((0., 0., 2.), (2., 0., 2.), (2., 1., 2.), (0., 1., 2.)))
        self.assertEqual(fleet[1].goals, ((0., 5., 2.), (2., 5., 2.), (2., 6., 2.), (0., 6., 2.)))
        self.assertEqual(fleet[2].goals, ((1., 6., 1.2),))
        self.assertIsNone(fleet[2].grid)
        self.assertEqual(fleet[0].grid["grid_direction"], 0)
        self.assertEqual(fleet[0].grid["waypointDistriFlag"], 0)
        self.assertNotIn("grid_direction", data["vehicles"][0]["grid"])

    def test_grid_direction_one_uses_own_axis_order(self):
        data = config(1)
        row = data["vehicles"][0]
        row.pop("goals")
        row["grid"] = {
            "grid_direction": 1,
            "x": {"min": 10, "max": 12, "step": 2},
            "y": {"min": 5, "max": 6, "step": 1},
            "z": {"min": 2, "max": 2, "step": 1},
        }
        self.assertEqual(load_fleet(data)[0].goals,
                         ((10., 5., 2.), (10., 6., 2.), (12., 6., 2.), (12., 5., 2.)))

    def test_invalid_or_ambiguous_grids_rejected(self):
        import copy

        grid = {axis: {"min": 2, "max": 2, "step": 1} for axis in "xyz"}
        cases = [None, [], {}, dict(grid, x=None), dict(grid, x={"min": 0}),
                 dict(grid, x={"min": 0, "max": 1, "step": 0}),
                 dict(grid, x={"min": 2, "max": 1, "step": 1}),
                 dict(grid, x={"min": 0, "max": float("inf"), "step": 1}),
                 dict(grid, x={"min": 0, "max": 1, "step": True}),
                 dict(grid, x={"min": 0, "max": 1, "step": "1"}),
                 dict(grid, z={"min": 0.4, "max": 1, "step": 1}),
                 dict(grid, waypointDistriFlag=1), dict(grid, grid_direction=3),
                 dict(grid, x={"min": 0, "max": 300, "step": 1}),
                 dict(grid, x={"min": 0, "max": 2001, "step": 1})]
        for value in cases:
            data = config(1)
            data["vehicles"][0].pop("goals")
            data["vehicles"][0]["grid"] = copy.deepcopy(value)
            with self.subTest(grid=value), self.assertRaises(ValueError):
                load_fleet(data)
        for goals in ([], [[1, 1, 2]], None):
            data = config(1)
            data["vehicles"][0].update(grid=grid, goals=goals)
            with self.subTest(goals=goals), self.assertRaises(ValueError):
                load_fleet(data)

    def test_grid_prepare_only_generates_separate_native_launches(self):
        import subprocess

        px4 = ROOT / "artifacts/px4_forest/PX4-Autopilot"
        if not (px4 / "build/px4_sitl_default/bin/px4").exists():
            self.skipTest("local pinned PX4 build missing")
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            result = subprocess.run(
                [sys.executable, str(ROOT / "Simulation/px4_forest/scripts/run_swarm.py"),
                 "--fleet", str(ROOT / "Simulation/px4_forest/config/fleet_two.json"),
                 "--output", str(out), "--prepare-only"],
                capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = json.loads((out / "manifest.json").read_text())
            for row, y in zip(manifest["vehicles"], (0, 5)):
                directory = out / ("uav%d" % row["uav_id"])
                self.assertEqual(row["initial"], [0, y, 0, 0])
                self.assertEqual(len(row["goals"]), 6)  # Current example: x=0,4,8 and two y rows.
                self.assertEqual(row["goals"][0], [0, y, 2])
                route = json.loads((directory / "route_goals.json").read_text())
                self.assertEqual(route["generator"], "generateWps")
                self.assertEqual(route["goals"][0], [0, y, 1.2])
                self.assertEqual(route["goals"][1:], row["goals"])
                info = json.loads((directory / "module_alignment.json").read_text())
                self.assertEqual(info["flight_type"], 3)
                self.assertEqual(float(info["overrides"]["box_min_y"]), y)
                self.assertEqual(float(info["overrides"]["step_X"]), 4)
                planner = ET.parse(directory / "planner_modules.launch").getroot()
                params = {e.get("name"): e.get("value") for e in planner.findall("param")}
                prefix = "/uav_%d_ego_planner_node/fsm/" % row["uav_id"]
                self.assertNotIn(prefix + "waypoint_num", params)

    def test_unresponsive_service_cannot_block_shutdown(self):
        import threading, time

        event = threading.Event()
        start = time.monotonic()
        with self.assertRaises(TimeoutError):
            bounded_call(lambda: event.wait(2), 0.02)
        self.assertLess(time.monotonic() - start, 0.3)
        event.set()
        self.assertEqual(bounded_call(lambda: 42, 0.2), 42)

    def test_gazebo_load_does_not_scale_physics_once_per_aircraft(self):
        env = {"PX4_SIM_SPEED_FACTOR": ".25", "ROS_HOME": "/run/ros_home"}
        neutral = gazebo_environment(env)
        self.assertEqual(neutral["PX4_SIM_SPEED_FACTOR"], "1")
        self.assertEqual(env["PX4_SIM_SPEED_FACTOR"], ".25")
        self.assertEqual(neutral["ROS_HOME"], "/run/ros_home")

    def test_three_vehicles_have_unique_resources(self):
        fleet = load_fleet(config())
        self.assertEqual([v.system_id for v in fleet], [21, 22, 23])
        self.assertEqual([v.namespace for v in fleet], ["/uav1", "/uav2", "/uav3"])
        ports = allocate_ports(fleet)
        numbers = [ports["ros"], ports["gazebo"]] + [
            p for v in ports["vehicles"].values() for p in v.values()
        ]
        self.assertEqual(len(numbers), len(set(numbers)))

    def test_invalid_ids_and_placements_rejected(self):
        for change in ("duplicate", "gap", "instance", "nan", "z", "overlap", "goal"):
            data = config()
            if change == "duplicate":
                data["vehicles"][1]["uav_id"] = 1
            if change == "gap":
                data["vehicles"][2]["uav_id"] = 4
            if change == "instance":
                data["vehicles"][1]["instance"] = 20
            if change == "nan":
                data["vehicles"][1]["initial"][0] = float("nan")
            if change == "z":
                data["vehicles"][1]["initial"][2] = 1.0
            if change == "overlap":
                data["vehicles"][1]["initial"] = [0, 0, 0, 0]
            if change == "goal":
                data["vehicles"][1]["goals"] = []
            with self.subTest(change=change), self.assertRaises(ValueError):
                load_fleet(data)

    def test_modules_are_namespaced_for_three_vehicles(self):
        with tempfile.TemporaryDirectory() as tmp:
            for v in load_fleet(config()):
                out = Path(tmp) / v.name
                out.mkdir()
                info = prepare_modules(
                    ROOT,
                    out,
                    [[*v.initial[:2], 1.2], *v.goals],
                    world_frame=True,
                    uav_id=v.uav_id,
                    swarm_num=3,
                    initial=v.initial,
                    sensor_prefix=v.sensor_prefix,
                )
                wrapper = ET.parse(out / "modules.launch").getroot()
                planner = ET.parse(out / "planner_modules.launch").getroot()
                lio = next(
                    e
                    for e in wrapper.findall("include")
                    if "mapping_mid360" in e.get("file")
                )
                args = {e.get("name"): e.get("value") for e in lio.findall("arg")}
                self.assertEqual(args["uav_id"], str(v.uav_id))
                self.assertEqual(args["lidar_topic"], v.sensor_prefix + "/mid360/lidar")
                self.assertEqual(info["planner_odom"], v.namespace + "/world/odom")
                visualization = next(
                    e
                    for e in planner.findall("node")
                    if e.get("type") == "odom_visualization"
                )
                self.assertEqual(
                    visualization.find("param[@name='tf45']").get("value"), "false"
                )
                self.assertEqual(wrapper.find("group/param").get("name"), "output_root")
                height = wrapper.find(
                    "param[@name='/swarm_controller_uav_%d/takeoff_height']" % v.uav_id
                )
                self.assertIsNotNone(height)
                self.assertEqual(float(height.get("value")), 1.2)
                self.assertIsNone(planner.find("remap[@from='/traj_start_trigger']"))
                self.assertEqual(info["mission_handover"], "shared_offboard_gate")
                control = next(e for e in wrapper.findall("include") if "ego_swarm_control" in e.get("file"))
                self.assertEqual(control.find("arg[@name='enable_mission_gate']").get("value"), "true")
                self.assertEqual(planner.find("arg[@name='enable_mission_gate']").get("default"), "true")
                self.assertTrue(
                    any(
                        e.get("name")
                        == "/uav_%d_ego_planner_node/fsm/strict_waypoint_tracking"
                        % v.uav_id
                        for e in planner.findall("param")
                    )
                )

    def test_model_generation_uses_per_vehicle_startup_and_one_world(self):
        px4 = ROOT / "artifacts/px4_forest/PX4-Autopilot"
        if not (px4 / "Tools/sitl_gazebo/scripts/jinja_gen.py").exists():
            self.skipTest("local pinned PX4 missing")
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            models = []
            for v in load_fleet(config()):
                directory = out / v.name
                directory.mkdir()
                ports = {
                    "sim": 24000 + v.uav_id,
                    "px4": 24100 + v.uav_id,
                    "mavros": 24200 + v.uav_id,
                    "unused": 24300 + v.uav_id,
                }
                prepare_model(
                    px4,
                    directory,
                    "/tmp/plugin.so",
                    ports,
                    v.instance,
                    model_name=v.model_name,
                    pose=[*v.initial[:2], 0.25, 0, 0, v.initial[3]],
                    sensor_prefix=v.sensor_prefix,
                    physics_speed=0.25 if v.uav_id == 1 else None,
                )
                models.append(directory / "iris.sdf")
                model = ET.parse(models[-1]).getroot().find("model")
                self.assertEqual(model.get("name"), v.model_name)
                rate = model.find(
                    "plugin[@name='mspace_forest_plant']/physics_update_rate"
                )
                if v.uav_id == 1:
                    self.assertEqual(float(rate.text), 62.5)
                else:
                    self.assertIsNone(rate)
                self.assertEqual(
                    model.find(
                        "plugin[@name='mspace_forest_plant']/sensor_prefix"
                    ).text,
                    v.sensor_prefix,
                )
                self.assertIn(
                    str(ports["sim"]),
                    (directory / "etc/init.d-posix/px4-rc.simulator").read_text(),
                )
            compose_world(models, out / "forest.world")
            self.assertEqual(
                len(ET.parse(out / "forest.world").getroot().findall("world/model")), 3
            )

    def test_nonzero_anchor_checks_configured_pose(self):
        rows = [[1, 2, 3, 0, 0, 0, np.sin(0.4), np.cos(0.4)]]
        self.assertTrue(
            initial_anchor(rows, 1, expected_position=[2, 3, 0], expected_yaw=0.8)[
                "passed"
            ]
        )
        self.assertFalse(initial_anchor(rows, 1)["passed"])

    def test_wire_frame_is_unique(self):
        msg = pack_scan(4, 1.2, np.zeros(2, dtype=POINT_DTYPE), frame_id="uav2/lidar")
        data = msg.wire_data
        size = struct.unpack_from("<I", data, 12)[0]
        self.assertEqual(data[16 : 16 + size], b"uav2/lidar")
        self.assertEqual(msg.header.frame_id, "uav2/lidar")

    def test_shared_origin_preserves_inter_vehicle_separation(self):
        rays = np.array([[1.0, 0, 0]])
        h1 = [[0, 10, 0, 0, 0, 0, 0, 1], [1, 10, 0, 0, 0, 0, 0, 1]]
        h2 = [[0, 10, 3, 0, 0, 0, 0, 1], [1, 10, 3, 0, 0, 0, 0, 1]]
        o1, _ = lidar_motion(h1, [0.5], [10, 0, 0], [0, 0, 0], np.eye(3), rays)
        o2, _ = lidar_motion(h2, [0.5], [10, 0, 0], [0, 0, 0], np.eye(3), rays)
        np.testing.assert_allclose(o2 - o1, [[0, 3, 0]])


if __name__ == "__main__":
    unittest.main()
