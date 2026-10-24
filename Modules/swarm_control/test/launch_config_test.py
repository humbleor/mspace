#!/usr/bin/env python3
"""Offline launch loading: does not start a ROS master, driver, or flight node."""
import os
import unittest
import roslaunch
import rospkg

RP = rospkg.RosPack()
def add(config, package, relative, *args):
    path = os.path.join(RP.get_path(package), relative)
    roslaunch.xmlloader.XmlLoader().load(path, config, argv=list(args), verbose=False)
    return config
def value(config, key):
    return config.params[key].value

class LaunchTest(unittest.TestCase):
    def test_two_uav_names_frames_and_connections(self):
        cfg = roslaunch.config.ROSLaunchConfig()
        for uid in (1, 2):
            common = ("uav_id:=%d" % uid, "enable_world_frame:=true")
            add(cfg, "fast_lio", "launch/swarm_mid360_driver.launch", "uav_id:=%d" % uid)
            add(cfg, "fast_lio", "launch/mapping_mid360.launch", *common, "rviz:=false")
            add(cfg, "mavros_bringup", "launch/px4_mavros_swarm_basic.launch", *common)
            add(cfg, "prometheus_swarm_control", "launch/ego_swarm_control.launch", *common,
                "swarm_num:=2", "uav_init_y:=%d" % (5 if uid == 2 else 0))
            add(cfg, "ego_planner", "launch_new/real_ego_run.launch", *common)
            ns = "/uav%d" % uid
            self.assertEqual(value(cfg, ns+"/frames/world"), ns[1:]+"/lio_init")
            self.assertEqual(value(cfg, ns+"/frames/body"), ns[1:]+"/lio_body")
            self.assertEqual(value(cfg, ns+"/frames/high_freq_body"), ns[1:]+"/lio_body")
            self.assertEqual(value(cfg, ns+"/common/lid_topic"), ns+"/livox/lidar_192_168_2_174")
            self.assertEqual(value(cfg, ns+"/mavros/local_position/frame_id"), ns[1:]+"/fcu_local")
            self.assertEqual(value(cfg, ns+"/mavros/local_position/tf/child_frame_id"), ns[1:]+"/base_link")
            est = "/swarm_estimator_uav_%d" % uid
            ctrl = "/swarm_controller_uav_%d" % uid
            planner = "/uav_%d_ego_planner_node" % uid
            self.assertFalse(value(cfg, est+"/publish_visualization"))
            self.assertEqual(value(cfg, est+"/offset_y"), 0)
            self.assertTrue(value(cfg, ctrl+"/require_frame_ready"))
            self.assertFalse(value(cfg, ctrl+"/drift_correction/enable"))
            self.assertTrue(value(cfg, planner+"/fsm/odom_twist_in_body"))
            self.assertEqual(value(cfg, "/swarm_frame_adapter_uav_%d/init_y" % uid), 5 if uid == 2 else 0)
            nodes = {n.namespace.rstrip("/")+"/"+n.name: n for n in cfg.nodes}
            # roslaunch stores each remap as an ordered pair (list) of from/to values.
            def remaps(node):
                return {tuple(pair) for pair in node.remap_args}
            self.assertIn((ns+"/prometheus/drone_state", ns+"/local/drone_state"), remaps(nodes[est]))
            self.assertIn((ns+"/mavros/setpoint_raw/local", ns+"/world/setpoint_raw/local"), remaps(nodes[ctrl]))
            self.assertIn(("~odom_world", ns+"/world/odom"), remaps(nodes[planner]))
            self.assertIn(("~grid_map/cloud", ns+"/world/cloud"), remaps(nodes[planner]))
        names = [n.namespace.rstrip("/")+"/"+n.name for n in cfg.nodes]
        self.assertEqual(len(names), len(set(names)), "duplicate node name")
        self.assertIn("/uav1/livox_lidar_publisher2", names)
        self.assertIn("/uav2/livox_lidar_publisher2", names)

    def test_legacy_defaults(self):
        cfg = roslaunch.config.ROSLaunchConfig()
        add(cfg, "fast_lio", "launch/mapping_mid360.launch", "rviz:=false")
        add(cfg, "prometheus_swarm_control", "launch/ego_swarm_control.launch", "uav_init_y:=3")
        add(cfg, "ego_planner", "launch_new/real_ego_run.launch")
        self.assertEqual(value(cfg, "/frames/world"), "world")
        self.assertEqual(value(cfg, "/frames/body"), "body")
        self.assertEqual(value(cfg, "/frames/high_freq_body"), "base_link")
        self.assertEqual(value(cfg, "/swarm_estimator_uav_1/offset_y"), 3)
        self.assertTrue(value(cfg, "/swarm_estimator_uav_1/publish_visualization"))
        self.assertFalse(value(cfg, "/swarm_controller_uav_1/require_frame_ready"))
        self.assertFalse(value(cfg, "/uav_1_ego_planner_node/fsm/odom_twist_in_body"))
        self.assertFalse(any(n.type == "swarm_frame_adapter" for n in cfg.nodes))

    def test_single_uav_world_script_wiring(self):
        # Mirrors ego_fastlio2_one_livox.sh in its default world mode.
        cfg = roslaunch.config.ROSLaunchConfig()
        common = ("uav_id:=1", "enable_world_frame:=true")
        add(cfg, "fast_lio", "launch/swarm_mid360_driver.launch", "uav_id:=1")
        add(cfg, "fast_lio", "launch/mapping_mid360.launch", *common, "rviz:=false",
            "lidar_topic:=/uav1/livox/lidar_192_168_2_174",
            "imu_topic:=/uav1/livox/imu_192_168_2_174")
        add(cfg, "mavros_bringup", "launch/px4_mavros.launch", *common)
        add(cfg, "prometheus_swarm_control", "launch/ego_swarm_control.launch", *common, "swarm_num:=1")
        add(cfg, "ego_planner", "launch_new/real_ego_run.launch", *common)
        est, ctrl, planner = "/swarm_estimator_uav_1", "/swarm_controller_uav_1", "/uav_1_ego_planner_node"
        self.assertEqual(value(cfg, "/uav1/frames/world"), "uav1/lio_init")
        self.assertEqual(value(cfg, "/uav1/frames/body"), "uav1/lio_body")
        self.assertEqual(value(cfg, "/uav1/common/lid_topic"), "/uav1/livox/lidar_192_168_2_174")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/frame_id"), "uav1/fcu_local")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/tf/frame_id"), "uav1/fcu_local")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/tf/child_frame_id"), "uav1/base_link")
        self.assertEqual(value(cfg, "/uav1/mavros/imu/frame_id"), "uav1/base_link")
        self.assertFalse(value(cfg, est+"/publish_visualization"))
        self.assertEqual(value(cfg, est+"/offset_x"), 0)
        self.assertTrue(value(cfg, ctrl+"/require_frame_ready"))
        self.assertFalse(value(cfg, ctrl+"/drift_correction/enable"))
        self.assertTrue(value(cfg, planner+"/fsm/odom_twist_in_body"))
        self.assertEqual(value(cfg, "/swarm_frame_adapter_uav_1/init_x"), 0)
        self.assertEqual(value(cfg, "/swarm_frame_adapter_uav_1/init_yaw"), 0)
        nodes = {n.namespace.rstrip("/")+"/"+n.name: n for n in cfg.nodes}
        self.assertIn("/uav1/livox_lidar_publisher2", nodes)
        self.assertIn("/swarm_frame_adapter_uav_1", nodes)
        self.assertIn(("/uav1/prometheus/drone_state", "/uav1/local/drone_state"),
                      {tuple(pair) for pair in nodes[est].remap_args})
        self.assertIn(("/uav1/mavros/setpoint_raw/local", "/uav1/world/setpoint_raw/local"),
                      {tuple(pair) for pair in nodes[ctrl].remap_args})
        remaps = {tuple(pair) for pair in nodes[planner].remap_args}
        self.assertIn(("~odom_world", "/uav1/world/odom"), remaps)
        self.assertIn(("~grid_map/cloud", "/uav1/world/cloud"), remaps)

    def test_single_uav_legacy_mavros_keeps_world_frames(self):
        cfg = roslaunch.config.ROSLaunchConfig()
        add(cfg, "mavros_bringup", "launch/px4_mavros.launch")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/frame_id"), "world")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/tf/frame_id"), "world")
        self.assertEqual(value(cfg, "/uav1/mavros/local_position/tf/child_frame_id"), "base_link")

if __name__ == "__main__":
    unittest.main(verbosity=2)
