import sys, unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fleet import load_fleet
from world_frame_evidence import cloud_alignment
from fleet_evidence import pair_separation, audit_graph, all_ready, all_settled, bridge_traffic


def rows(points, times=(0.0, 0.04)):
    return [[t, *p, 0, 0, 0, 1] for t, p in zip(times, points)]


class FleetEvidenceTests(unittest.TestCase):
    def test_cloud_alignment_does_not_delete_the_matching_map_points(self):
        ground = np.zeros((200001, 3))
        ground[:, 0] = np.arange(len(ground))
        cloud = ground[1:101:2]
        result = cloud_alignment(cloud, ground)
        self.assertTrue(result["passed"])
        self.assertEqual(result["median_m"], 0.0)

    def test_crossing_between_samples_is_rejected(self):
        result = pair_separation(
            {
                "uav1": rows([[-1, 0, 0], [1, 0, 0]]),
                "uav2": rows([[1, 0, 0], [-1, 0, 0]]),
            },
            0.8,
        )
        self.assertFalse(result["passed"])
        self.assertAlmostEqual(result["pairs"][0]["minimum_m"], 0.0)

    def test_parallel_motion_and_three_pairs(self):
        traces = {"uav%d" % i: rows([[0, i * 3, 0], [1, i * 3, 0]]) for i in (1, 2, 3)}
        result = pair_separation(traces, 1.2)
        self.assertTrue(result["passed"])
        self.assertEqual(len(result["pairs"]), 3)
        self.assertAlmostEqual(result["pairs"][0]["minimum_m"], 3.0)

    def test_missing_gapped_or_short_observations_fail(self):
        good = rows([[0, 0, 0], [0, 0, 0]])
        for bad in (
            [],
            rows([[0, 3, 0], [0, 3, 0]], (2, 2.04)),
            rows([[0, 3, 0], [0, 3, 0]], (0, 0.2)),
        ):
            self.assertFalse(
                pair_separation({"uav1": good, "uav2": bad}, 1.0)["passed"]
            )

    def test_vehicle_completion_requires_every_vehicle(self):
        self.assertFalse(all_ready([True, False]))
        self.assertTrue(all_ready([True, True, True]))
        self.assertFalse(all_ready([]))
        self.assertFalse(all_settled([10.0, None], 14.0))
        self.assertFalse(all_settled([10.0, 12.0], 14.0))
        self.assertTrue(all_settled([10.0, 12.0], 16.0))

    def test_bridge_traffic_requires_each_peer_and_predecessor_chain(self):
        fleet=load_fleet({"vehicles":[{"uav_id":i,"instance":19+i,"initial":[0,3*(i-1),0,0],
                                    "goals":[[1,3*(i-1),1.2]]} for i in (1,2,3)]})
        udp={v.name:{str(peer.uav_id):1 for peer in fleet if peer!=v} for v in fleet}
        tcp={"uav1":0,"uav2":1,"uav3":1}
        self.assertTrue(bridge_traffic(fleet,udp,tcp)["passed"])
        udp["uav1"]["3"]=0
        self.assertFalse(bridge_traffic(fleet,udp,tcp)["passed"])
        udp["uav1"]["3"]=1;tcp["uav3"]=0
        self.assertFalse(bridge_traffic(fleet,udp,tcp)["passed"])

    def test_bridge_graph_requires_local_tx_rx_and_rejects_ros_bypass(self):
        fleet=load_fleet({"vehicles":[{"uav_id":i,"instance":19+i,
                          "initial":[0,3*(i-1),0,0],"goals":[[1,3*(i-1),1.2]]} for i in (1,2)]})
        pubs,subs={},{}
        for v in fleet:
            ns=v.namespace;uid=v.uav_id;planner="/uav_%d_ego_planner_node"%uid
            bridge="/uav_%d_bridge_node"%uid;adapter="/swarm_frame_adapter_uav_%d"%uid
            for topic,node in {ns+"/mavros/setpoint_raw/local":adapter,
                               ns+"/mavros/vision_pose/pose":"/swarm_estimator_uav_%d"%uid,
                               ns+"/planning/ego/traj_cmd":ns+"_traj_server",
                               ns+"/world/odom":adapter,ns+"/world/cloud":adapter,
                               ns+"/prometheus/drone_state":adapter,
                               v.sensor_prefix+"/mid360/lidar":"/forest_swarm",
                               v.sensor_prefix+"/mid360/imu":"/gazebo"}.items():pubs[topic]=[node]
            for kind in ("lidar","imu"):subs[v.sensor_prefix+"/mid360/"+kind]=[ns+"/laserMapping"]
            for topic,publisher,subscriber in [(ns+"/bridge/tx/bspline",planner,bridge),
                                               (ns+"/bridge/rx/bspline",bridge,planner)]:
                pubs[topic]=[publisher];subs[topic]=[subscriber]
            pubs[ns+"/bridge/tx/multi_traj"]=[planner]
            if uid<2:subs[ns+"/bridge/tx/multi_traj"]=[bridge]
            if uid>1:
                pubs[ns+"/bridge/rx/prev_multi_traj"]=[bridge]
                subs[ns+"/bridge/rx/prev_multi_traj"]=[planner]
        self.assertTrue(audit_graph(fleet,pubs,subs,use_bridge=True)["passed"])
        pubs["/broadcast_bspline"]=["/uav_1_ego_planner_node"]
        self.assertFalse(audit_graph(fleet,pubs,subs,use_bridge=True)["passed"])
        del pubs["/broadcast_bspline"]
        subs["/uav1/planning/swarm_trajs"]=["/uav_2_ego_planner_node"]
        self.assertFalse(audit_graph(fleet,pubs,subs,use_bridge=True)["passed"])
        del subs["/uav1/planning/swarm_trajs"]
        subs["/uav2/bridge/rx/bspline"]=[]
        self.assertFalse(audit_graph(fleet,pubs,subs,use_bridge=True)["passed"])

    def test_truth_and_cross_sensor_subscription_rejected(self):
        fleet = load_fleet(
            {
                "vehicles": [
                    {
                        "uav_id": 1,
                        "instance": 20,
                        "initial": [0, 0, 0, 0],
                        "goals": [[1, 0, 1.2]],
                    }
                ]
            }
        )
        pubs = {
            "/uav1/mavros/setpoint_raw/local": ["/swarm_frame_adapter_uav_1"],
            "/uav1/mavros/vision_pose/pose": ["/swarm_estimator_uav_1"],
            "/uav1/planning/ego/traj_cmd": ["/uav1_traj_server"],
            "/uav1/world/odom": ["/swarm_frame_adapter_uav_1"],
            "/uav1/world/cloud": ["/swarm_frame_adapter_uav_1"],
            "/uav1/prometheus/drone_state": ["/swarm_frame_adapter_uav_1"],
            "/sim/uav1/mid360/lidar": ["/forest_swarm"],
            "/sim/uav1/mid360/imu": ["/gazebo"],
        }
        subs = {
            "/sim/uav1/mid360/lidar": ["/uav1/laserMapping"],
            "/sim/uav1/mid360/imu": ["/uav1/laserMapping"],
        }
        self.assertTrue(audit_graph(fleet, pubs, subs)["passed"])
        subs["/sim/uav2/mid360/lidar"] = ["/uav1/laserMapping"]
        self.assertFalse(audit_graph(fleet, pubs, subs)["passed"])
        del subs["/sim/uav2/mid360/lidar"]
        subs["/sim/uav1/plant/odom"] = ["/swarm_estimator_uav_1"]
        self.assertFalse(audit_graph(fleet, pubs, subs)["passed"])


if __name__ == "__main__":
    unittest.main()
