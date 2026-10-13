import sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from module_launch import prepare_modules

class ModuleBinding(unittest.TestCase):
    def test_grid_uses_original_templates_and_native_task(self):
        root=Path(__file__).resolve().parents[3]
        grid={"x":{"min":0,"max":32,"step":8},"y":{"min":0,"max":16,"step":8},"z":{"min":2,"max":2,"step":2}}
        with tempfile.TemporaryDirectory() as d:
            info=prepare_modules(root,Path(d),np.array([[0,0,1.2],[0,0,2]]),grid)
            wrapper=ET.parse(Path(d)/"modules.launch").getroot()
            includes=[x.get("file") for x in wrapper.findall("include")]
            self.assertIn(str(root/"Modules/fast_lio2/launch/mapping_mid360.launch"),includes)
            self.assertIn(str(root/"Modules/swarm_control/launch/ego_swarm_control.launch"),includes)
            planner=ET.parse(Path(d)/"planner_modules.launch").getroot()
            original=ET.parse(root/"Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch").getroot()
            self.assertEqual(ET.tostring(planner.find("node")),ET.tostring(original.find("node")))
            args={a.get("name"):a.get("value") for a in planner.find("include").findall("arg")}
            self.assertEqual(args["flight_type"],"3");self.assertEqual(args["box_max_x"],"32")
            self.assertEqual(args["max_vel"],"0.6");self.assertEqual(args["max_acc"],"0.5")
            self.assertEqual(args["odometry_topic"],"$(arg odom_topic)")
            self.assertEqual(next(a.get("value") for a in planner.findall("arg") if a.get("name")=="odom_topic"),"mavros/local_position/odom")
            self.assertEqual(info["control_takeoff_height"],1.2)
            params={e.get("name"):e.get("value") for e in planner.findall("param")}
            self.assertEqual(params["/uav_1_ego_planner_node/fsm/realworld_experiment"],"true")
            self.assertEqual(params["/uav_1_ego_planner_node/fsm/strict_waypoint_tracking"],"true")
            self.assertTrue(info["algorithms_modified"])
    def test_recorded_route_uses_native_preset_parameters(self):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as d:
            prepare_modules(root,Path(d),np.array([[0,0,1.2],[1,2,1.5]]))
            planner=ET.parse(Path(d)/"planner_modules.launch").getroot()
            args={a.get("name"):a.get("value") for a in planner.find("include").findall("arg")}
            self.assertEqual(args["flight_type"],"2")
            params={a.get("name"):a.get("value") for a in planner.findall("param")}
            self.assertEqual(params["/uav_1_ego_planner_node/fsm/waypoint_num"],"1")
            self.assertEqual(params["/uav_1_ego_planner_node/fsm/waypoint0_z"],"1.5")

    def test_full_grid_map_covers_all_goals_and_planning_padding(self):
        root=Path(__file__).resolve().parents[3]
        grid={"x":{"min":0,"max":32,"step":8},"y":{"min":0,"max":16,"step":8},"z":{"min":2,"max":2,"step":2}}
        template=root/"Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch"
        original=template.read_bytes()
        with tempfile.TemporaryDirectory() as d:
            info=prepare_modules(root,Path(d),np.array([[0,0,1.2],[0,0,2]]),grid)
            layout=info["simulation_map_layout"]
            self.assertEqual(layout["size_m"],{"x":96.,"y":64.,"z":10.})
            self.assertEqual(layout["dense_buffer_estimate_bytes"],921600000)
            self.assertEqual(layout["original_dense_buffer_estimate_bytes"],6000000000)
            self.assertEqual(layout["resolution_m"],.1)
            for i,axis in enumerate("xy"):
                self.assertGreaterEqual(layout["size_m"][axis]/2,grid[axis]["max"]+layout["xy_padding_m"][i])
            planner=ET.parse(Path(d)/"planner_modules.launch").getroot()
            arguments={e.get("name"):e.get("value") for e in planner.findall("arg")}
            self.assertEqual(float(arguments["map_size_x"]),96)
            self.assertEqual(float(arguments["map_size_y"]),64)
        self.assertEqual(template.read_bytes(),original)

    def test_negative_route_keeps_symmetric_map_and_original_z(self):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as d:
            layout=prepare_modules(root,Path(d),np.array([[0,0,1.2],[-50,-25,2]]))["simulation_map_layout"]
            self.assertGreaterEqual(layout["size_m"]["x"]/2,50+layout["xy_padding_m"][0])
            self.assertGreaterEqual(layout["size_m"]["y"]/2,25+layout["xy_padding_m"][1])
            self.assertEqual(layout["size_m"]["z"],10)

    def test_sizing_uses_actual_local_range_and_resolution(self):
        root=Path(__file__).resolve().parents[3]
        planner=ET.parse(root/"Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch").getroot()
        advanced=ET.parse(root/"Modules/ego_planner_swarm/plan_manage/launch_new/advanced_param_px4.xml").getroot()
        for e in advanced.iter("param"):
            if e.get("name")=="grid_map/local_update_range_x":e.set("value","20")
        from module_launch import task_map_layout
        layout=task_map_layout([[32,16,2]],None,planner,advanced)
        self.assertEqual(layout["xy_padding_m"][0],27.5)
        self.assertGreaterEqual(layout["size_m"]["x"]/2,59.5)
        self.assertEqual(layout["resolution_m"],.1)

    def test_invalid_or_below_ground_goals_rejected(self):
        root=Path(__file__).resolve().parents[3]
        for goals in ([],[[float("nan"),0,1]],[[0,0,-1]]):
            with tempfile.TemporaryDirectory() as d, self.assertRaises(ValueError):
                prepare_modules(root,Path(d),goals)

    def test_high_goal_expands_z_instead_of_clipping(self):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as d:
            layout=prepare_modules(root,Path(d),[[0,0,1.2],[0,0,12]])["simulation_map_layout"]
            self.assertGreaterEqual(layout["size_m"]["z"]+layout["ground_height_m"],12+layout["planning_horizon_m"]+1)

if __name__=="__main__":unittest.main()
