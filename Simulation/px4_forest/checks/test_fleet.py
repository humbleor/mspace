import sys, unittest, tempfile, json, struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"Simulation/px4_forest/scripts"))
from fleet import load_fleet, allocate_ports, gazebo_environment, bounded_call
from module_launch import prepare_modules
from runtime_model import prepare_model, compose_world
from world_frame_evidence import initial_anchor
from scan_message import pack_scan
from motion import lidar_motion
sys.path.insert(0,str(ROOT/"Simulation/forest_lio/scripts"))
from common import POINT_DTYPE

def config(n=3):
    return {"vehicles":[{"uav_id":i,"instance":19+i,"initial":[0.,3.*(i-1),0.,0.],
                         "goals":[[1.,3.*(i-1),1.2]]} for i in range(1,n+1)]}

class FleetTests(unittest.TestCase):
    def test_cli_help_without_ros_environment(self):
        import subprocess,os
        env=dict(os.environ);env.pop("PYTHONPATH",None)
        result=subprocess.run([sys.executable,str(ROOT/"Simulation/px4_forest/scripts/run_swarm.py"),"--help"],
                              env=env,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn("--fleet",result.stdout)

    def test_unresponsive_service_cannot_block_shutdown(self):
        import threading,time
        event=threading.Event();start=time.monotonic()
        with self.assertRaises(TimeoutError):bounded_call(lambda:event.wait(2),.02)
        self.assertLess(time.monotonic()-start,.3)
        event.set()
        self.assertEqual(bounded_call(lambda:42,.2),42)

    def test_gazebo_load_does_not_scale_physics_once_per_aircraft(self):
        env={"PX4_SIM_SPEED_FACTOR":".25","ROS_HOME":"/run/ros_home"}
        neutral=gazebo_environment(env)
        self.assertEqual(neutral["PX4_SIM_SPEED_FACTOR"],"1")
        self.assertEqual(env["PX4_SIM_SPEED_FACTOR"],".25")
        self.assertEqual(neutral["ROS_HOME"],"/run/ros_home")

    def test_three_vehicles_have_unique_resources(self):
        fleet=load_fleet(config())
        self.assertEqual([v.system_id for v in fleet],[21,22,23])
        self.assertEqual([v.namespace for v in fleet],["/uav1","/uav2","/uav3"])
        ports=allocate_ports(fleet)
        numbers=[ports["ros"],ports["gazebo"]]+[p for v in ports["vehicles"].values() for p in v.values()]
        self.assertEqual(len(numbers),len(set(numbers)))
    def test_invalid_ids_and_placements_rejected(self):
        for change in ("duplicate","gap","instance","nan","z","overlap","goal"):
            data=config()
            if change=="duplicate":data["vehicles"][1]["uav_id"]=1
            if change=="gap":data["vehicles"][2]["uav_id"]=4
            if change=="instance":data["vehicles"][1]["instance"]=20
            if change=="nan":data["vehicles"][1]["initial"][0]=float("nan")
            if change=="z":data["vehicles"][1]["initial"][2]=1.
            if change=="overlap":data["vehicles"][1]["initial"]=[0,0,0,0]
            if change=="goal":data["vehicles"][1]["goals"]=[]
            with self.subTest(change=change), self.assertRaises(ValueError):load_fleet(data)
    def test_modules_are_namespaced_for_three_vehicles(self):
        with tempfile.TemporaryDirectory() as tmp:
            for v in load_fleet(config()):
                out=Path(tmp)/v.name;out.mkdir()
                info=prepare_modules(ROOT,out,[[*v.initial[:2],1.2],*v.goals],world_frame=True,
                                     uav_id=v.uav_id,swarm_num=3,initial=v.initial,
                                     sensor_prefix=v.sensor_prefix,trigger_topic=v.namespace+"/traj_start_trigger")
                wrapper=ET.parse(out/"modules.launch").getroot()
                planner=ET.parse(out/"planner_modules.launch").getroot()
                lio=next(e for e in wrapper.findall("include") if "mapping_mid360" in e.get("file"))
                args={e.get("name"):e.get("value") for e in lio.findall("arg")}
                self.assertEqual(args["uav_id"],str(v.uav_id))
                self.assertEqual(args["lidar_topic"],v.sensor_prefix+"/mid360/lidar")
                self.assertEqual(info["planner_odom"],v.namespace+"/world/odom")
                visualization=next(e for e in planner.findall("node") if e.get("type")=="odom_visualization")
                self.assertEqual(visualization.find("param[@name='tf45']").get("value"),"false")
                self.assertEqual(wrapper.find("group/param").get("name"),"output_root")
                height=wrapper.find("param[@name='/swarm_controller_uav_%d/Takeoff_height']"%v.uav_id)
                self.assertIsNotNone(height)
                self.assertEqual(float(height.get("value")),1.2)
                trigger=planner.find("remap[@from='/traj_start_trigger']")
                self.assertLess(list(planner).index(trigger),list(planner).index(planner.find("include")))
                self.assertTrue(any(e.get("name")=="/uav_%d_ego_planner_node/fsm/strict_waypoint_tracking"%v.uav_id for e in planner.findall("param")))
    def test_model_generation_uses_per_vehicle_startup_and_one_world(self):
        px4=ROOT/"artifacts/px4_forest/PX4-Autopilot"
        if not (px4/"Tools/sitl_gazebo/scripts/jinja_gen.py").exists():self.skipTest("local pinned PX4 missing")
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);models=[]
            for v in load_fleet(config()):
                directory=out/v.name;directory.mkdir()
                ports={"sim":24000+v.uav_id,"px4":24100+v.uav_id,"mavros":24200+v.uav_id,"unused":24300+v.uav_id}
                prepare_model(px4,directory,"/tmp/plugin.so",ports,v.instance,model_name=v.model_name,
                              pose=[*v.initial[:2],.25,0,0,v.initial[3]],sensor_prefix=v.sensor_prefix,physics_speed=.25 if v.uav_id==1 else None)
                models.append(directory/"iris.sdf")
                model=ET.parse(models[-1]).getroot().find("model")
                self.assertEqual(model.get("name"),v.model_name)
                rate=model.find("plugin[@name='mspace_forest_plant']/physics_update_rate")
                if v.uav_id==1:self.assertEqual(float(rate.text),62.5)
                else:self.assertIsNone(rate)
                self.assertEqual(model.find("plugin[@name='mspace_forest_plant']/sensor_prefix").text,v.sensor_prefix)
                self.assertIn(str(ports["sim"]),(directory/"etc/init.d-posix/px4-rc.simulator").read_text())
            compose_world(models,out/"forest.world")
            self.assertEqual(len(ET.parse(out/"forest.world").getroot().findall("world/model")),3)
    def test_nonzero_anchor_checks_configured_pose(self):
        rows=[[1,2,3,0,0,0,np.sin(.4),np.cos(.4)]]
        self.assertTrue(initial_anchor(rows,1,expected_position=[2,3,0],expected_yaw=.8)["passed"])
        self.assertFalse(initial_anchor(rows,1)["passed"])
    def test_wire_frame_is_unique(self):
        msg=pack_scan(4,1.2,np.zeros(2,dtype=POINT_DTYPE),frame_id="uav2/lidar")
        data=msg.wire_data
        size=struct.unpack_from("<I",data,12)[0]
        self.assertEqual(data[16:16+size],b"uav2/lidar")
        self.assertEqual(msg.header.frame_id,"uav2/lidar")
    def test_shared_origin_preserves_inter_vehicle_separation(self):
        rays=np.array([[1.,0,0]])
        h1=[[0,10,0,0,0,0,0,1],[1,10,0,0,0,0,0,1]]
        h2=[[0,10,3,0,0,0,0,1],[1,10,3,0,0,0,0,1]]
        o1,_=lidar_motion(h1,[.5],[10,0,0],[0,0,0],np.eye(3),rays)
        o2,_=lidar_motion(h2,[.5],[10,0,0],[0,0,0],np.eye(3),rays)
        np.testing.assert_allclose(o2-o1,[[0,3,0]])

if __name__=="__main__":unittest.main()
