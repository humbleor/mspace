#!/usr/bin/env python3
"""Isolated PX4 SITL -> Iris dynamics -> forest lidar/IMU -> LIO/EGO -> MAVROS."""
import argparse,json,os,socket,subprocess,sys,time,threading,traceback,shutil
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"Simulation/forest_lio/scripts"))
from common import POINT_DTYPE,save_trajectory,trajectory_comparison,write_json,summary
from run_replay import stop
from renderer import read_pcd
from bvh_renderer import BVHRenderer
from generate_mid360_bag import templates_from_bag
from scan_message import pack_scan
from scene import to_local,to_world,footprint_evidence
from coverage_map import CoverageMap
from runtime_model import prepare_model
from motion import lidar_motion
from waypoints import load_wps
from module_launch import prepare_modules
from collision_evidence import collision_acceptance,ego_acceptance,validate_parameters
from forest_geometry import extract_geometry,geometry_model
from contact_evidence import contact_records,contact_acceptance

class TreeContactDetected(RuntimeError):pass

def port(socktype=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET,socktype) as s:s.bind(("127.0.0.1",0));return s.getsockname()[1]
def values(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation
    return [m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w]
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stage",choices=["hover","forest"],default="forest")
    ap.add_argument("--px4",type=Path,default=ROOT/"artifacts/px4_forest/PX4-Autopilot")
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--scene",type=Path)
    ap.add_argument("--template-bag",type=Path,default=Path("/home/mspace/bagfiles/yxb_20260208-104116.bag"))
    ap.add_argument("--duration",type=float,default=100)
    ap.add_argument("--takeoff-height",type=float,default=1.5)
    ap.add_argument("--speed",type=float,default=.25)
    ap.add_argument("--surface-model",choices=["fixed","adaptive"],default="fixed")
    ap.add_argument("--body-radius",type=float,default=.4,help="Proxy mode only: conservative body/rotor sphere radius in metres")
    ap.add_argument("--collision-model",choices=["ego","proxy"],default="ego",help="Default EGO recorded inflation clearance; proxy enables independent body sphere")
    ap.add_argument("--obstacle-radius",type=float,default=.2,help="PCD obstacle proxy radius; at least 0.2 m to enclose sensor surfels")
    ap.add_argument("--max-truth-gap",type=float,default=.05,help="Maximum physical truth interval for collision acceptance")
    ap.add_argument("--forest-geometry",type=Path,help="Pre-extracted geometry.json matching this local PCD hash")
    ap.add_argument("--no-tree-collision",action="store_true",help="Explicit legacy comparison: disable physical tree collision")
    ap.add_argument("--keep-contact-open",action="store_true",help="Pause on tree contact and preserve active GUI windows")
    ap.add_argument("--instance",type=int,default=20)
    ap.add_argument("--waypoints",type=Path,help="generateWps grid JSON: x/y/z min,max,step; absolute local world heights")
    ap.add_argument("--goals",type=int,help="Limit route goals after takeoff for initial validation")
    ap.add_argument("--gui",choices=["both","gazebo","rviz","none"],default="both",help="Default: open Gazebo and RViz on this run's private masters")
    ap.add_argument("--headless",dest="gui",action="store_const",const="none",help="Run without visualization windows")
    ap.add_argument("--keep-open",action="store_true",help="Pause after a successful mission for inspection; close windows or Ctrl+C to exit")
    a=ap.parse_args()
    try:validate_parameters(a.body_radius,a.obstacle_radius,a.max_truth_gap)
    except ValueError as exc:ap.error(str(exc))
    if a.collision_model=="proxy" and a.obstacle_radius<.2:ap.error("--obstacle-radius must enclose the 0.2 m sensor surfel support")
    if a.stage=="forest" and not a.scene:ap.error("forest requires --scene")
    if a.waypoints and a.stage!="forest":ap.error("--waypoints requires forest stage")
    if a.goals is not None and a.goals<1:ap.error("--goals must be positive")
    if a.waypoints and a.goals:ap.error("Native generateWps executes the full grid; omit --goals")
    if a.duration<=10 or not 0<a.speed<=1 or a.takeoff_height<=.5 or not 0<=a.instance<254:ap.error("Invalid limits")
    if a.gui!="none" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        ap.error("No graphical display detected; enable WSLg/X11 or explicitly use --headless")
    for program in (["gzclient"] if a.gui=="gazebo" else ["rviz"] if a.gui=="rviz" else ["gzclient","rviz"] if a.gui=="both" else []):
        if not shutil.which(program):ap.error(program+" is not installed")
    out=a.output.resolve();px4=a.px4.resolve()
    if out.exists() and any(out.iterdir()):ap.error("output must be empty")
    binary=px4/"build/px4_sitl_default/bin/px4"
    plugin=ROOT/"artifacts/px4_forest/plant_build/libmspace_forest_plant.so"
    if not binary.exists() or not plugin.exists():raise ValueError("Build isolated PX4 and plant plugin first")
    for name in ("Log","PCD","ros_home"): (out/name).mkdir(parents=True,exist_ok=True)
    scene=json.loads(a.scene.read_text()) if a.scene else {}
    world_R=np.asarray(scene.get("rotation_world_from_local",np.eye(3)))
    map_origin=np.asarray(scene.get("origin_world",[0,0,0]),dtype=float)
    if a.stage=="forest":map_origin[2]-=a.takeoff_height
    goals=np.array([[0,0,1.2 if a.stage=="forest" else a.takeoff_height]],dtype=float)
    module_info={}
    renderer=None;templates=None;coverage=None;T=E=None
    geometry=None
    if a.stage=="forest":
        waypoint_config=None
        if a.waypoints:
            waypoint_config,route=load_wps(a.waypoints)
            write_json(out/"waypoint_config.json",waypoint_config)
        else:
            route=np.asarray(scene["goals_local"],dtype=float).copy()
            route[:,2]+=a.takeoff_height
        if a.goals:route=route[:a.goals]
        goals=np.vstack([goals,route])
        write_json(out/"route_goals.json",{"frame":"world","height_origin":"initial body position","generator":"generateWps" if a.waypoints else "scene","goals":goals})
        local_map=to_local(read_pcd(scene["map"]),map_origin,world_R)
        renderer=BVHRenderer(local_map,threads=4,surface_model=a.surface_model)
        if not a.no_tree_collision:
            geometry=json.loads(a.forest_geometry.read_text()) if a.forest_geometry else extract_geometry(local_map)
            import hashlib
            digest=hashlib.sha256(np.ascontiguousarray(local_map,dtype="<f8").tobytes()).hexdigest()
            if geometry.get("local_map_sha256")!=digest or geometry.get("frame")!="world":
                raise ValueError("Forest collision geometry does not match local map/frame")
            if not geometry.get("colliders"):raise ValueError("No supported forest collision shapes")
            write_json(out/"forest_geometry.json",geometry)
        if a.gui in ("both","rviz"):
            # Display-only thinning; the sensor BVH retains the complete map.
            np.save(out/"visual_map.npy",local_map[::max(1,int(np.ceil(len(local_map)/200000)))].astype("<f4"))
        templates=templates_from_bag(a.template_bag,"/livox/lidar_192_168_2_181",24,.1,window=(40,120))
        cfg=yaml.safe_load((ROOT/"Modules/fast_lio2/config/mid360.yaml").read_text())
        T=np.asarray(cfg["mapping"]["extrinsic_T"]);E=np.asarray(cfg["mapping"]["extrinsic_R"]).reshape(3,3)
        coverage=CoverageMap.load(scene["coverage"])
    if a.stage=="forest":
        module_info=prepare_modules(ROOT,out,goals,waypoint_config)
        write_json(out/"module_alignment.json",module_info)
    ports={key:port(kind) for key,kind in [("ros",socket.SOCK_STREAM),("gazebo",socket.SOCK_STREAM),("sim",socket.SOCK_STREAM),("px4",socket.SOCK_DGRAM),("mavros",socket.SOCK_DGRAM),("unused",socket.SOCK_DGRAM)]}
    gazebo=prepare_model(px4,out,plugin,ports,a.instance,gps=a.stage=="hover",speed=a.speed)
    env=os.environ.copy();env["PATH"]=":".join(p for p in env["PATH"].split(":") if not p.startswith("/mnt/"))
    env.update(ROS_MASTER_URI="http://127.0.0.1:%d"%ports["ros"],ROS_IP="127.0.0.1",ROS_HOME=str(out/"ros_home"),
        GAZEBO_MASTER_URI="http://127.0.0.1:%d"%ports["gazebo"],GAZEBO_MODEL_PATH=str(gazebo/"models")+":/usr/share/gazebo-11/models",
        GAZEBO_PLUGIN_PATH=str(px4/"build/px4_sitl_default/build_gazebo")+":/opt/ros/noetic/lib",
        PX4_SIM_MODEL="iris",PX4_ESTIMATOR="ekf2",PX4_SIM_SPEED_FACTOR=str(a.speed),PX4_SIM_HOST_ADDR="127.0.0.1")
    env["PATH"]=str(binary.parent)+":"+env["PATH"];env.pop("ROS_HOSTNAME",None)
    os.environ.pop("ROS_HOSTNAME",None);os.environ.update(env)
    write_json(out/"manifest.json",{"parameters":{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
       "ports":ports,"gps_enabled":a.stage=="hover","gazebo_external_odometry_enabled":False,
       "map_origin_world":map_origin,"rotation_world_from_local":world_R,"goals_local":goals,
       "px4_commit":subprocess.check_output(["git","-C",str(px4),"rev-parse","HEAD"],text=True).strip()})
    procs=[];required=[];windows=[];ui_errors=[];handles=[];truth=[];lio=[];fcu=[];imu_rows=[];frames=[];render=[];pipeline=[];commands=[];arrivals=[];state_history=[]
    lock=threading.Lock();high=[];state=None;cmd=None;last_vision=-1;initial=None;active=-1;last_goal=-1;finished=None
    accepted_goals=goals.copy();takeoff_sent=False;trigger_sent=False;station_id=0
    started=time.monotonic();success=False;errors=[];param_checks={};graph={};armed_at=None;frame=0;next_end=None;start_sim=None
    collision={"status":"not_applicable","passed":False,"reason":"Hover has no forest geometry"}
    flight_end=None
    grid_params=None
    process_logs={}
    tree_enabled=a.stage=="forest" and not a.no_tree_collision
    contact_rows=[];contact_frames=0;contact_times=[];contact_event=threading.Event()
    def assess_collision(rows):
        if a.collision_model=="ego":
            if grid_params is None:raise ValueError("Actual EGO grid_map parameters unavailable")
            return ego_acceptance(local_map,rows,grid_params,a.max_truth_gap,tree=renderer.tree)
        return collision_acceptance(local_map,rows,a.body_radius,a.obstacle_radius,a.max_truth_gap,tree=renderer.tree)
    try:
        def launch(args,name,cwd=out,critical=True):
            h=open(out/name,"w");handles.append(h)
            p=subprocess.Popen([str(x) for x in args],cwd=cwd,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
            procs.append(p)
            process_logs[p.pid]=name
            if critical:required.append(p)
            return p
        def check_required(phase):
            for p in required:
                code=p.poll()
                if code is not None:
                    name=process_logs.get(p.pid,"unknown")
                    raise RuntimeError("Required process exited %s: %s (pid %s, exit %s); log: %s" %
                                       (phase,name,p.pid,code,out/name))
        master=launch(["roscore","-p",ports["ros"]],"roscore.log")
        import xmlrpc.client
        proxy=xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        deadline=time.monotonic()+30
        while True:
            try:
                if proxy.getPid("/forest_sitl")[0]==1:break
            except OSError:pass
            if master.poll() is not None or time.monotonic()>deadline:raise RuntimeError("ROS master startup failed")
            time.sleep(.1)
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import PoseStamped
        from quadrotor_msgs.msg import PositionCommand
        from livox_ros_driver2.msg import CustomMsg
        from mavros_msgs.msg import State,PositionTarget
        from mavros_msgs.srv import SetMode,CommandBool,ParamGet
        from std_msgs.msg import Header
        from prometheus_msgs.msg import SwarmCommand
        from visualization_msgs.msg import Marker
        from gazebo_msgs.msg import ContactsState
        rospy.set_param("/use_sim_time",True);rospy.init_node("forest_sitl",disable_signals=True)
        mavros_topic="/uav1/mavros" if a.stage=="forest" else "/mavros"
        lio_topic="/uav1/drone_Odometry"
        high_topic="/uav1/drone_Odom_high_freq"
        traj_topic="/uav1/planning/ego/traj_cmd"
        setpoint_pub=rospy.Publisher(mavros_topic+"/setpoint_raw/local",PositionTarget,queue_size=10) if a.stage=="hover" else None
        station_pub=rospy.Publisher("/uav1/prometheus/swarm_command",SwarmCommand,queue_size=10) if a.stage=="forest" else None
        trigger_pub=rospy.Publisher("/traj_start_trigger",PoseStamped,queue_size=1,latch=True) if a.stage=="forest" else None
        def station_command(mode,yaw=0):
            nonlocal station_id
            station_id+=1
            m=SwarmCommand(header=Header(stamp=rospy.Time.now()),Mode=mode,Command_ID=station_id,source="simulation_station",yaw_ref=yaw)
            station_pub.publish(m)
        def accepted_goal_cb(m):
            # Observe the native module's yellow relocation markers; never
            # choose or modify a navigation goal from the simulator.
            i=m.id-100+1
            if m.action==Marker.ADD and 1<=i<len(accepted_goals):
                accepted_goals[i]=[m.pose.position.x,m.pose.position.y,m.pose.position.z]

        lidar_pub=rospy.Publisher("/sim/mid360/lidar",CustomMsg,queue_size=10)
        def plant_cb(m):
            nonlocal initial
            row=values(m)
            with lock:truth.append(row)
        def state_cb(m):
            nonlocal state
            state=m;state_history.append([rospy.Time.now().to_sec(),m.connected,m.armed,m.mode])
        def command_cb(m):
            nonlocal cmd
            cmd=m;commands.append([m.header.stamp.to_sec(),m.position.x,m.position.y,m.position.z])
        def high_cb(m):
            high.append(values(m))
        def vision_cb(m):
            nonlocal last_vision
            last_vision=m.header.stamp.to_sec()
        def contact_cb(m):
            nonlocal contact_frames
            rows=contact_records(m)
            with lock:
                contact_frames+=1
                contact_times.append(m.header.stamp.to_sec())
                contact_rows.extend(rows)
            if rows:contact_event.set()
        rospy.Subscriber("/sim/plant/odom",Odometry,plant_cb,queue_size=5000)
        if tree_enabled:rospy.Subscriber("/sim/plant/tree_contacts",ContactsState,contact_cb,queue_size=1000)
        rospy.Subscriber(mavros_topic+"/state",State,state_cb,queue_size=100)
        rospy.Subscriber(mavros_topic+"/local_position/odom",Odometry,lambda m:fcu.append(values(m)),queue_size=1000)
        rospy.Subscriber(lio_topic,Odometry,lambda m:lio.append(values(m)),queue_size=1000)
        rospy.Subscriber(high_topic,Odometry,high_cb,queue_size=1000)
        rospy.Subscriber(traj_topic,PositionCommand,command_cb,queue_size=10)
        rospy.Subscriber("/sim/mid360/imu",Imu,lambda m:imu_rows.append([m.header.stamp.to_sec(),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]),queue_size=5000)
        if a.stage=="forest":
            rospy.Subscriber("/uav_1_ego_planner_node/goal_point",Marker,accepted_goal_cb,queue_size=100)
            rospy.Subscriber(mavros_topic+"/vision_pose/pose",PoseStamped,vision_cb,queue_size=1)
        flight=launch([binary,"-i",a.instance,"-d","-s","etc/init.d-posix/rcS"],"px4.log")
        physics=launch(["gzserver","--verbose",out/"forest.world","-s","/opt/ros/noetic/lib/libgazebo_ros_api_plugin.so"],"gazebo.log")
        # MAVROS measures connection loss in wall time; PX4 heartbeats follow
        # the slowed simulation clock. Keep a finite, speed-aware watchdog.
        import rospkg
        mavros_share=Path(rospkg.RosPack().get_path("mavros"))
        mavros_config=yaml.safe_load((ROOT/"Experiment/mavros/launch/px4_config.yaml").read_text())
        mavros_config["conn"]["timeout"]=max(10.0,3.0/a.speed)
        (out/"mavros_config.yaml").write_text(yaml.safe_dump(mavros_config))
        import xml.etree.ElementTree as ET
        mavros_launch=ET.Element("launch")
        mavros_group=ET.SubElement(mavros_launch,"group",ns="uav1") if a.stage=="forest" else mavros_launch
        include=ET.SubElement(mavros_group,"include",file=str(mavros_share/"launch/node.launch"))
        for key,value in {
            "pluginlists_yaml":str(ROOT/"Experiment/mavros/launch/px4_pluginlists.yaml"),
            "config_yaml":str(out/"mavros_config.yaml"),"gcs_url":"","tgt_component":"1",
            "fcu_url":"udp://127.0.0.1:%d@127.0.0.1:%d"%(ports["mavros"],ports["px4"]),
            "tgt_system":str(a.instance+1)}.items():
            ET.SubElement(include,"arg",name=key,value=value)
        ET.ElementTree(mavros_launch).write(out/"mavros.launch",encoding="unicode")
        bridge=launch(["roslaunch",out/"mavros.launch"],"mavros.log")
        deadline=time.monotonic()+80
        while not truth or truth[-1][0]<2 or state is None or not state.connected:
            check_required("during startup")
            if time.monotonic()>deadline:raise RuntimeError("Physics/PX4/MAVROS startup timeout")
            time.sleep(.05)
        with lock:initial=np.asarray(truth[-1][1:4])
        if tree_enabled:
            # Fit in navigation-local coordinates, spawn only after the actual
            # settled physical origin is known. Persist the exact same world.
            from gazebo_msgs.srv import SpawnModel
            from geometry_msgs.msg import Pose
            root=ET.Element("sdf",version="1.6");model=geometry_model(geometry,initial);root.append(model)
            xml=ET.tostring(root,encoding="unicode")
            (out/"forest_collisions.sdf").write_text(xml)
            world=ET.parse(out/"forest.world");world.getroot().find("world").append(model)
            world.write(out/"forest.world",encoding="unicode")
            rospy.wait_for_service("/gazebo/spawn_sdf_model",timeout=30)
            pose=Pose();pose.orientation.w=1
            spawned=rospy.ServiceProxy("/gazebo/spawn_sdf_model",SpawnModel)("forest_collision_geometry",xml,"",pose,"world")
            if not spawned.success:raise RuntimeError("Forest geometry spawn failed: "+spawned.status_message)
            from gazebo_msgs.srv import GetModelState
            deadline=time.monotonic()+10
            while True:
                actual=rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)("forest_collision_geometry","world")
                if actual.success:break
                if time.monotonic()>deadline:raise RuntimeError("Spawned forest geometry not visible in physics")
                time.sleep(.02)
            physical_offset=np.array([actual.pose.position.x,actual.pose.position.y,actual.pose.position.z])
            if np.linalg.norm(physical_offset-initial)>1e-6:raise RuntimeError("Forest collision coordinate alignment failed")
            write_json(out/"forest_geometry_alignment.json",{"frame":"world","gazebo_offset":initial,"counts":geometry["counts"],
                       "verified_physical_offset":physical_offset,
                       "method":"PCD map to local world, then actual settled Gazebo initial translation"})
        if a.stage=="forest":
            mapping=launch(["roslaunch",out/"modules.launch"],"navigation.log")
        if a.gui in ("both","rviz"):
            write_json(out/"visual_scene.json",{"initial":initial,"goals":goals,"lio_topic":high_topic,"fcu_topic":mavros_topic+"/local_position/odom","state_topic":mavros_topic+"/state"})
            launch([sys.executable,ROOT/"Simulation/px4_forest/scripts/visualize_sitl.py",out],"visualization.log")
            windows.append(("rviz",launch(["rviz","-d",ROOT/"Simulation/px4_forest/config/forest.rviz"],"rviz.log",critical=False)))
        if a.gui in ("both","gazebo"):
            windows.append(("gazebo",launch(["gzclient"],"gazebo_gui.log",critical=False)))
        start_sim=rospy.Time.now().to_sec();next_end=start_sim+.2;last_setpoint=-1;last_service=-10;last_print=-10
        set_mode=rospy.ServiceProxy(mavros_topic+"/set_mode",SetMode);arm=rospy.ServiceProxy(mavros_topic+"/cmd/arming",CommandBool)
        get_param=rospy.ServiceProxy(mavros_topic+"/param/get",ParamGet)
        deadline=time.monotonic()+a.duration/a.speed+180
        while rospy.Time.now().to_sec()-start_sim<a.duration:
            if contact_event.is_set():raise TreeContactDetected("Physical Iris/tree contact detected")
            if tree_enabled and rospy.Time.now().to_sec()-start_sim>8 and not contact_frames:
                raise RuntimeError("Tree contact observer unavailable; rebuild forest plant plugin")
            check_required("during flight")
            if time.monotonic()>deadline:raise RuntimeError("SITL wall-clock timeout")
            for name,p in windows:
                if p.poll() not in (None,0) and name not in ui_errors:
                    ui_errors.append(name);print("Visualization failed: "+name+"; inspect GUI log",flush=True)
            now=rospy.Time.now().to_sec()
            if a.stage=="forest" and truth[-1][0]>=next_end:
                with lock:history=np.asarray(truth[-300:])
                if next_end-.1<history[0,0]:raise RuntimeError("Sensor history underflow")
                if now-next_end>.4:raise RuntimeError("PCD sensor fell behind physics by >0.4 s")
                rays,offsets,lines,refl=templates[frame%len(templates)]
                times=next_end-.1+offsets*1e-9
                cycle_began=time.monotonic()
                origins,directions=lidar_motion(history,times,initial,T,E,rays)
                interpolation_s=time.monotonic()-cycle_began
                began=time.monotonic();ranges=renderer.scan_world(directions,origins,.5);render.append(time.monotonic()-began)
                points=np.zeros(len(rays),dtype=POINT_DTYPE);points["offset_time"]=offsets;points["xyz"]=rays*ranges[:,None];points["line"]=lines;points["reflectivity"]=np.where(ranges>0,refl,0)
                transport_began=time.monotonic()
                lidar_pub.publish(pack_scan(frame,next_end-.1,points))
                pipeline.append([next_end,interpolation_s,render[-1],time.monotonic()-transport_began,time.monotonic()-cycle_began,rospy.Time.now().to_sec()-next_end])
                frames.append([next_end,int((ranges>0).sum()),len(rays)]);frame+=1;next_end+=.1
            if a.stage=="hover" and now-last_setpoint>=.018:
                target=PositionTarget(header=Header(stamp=rospy.Time.now(),frame_id="world"))
                target.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                target.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
                if a.stage=="hover":
                    target.position.z=a.takeoff_height
                setpoint_pub.publish(target);last_setpoint=now
            ready=now-start_sim>8 and (a.stage=="hover" or len(lio)>25)
            if a.stage=="hover" and ready and time.monotonic()-last_service>1:
                if state.mode!="OFFBOARD":set_mode(custom_mode="OFFBOARD")
                elif not state.armed:arm(value=True)
                last_service=time.monotonic()
            if a.stage=="forest" and ready and not trigger_sent:
                # Prestream the native controller's valid takeoff target before
                # ground-station mode/arm requests. Its legacy Idle (0x4000)
                # packet enables zero fields on PX4 1.13 and can falsely clear
                # landed before the actual takeoff request reaches the FCU.
                if not takeoff_sent and station_pub.get_num_connections():
                    station_command(SwarmCommand.Takeoff);takeoff_sent=True
                    last_service=time.monotonic()
                elif takeoff_sent and time.monotonic()-last_service>1:
                    if state.mode!="OFFBOARD":set_mode(custom_mode="OFFBOARD")
                    elif not state.armed:arm(value=True)
                    last_service=time.monotonic()
            if state.armed and state.mode=="OFFBOARD":
                if armed_at is None:armed_at=now
                if a.stage=="hover":
                    if fcu and abs(fcu[-1][3]-a.takeoff_height)<.2 and now-armed_at>6:
                        if finished is None:finished=now
                elif lio:
                    if not trigger_sent and takeoff_sent and now-armed_at>4 and abs(lio[-1][3]-1.2)<.2 and trigger_pub.get_num_connections():
                        arrivals.append([0,now])
                        trigger_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id="world")))
                        trigger_sent=True
                        print("Native EGO task triggered; station yields to module control.",flush=True)
                    if trigger_sent:
                        index=len(arrivals)
                        if index<len(goals) and np.linalg.norm(np.asarray(lio[-1][1:4])-accepted_goals[index])<.5:
                            arrivals.append([index,now])
            if a.stage=="forest" and len(arrivals)==len(goals) and finished is None and lio and np.linalg.norm(np.asarray(lio[-1][1:4])-accepted_goals[-1])<.2:
                finished=now
            if now-last_print>5:
                print("SITL %.1fs mode=%s armed=%s lidar=%d goals=%d/%d"%(now-start_sim,state.mode,state.armed,frame,len(arrivals),len(goals)),flush=True);last_print=now
            if finished is not None:
                # A goal crossing is not a settled arrival with a physical plant.
                estimate = np.asarray(lio[-1][1:4]) if a.stage=="forest" and lio else np.asarray(fcu[-1][1:4]) if fcu else None
                if estimate is None or np.linalg.norm(estimate-accepted_goals[-1])>.2:
                    finished=None
                elif now-finished>3:break
            time.sleep(.003)
        for name in ("EKF2_AID_MASK","EKF2_HGT_MODE","COM_ARM_WO_GPS","COM_RCL_EXCEPT","COM_OF_LOSS_T"):
            response=get_param(param_id=name);param_checks[name]=response.value.real if name=="COM_OF_LOSS_T" else response.value.integer
        if param_checks["COM_OF_LOSS_T"]<=0:raise RuntimeError("OFFBOARD watchdog was truncated to zero")
        status=proxy.getSystemState("/forest_sitl")[2]
        graph={"publishers":dict(status[0]),"subscribers":dict(status[1]),"protected_truth_subscribers_in_navigation":[(t,n) for t,nodes in status[1] for n in nodes if t=="/sim/plant/odom" and ("mapping" in n.lower() or "ego" in n or "swarm" in n or n.endswith("/mavros"))]}
        if a.stage=="forest":
            expected={mavros_topic+"/setpoint_raw/local":"/swarm_controller_uav_1",
                      mavros_topic+"/vision_pose/pose":"/swarm_estimator_uav_1",
                      traj_topic:"/uav1_traj_server"}
            graph["module_publishers_valid"]=all(graph["publishers"].get(t)==[n] for t,n in expected.items())
            if not graph["module_publishers_valid"]:raise RuntimeError("Native module publisher ownership audit failed")
        if graph["protected_truth_subscribers_in_navigation"]:raise RuntimeError("Truth leaked into navigation")
        if a.stage=="forest":
            write_json(out/"module_params.json",{k:rospy.get_param(k) for k in ("/uav_1_ego_planner_node","/swarm_controller_uav_1","/swarm_estimator_uav_1","/ego_traj_to_cmd_uav_1","/mapping","/preprocess","/common") if rospy.has_param(k)})
        with lock:last_truth=np.asarray(truth[-1][1:4])-initial
        success=finished is not None and np.linalg.norm(last_truth-accepted_goals[-1])<.3 and state.armed and state.mode=="OFFBOARD"
        if a.stage=="forest" and (param_checks["EKF2_AID_MASK"]!=24 or param_checks["EKF2_HGT_MODE"]!=3):raise RuntimeError("GNSS-denied EKF parameters not active")
        if not success:errors.append("Mission did not settle within the goal tolerance while armed in OFFBOARD")
        if a.stage=="forest":
            # Stop time before potentially expensive independent acceptance. No
            # truth-based feedback enters navigation or sensor timing.
            from std_srvs.srv import Empty
            rospy.ServiceProxy("/gazebo/pause_physics",Empty)()
            with lock:
                flight_end=truth[-1][0]
                collision_truth=np.asarray(truth).copy()
            collision_truth[:,1:4]-=initial
            grid_params=rospy.get_param("/uav_1_ego_planner_node/grid_map")
            collision=assess_collision(collision_truth)
            write_json(out/"collision_evidence.json",collision)
            if not collision["passed"]:
                success=False
                errors.append("Forest collision acceptance failed: "+collision["status"])
        if tree_enabled and contact_event.is_set():raise TreeContactDetected("Physical Iris/tree contact detected during final audit")
        if a.keep_open and success and any(p.poll() is None for _,p in windows):
            from std_srvs.srv import Empty
            rospy.ServiceProxy("/gazebo/pause_physics",Empty)()
            print("Mission passed; physics paused for inspection. Close GUI windows or press Ctrl+C to finish.",flush=True)
            try:
                while any(p.poll() is None for _,p in windows):time.sleep(.2)
            except KeyboardInterrupt:pass
    except TreeContactDetected as exc:
        errors.append(str(exc));success=False
        from std_srvs.srv import Empty
        try:
            rospy.ServiceProxy("/gazebo/pause_physics",Empty)()
            with lock:flight_end=max(truth[-1][0] if truth else 0,max((r["sim_time_s"] for r in contact_rows),default=0))
            if a.keep_contact_open and any(p.poll() is None for _,p in windows):
                print("Tree contact: physics paused, task FAILED. Close windows or Ctrl+C to save evidence.",flush=True)
                try:
                    while any(p.poll() is None for _,p in windows):time.sleep(.2)
                except KeyboardInterrupt:pass
        except Exception as pause_error:errors.append("Contact pause failed: "+str(pause_error))
    except KeyboardInterrupt:
        errors.append("Run interrupted")
    except Exception as exc:
        errors.append(type(exc).__name__+": "+str(exc));traceback.print_exc()
    finally:
        if a.stage=="forest" and grid_params is None and "rospy" in locals():
            try:grid_params=rospy.get_param("/uav_1_ego_planner_node/grid_map")
            except Exception:pass
        flight_end = flight_end if flight_end is not None else truth[-1][0] if truth else None
        # Only processes created by this runner; Gazebo continues clock while navigation exits.
        for p in reversed(procs):stop(p,timeout=8)
        if "rospy" in locals():rospy.signal_shutdown("SITL complete")
        for h in handles:h.close()
        save_trajectory(out/"plant_world.tum",truth)
        mission_truth=[r for r in truth if flight_end is not None and r[0]<=flight_end]
        raw=np.asarray(mission_truth).reshape(-1,8)
        lio=[r for r in lio if flight_end is not None and r[0]<=flight_end]
        fcu=[r for r in fcu if flight_end is not None and r[0]<=flight_end]
        high=[r for r in high if flight_end is not None and r[0]<=flight_end]
        imu_rows=[r for r in imu_rows if flight_end is not None and r[0]<=flight_end]
        local=raw.copy()
        if initial is not None and len(local):local[:,1:4]-=initial
        if a.stage=="forest" and collision["status"]=="not_applicable":
            try:
                if initial is None:raise ValueError("No initial physical origin")
                collision=assess_collision(local)
            except Exception as exc:
                collision={"status":"unavailable","passed":False,"reason":str(exc)}
            if not collision["passed"]:
                success=False
                errors.append("Forest collision acceptance failed: "+collision["status"])
        if a.stage=="forest":write_json(out/"collision_evidence.json",collision)
        contacts=[r for r in contact_rows if flight_end is not None and r["sim_time_s"]<=flight_end]
        tree_contact=contact_acceptance(contacts,contact_frames,tree_enabled,contact_times,
                                       (float(raw[0,0]),flight_end) if len(raw) else None)
        write_json(out/"tree_contact_evidence.json",tree_contact)
        write_json(out/"tree_contacts.json",contacts)
        if tree_enabled and not tree_contact["passed"]:
            success=False
            errors.append("Physical tree contact acceptance failed: "+tree_contact["status"])
        save_trajectory(out/"truth.tum",local);est=save_trajectory(out/"lio.tum",lio);save_trajectory(out/"fcu.tum",fcu)
        save_trajectory(out/"lio_high_freq.tum",high)
        evidence=None
        if coverage is not None and len(local):
            sampled=local[::25];world=to_world(sampled[:,1:4],map_origin,world_R)
            rows,evidence=footprint_evidence(coverage,world,.25)
            np.savetxt(out/"footprint_evidence.csv",np.column_stack([sampled[:,0],rows]),delimiter=",",header="stamp,center_unknown,cells,unknown,endpoint")
        np.savetxt(out/"imu.csv",np.asarray(imu_rows).reshape(-1,7),delimiter=",",header="stamp,ax_g,ay_g,az_g,gx,gy,gz")
        np.savetxt(out/"sensor_timing.csv",np.asarray(pipeline).reshape(-1,6),delimiter=",",header="scan_end,interpolation_wall_s,render_wall_s,transport_wall_s,total_wall_s,published_lag_sim_s")
        np.savetxt(out/"frames.csv",np.asarray(frames).reshape(-1,3),delimiter=",",header="scan_end,hits,rays")
        np.savetxt(out/"commands.csv",np.asarray(commands).reshape(-1,4),delimiter=",",header="stamp,x,y,z")
        write_json(out/"graph.json",graph)
        write_json(out/"result.json",{"status":"ok" if success and not errors else "failed","errors":errors,"stage":a.stage,
          "collision":collision,
          "tree_contact":tree_contact,"forest_geometry_counts":geometry["counts"] if geometry else {},
          "visualization":{"mode":a.gui,"failed_windows":ui_errors},"flight_end_sim_s":flight_end,"goals":goals,"accepted_goals":accepted_goals,"module_alignment":module_info,"native_trigger_sent":trigger_sent,"arrivals":arrivals,"armed_at_sim_s":armed_at,"state_history":state_history,"gps_enabled":a.stage=="hover",
          "parameter_checks":param_checks,"initial_gazebo_position":initial,"final_position":local[-1,1:4] if len(local) else None,
          "goal_error_m":float(np.linalg.norm(local[-1,1:4]-accepted_goals[-1])) if len(local) else None,
          "goal_min_lio_distance_m":[float(np.min(np.linalg.norm(est[:,1:4]-g,axis=1))) for g in accepted_goals] if len(est) else [],
          "requested_goal_min_lio_distance_m":[float(np.min(np.linalg.norm(est[:,1:4]-g,axis=1))) for g in goals] if len(est) else [],
          "lio_frames":len(est),"lidar_frames":frame,"high_freq_poses":len(high),"vision_last_stamp":last_vision,
          "render_wall_s":summary(render),"sensor_pipeline":{key:summary(np.asarray(pipeline)[:,i]) for i,key in enumerate(["scan_end","interpolation_wall_s","render_wall_s","transport_wall_s","total_wall_s","published_lag_sim_s"]) if i and pipeline},"wall_s":time.monotonic()-started,"footprint_evidence":evidence,
          "comparison":trajectory_comparison(est,local) if len(est)>1 and len(local)>1 else {},
          "limits":["PX4 software flight stack on computer; no physical flight controller.",
                    "Gazebo Iris dynamics with fitted static trunk/coarse-branch shapes; unmodeled forest remains." if tree_enabled else "Physical forest collision explicitly disabled or not applicable.",
                    "ROS LIO IMU from actual physics specific force in g; PX4 uses standard simulated IMU.",
                    "LIO/world origin based on stable initial body position; truth used only for sensor rendering and evaluation.",
                    "This isolated single-UAV test does not validate swarm flight or measured sensor realism."]})
        print(out/"result.json",flush=True)
    if not success or errors:raise SystemExit("SITL validation failed")
if __name__=="__main__":main()
