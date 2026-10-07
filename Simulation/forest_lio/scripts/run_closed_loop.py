#!/usr/bin/env python3
"""Online PCD -> Mid-360/IMU -> Fast-LIO2 -> EGO -> second-order motion loop."""
import argparse
import os
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from renderer import PointRenderer, read_pcd
from bvh_renderer import BVHRenderer
from generate_mid360_bag import templates_from_bag, pack_scan
from common import POINT_DTYPE, save_trajectory, trajectory_comparison, write_json, summary
from run_replay import stop
from scene import to_local,to_world,footprint_evidence,planner_config_for_route
import json

def tracking_acceleration(position, velocity, target, target_velocity, feedforward, limit=1.5):
    a=feedforward+4*(target-position)+3*(target_velocity-velocity)
    return a*min(1.,limit/max(np.linalg.norm(a),1e-12))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map",type=Path,required=True)
    ap.add_argument("--template-bag",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--duration",type=float,default=15)
    ap.add_argument("--scene",type=Path,help="Prepared yaw-aligned local frame and estimated-route goals")
    ap.add_argument("--body-radius",type=float,default=.25)
    ap.add_argument("--goal",type=float,nargs=3,default=[-.7,0,.5])
    ap.add_argument("--sensor-model",type=Path)
    ap.add_argument("--noise",action="store_true")
    ap.add_argument("--seed",type=int,default=42)
    ap.add_argument("--renderer",choices=["bvh","conservative","full","cached"],default="bvh",help="cached is experimental; moving visibility differs from full raster")
    ap.add_argument("--render-threads",type=int,default=4)
    ap.add_argument("--surface-model",choices=["fixed","adaptive"],default="fixed")
    ap.add_argument("--coverage",type=Path,help="Optional original-world coverage evidence for evaluation only")
    args=ap.parse_args()
    if args.surface_model=="adaptive" and args.renderer!="bvh":ap.error("Adaptive surfaces require BVH")
    if args.render_threads<1:ap.error("render-threads must be positive")
    if args.body_radius<=0 or not np.isfinite(args.body_radius):ap.error("body-radius must be positive")
    if args.duration<=5: ap.error("duration must exceed 5 seconds")
    if args.noise and not args.sensor_model: ap.error("--noise requires --sensor-model")
    out=args.output.resolve()
    if out.exists() and any(out.iterdir()): ap.error("output must be empty")
    for name in ("Log","PCD","ros_home"): (out/name).mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[3]
    cfg=yaml.safe_load((root/"Modules/fast_lio2/config/mid360.yaml").read_text())
    T=np.asarray(cfg["mapping"]["extrinsic_T"]); E=np.asarray(cfg["mapping"]["extrinsic_R"]).reshape(3,3)
    scene=json.loads(args.scene.read_text()) if args.scene else {}
    origin=np.asarray(scene.get("origin_world",[0,0,0]),dtype=float)
    world_R=np.asarray(scene.get("rotation_world_from_local",np.eye(3)),dtype=float)
    if not np.isfinite(origin).all() or not np.isfinite(world_R).all() or origin.shape!=(3,) or world_R.shape!=(3,3) or not np.allclose(world_R.T@world_R,np.eye(3)) or not np.isclose(np.linalg.det(world_R),1) or not np.allclose(world_R[:,2],[0,0,1]):
        raise ValueError("Scene must use a finite, gravity-preserving proper rotation")
    if scene and Path(scene["map"]).resolve()!=args.map.resolve():raise ValueError("Scene map must match --map")
    if scene and not args.coverage:args.coverage=Path(scene["coverage"])
    write_json(out/"scene.json",scene)
    goals=np.asarray(scene.get("goals_local",[args.goal]),dtype=float).reshape(-1,3)
    if not len(goals) or not np.isfinite(goals).all():raise ValueError("Finite mission goals required")
    goal=goals[-1]
    planner_cfg=yaml.safe_load((root/"Simulation/forest_lio/config/ego_local.yaml").read_text())
    if args.scene:planner_cfg=planner_config_for_route(planner_cfg,goals)
    planner_path=out/"planner_config.yaml"
    planner_path.write_text(yaml.safe_dump(planner_cfg))
    lower=np.array([-planner_cfg["grid_map/map_size_x"]/2,-planner_cfg["grid_map/map_size_y"]/2,planner_cfg["grid_map/ground_height"]])
    upper=lower+np.array([planner_cfg["grid_map/map_size_x"],planner_cfg["grid_map/map_size_y"],planner_cfg["grid_map/map_size_z"]])
    upper[2]=min(upper[2],planner_cfg["grid_map/virtual_ceil_height"])
    if np.any(goals<lower+args.body_radius) or np.any(goals>upper-args.body_radius):
        raise ValueError("Mission goals/body proxy exceed planner map or configured floor/ceiling")
    xyz=to_local(read_pcd(args.map),origin,world_R)
    renderer=(BVHRenderer(xyz,.2,30,threads=args.render_threads,surface_model=args.surface_model) if args.renderer=="bvh"
              else PointRenderer(xyz,.5,.2,30))
    templates=templates_from_bag(args.template_bag,"/livox/lidar_192_168_2_181",24,.1,window=(40,120))
    model=__import__("json").loads(args.sensor_model.read_text()) if args.sensor_model else {}
    gyro_std=np.asarray(model.get("gyro_sample_std_rad_s",[0,0,0])) if args.noise else np.zeros(3)
    acc_std=np.asarray(model.get("acc_sample_std_g",[0,0,0])) if args.noise else np.zeros(3)
    gyro_bias=np.asarray(model.get("gyro_bias_rad_s",[0,0,0])) if args.noise else np.zeros(3)
    (out/"input_config.yaml").write_text(yaml.safe_dump(cfg))
    write_json(out/"input_sensor_model.json",model)
    (out/"input_launch.xml").write_text((root/"Simulation/forest_lio/launch/forest_lio_ego_sim.launch").read_text())
    rng=np.random.default_rng(args.seed)
    from coverage_map import CoverageMap
    coverage=CoverageMap.load(args.coverage) if args.coverage else None
    vertices=np.vstack([np.zeros(3),goals])
    route=np.vstack([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)) for a,b in zip(vertices[:-1],vertices[1:])])
    clearance=renderer.tree.query(route)[0].min()
    if clearance<args.body_radius:raise ValueError("Requested goal polyline violates point-map body clearance")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1",0)); port=sock.getsockname()[1]
    env=os.environ.copy()
    env.update(ROS_MASTER_URI="http://127.0.0.1:%d"%port,ROS_IP="127.0.0.1",ROS_HOME=str(out/"ros_home"))
    env.pop("ROS_HOSTNAME",None); os.environ.pop("ROS_HOSTNAME",None); os.environ.update(env)
    procs=[]; handles=[]; estimate=[]; truth=[]; commands=[]; render_times=[]; frame_stats=[]; plans=[]
    p=np.zeros(3); v=np.zeros(3); command=None; clock=None; now=1700000000.; goal_sent=False
    active_goal=-1;goal_events=[];goal_arrivals=[];final_arrival_frame=None
    started=time.monotonic(); succeeded=False; errors=[];loop_started=None;loop_wall_s=None
    try:
        def launch(cmd,name):
            h=open(out/name,"w"); handles.append(h)
            proc=subprocess.Popen(cmd,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
            procs.append(proc); return proc
        master=launch(["roscore","-p",str(port)],"roscore.log")
        proxy=xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        deadline=time.monotonic()+30
        while True:
            try:
                if proxy.getPid("/forest_closed_loop")[0]==1: break
            except OSError: pass
            if master.poll() is not None or time.monotonic()>deadline: raise RuntimeError("Master startup failed")
            time.sleep(.1)
        import rospy
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        from quadrotor_msgs.msg import PositionCommand
        from traj_utils.msg import Bspline
        from livox_ros_driver2.msg import CustomMsg
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Header
        rospy.set_param("/use_sim_time",True)
        rospy.init_node("forest_closed_loop",disable_signals=True)
        clock=rospy.Publisher("/clock",Clock,queue_size=100)
        imu_pub=rospy.Publisher("/sim/mid360/imu",Imu,queue_size=1000)
        lidar_pub=rospy.Publisher("/sim/mid360/lidar",CustomMsg,queue_size=10)
        goal_pub=rospy.Publisher("/uav1/prometheus/ego/goal",PoseStamped,queue_size=1,latch=True)
        def odom_cb(m):
            x=m.pose.pose.position; q=m.pose.pose.orientation
            estimate.append([m.header.stamp.to_sec(),x.x,x.y,x.z,q.x,q.y,q.z,q.w])
        def cmd_cb(m):
            nonlocal command
            command=m
            commands.append([m.header.stamp.to_sec(),m.position.x,m.position.y,m.position.z])
        rospy.Subscriber("/replay/lio/odom",Odometry,odom_cb,queue_size=1000)
        rospy.Subscriber("/sim/position_cmd",PositionCommand,cmd_cb,queue_size=10)
        rospy.Subscriber("/sim/planning/bspline",Bspline,lambda m:plans.append(m.start_time.to_sec()),queue_size=100)
        mapping=launch(["roslaunch",str(root/"Simulation/forest_lio/launch/forest_lio_ego_sim.launch"),
                        "output_root:="+str(out),"planner_config:="+str(planner_path)],"navigation.log")
        deadline=time.monotonic()+45
        while not (imu_pub.get_num_connections() and lidar_pub.get_num_connections() and goal_pub.get_num_connections()):
            clock.publish(Clock(clock=rospy.Time.from_sec(now)))
            if mapping.poll() is not None or time.monotonic()>deadline: raise RuntimeError("Navigation startup/subscription timeout")
            time.sleep(.05)
        state=proxy.getSystemState("/forest_closed_loop")[2]
        subscribers={topic:nodes for topic,nodes in state[1]}
        forbidden=[t for t,nodes in subscribers.items() if "truth" in t and any("ego" in n or "mapping" in n or "forest_lio" in n for n in nodes)]
        if forbidden: raise RuntimeError("Navigation subscribes to truth: "+str(forbidden))
        (out/"effective_planner_config.yaml").write_text(yaml.safe_dump(rospy.get_param("/uav_1_ego_planner_node")))
        write_json(out/"graph.json",{"subscribers":subscribers,"truth_subscriptions_in_navigation":forbidden})
        np.savetxt(out/"requested_goal.txt",goal)
        dt=.005; epoch=now
        def publish_imu(t,position,velocity,acceleration,index):
            stamp=rospy.Time.from_sec(epoch+t)
            m=Imu(header=Header(seq=index,stamp=stamp,frame_id="livox_frame"))
            m.orientation_covariance[0]=-1
            # Fixed attitude second-order plant. Gyro is zero plus optional measured scatter/bias.
            omega=gyro_bias+rng.normal(0,gyro_std,3)
            specific=(acceleration+np.array([0,0,9.81]))/9.81+rng.normal(0,acc_std,3)
            m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=omega
            m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=specific
            clock.publish(Clock(clock=stamp)); imu_pub.publish(m)
            truth.append(np.r_[epoch+t,position,0,0,0,1])
        publish_imu(0,p,v,np.zeros(3),0)
        loop_started=time.monotonic()
        for frame in range(int(args.duration*10)):
            if mapping.poll() is not None: raise RuntimeError("Navigation process exited")
            if frame>=40 and estimate:
                send_next=active_goal<0
                if active_goal>=0 and len(goal_arrivals)==active_goal:
                    # Mission arrival uses only estimated odometry, not simulator truth.
                    threshold=.2 if active_goal==len(goals)-1 else .25
                    if np.linalg.norm(np.asarray(estimate[-1][1:4])-goals[active_goal])<threshold and frame-goal_events[-1][1]>=10:
                        goal_arrivals.append([active_goal,frame,estimate[-1][0]])
                        if active_goal==len(goals)-1:final_arrival_frame=frame
                        else:send_next=True
                if send_next:
                    active_goal+=1
                    m=PoseStamped(header=Header(stamp=rospy.Time.from_sec(epoch+frame*.1),frame_id="world"))
                    m.pose.position.x,m.pose.position.y,m.pose.position.z=goals[active_goal]
                    m.pose.orientation.w=1;goal_pub.publish(m);goal_sent=True
                    goal_events.append([active_goal,frame,epoch+frame*.1])
            positions=[p.copy()]
            for j in range(1,21):
                if command is None or frame<40:
                    target=np.zeros(3); target_v=np.zeros(3); ff=np.zeros(3)
                else:
                    target=np.array([command.position.x,command.position.y,command.position.z])
                    target_v=np.array([command.velocity.x,command.velocity.y,command.velocity.z])
                    ff=np.array([command.acceleration.x,command.acceleration.y,command.acceleration.z])
                a=tracking_acceleration(p,v,target,target_v,ff)
                p=p+v*dt+.5*a*dt*dt; v=v+a*dt
                index=frame*20+j; t=index*dt; now=epoch+t
                publish_imu(t,p,v,a,index); positions.append(p.copy())
                time.sleep(.001)
            rays,offsets,lines,refl=templates[frame%len(templates)]
            positions=np.asarray(positions)
            pmid=positions[10]
            began=time.monotonic()
            if args.renderer=="bvh":
                # Fixed attitude plant: interpolate its actual sampled position at every point time.
                times=offsets*1e-9
                origins=np.column_stack([np.interp(times,np.arange(21)*dt,positions[:,k]) for k in range(3)])+T
                ranges=renderer.scan_world(rays@E.T,origins,.5)
            else:
                reuse=args.renderer=="cached" or (args.renderer=="conservative" and np.max(np.linalg.norm(positions-pmid,axis=1))<1e-6)
                scene=renderer.prepare(pmid+T,E) if reuse else None
                ranges=np.zeros(len(rays)); groups=np.floor(offsets*1e-6/10).astype(int)
                for g in np.unique(groups):
                    mask=groups==g; t=(float(offsets[mask].min())+float(offsets[mask].max()))*.5e-9
                    pos=np.array([np.interp(t,np.arange(21)*dt,positions[:,k]) for k in range(3)])
                    ranges[mask]=(renderer.cast(rays[mask],scene,pos+T,E,.5) if reuse
                                  else renderer.scan(rays[mask],pos+T,E,.5))
            render_times.append(time.monotonic()-began)
            pts=np.zeros(len(rays),dtype=POINT_DTYPE)
            pts["offset_time"]=offsets; pts["xyz"]=rays*ranges[:,None]; pts["line"]=lines
            pts["reflectivity"]=np.where(ranges>0,refl,0)
            lidar_pub.publish(pack_scan(frame,epoch+frame*.1,pts))
            expected=epoch+frame*.1+float(offsets.max())*1e-9
            deadline=time.monotonic()+5
            # Initialization scans need no odom; thereafter acknowledge LIO before next sensor frame.
            if frame>=5:
                while not estimate or estimate[-1][0]<expected-.02:
                    if time.monotonic()>deadline: raise RuntimeError("LIO frame acknowledgement timeout at frame %d"%frame)
                    elapsed=5-(deadline-time.monotonic())
                    clock.publish(Clock(clock=rospy.Time.from_sec(now+min(.004,max(0,elapsed)*.08))))
                    time.sleep(.005)
            else: time.sleep(.03)
            frame_stats.append([frame,int((ranges>0).sum()),len(rays)])
            if args.scene and final_arrival_frame is not None and frame-final_arrival_frame>=30:break
            if frame%25==0: print("Closed loop %.1fs, plans %d, position %s"%(frame*.1,len(plans),p),flush=True)
        loop_wall_s=time.monotonic()-loop_started
        time.sleep(.1)
        succeeded=goal_sent and bool(plans) and len(goal_arrivals)==len(goals) and np.linalg.norm(p-goal)<.2
    except Exception as exc:
        errors.append(str(exc))
        raise
    finally:
        # Unblock ROS simulated-time shutdown without publishing additional sensor observations.
        import threading
        done=threading.Event()
        def drain():
            while not done.wait(.02):
                if clock is not None: clock.publish(Clock(clock=rospy.Time.from_sec(now+time.monotonic()-drain_start)))
        drain_start=time.monotonic()
        thread=threading.Thread(target=drain,daemon=True); thread.start()
        for proc in reversed(procs): stop(proc,timeout=8)
        done.set(); thread.join(timeout=1)
        if "rospy" in locals(): rospy.signal_shutdown("Closed loop complete")
        for h in handles:h.close()
        est=save_trajectory(out/"lio.tum",estimate); tr=save_trajectory(out/"truth.tum",truth)
        evidence=None
        if len(tr):
            world_truth=tr.copy();world_truth[:,1:4]=to_world(tr[:,1:4],origin,world_R)
            world_truth[:,4:8]=Rotation.from_matrix(world_R).as_quat()
            save_trajectory(out/"truth_world.tum",world_truth)
            if coverage is not None:
                rows,evidence=footprint_evidence(coverage,world_truth[::20,1:4],args.body_radius)
                np.savetxt(out/"footprint_evidence.csv",np.column_stack([world_truth[::20,0],rows]),delimiter=",",header="stamp,center_unknown,intersecting_cells,unknown_cells,endpoint_cells")
        np.savetxt(out/"commands.csv",np.asarray(commands).reshape(-1,4),delimiter=",",header="stamp,x,y,z")
        np.savetxt(out/"frames.csv",np.asarray(frame_stats).reshape(-1,3),delimiter=",",header="frame,hits,rays")
        write_json(out/"result.json",{"status":"ok" if succeeded else "failed","errors":errors,
            "gyro_sample_std_rad_s":gyro_std,"acc_sample_std_g":acc_std,"gyro_bias_rad_s":gyro_bias,
            "planner_map_bounds_local":{"min":lower,"max":upper},"goal_sent":goal_sent,"goals_local":goals,"goal_events":goal_events,"goal_arrivals":goal_arrivals,"mission_completed":len(goal_arrivals)==len(goals),"plans":len(plans),"commands":len(commands),"frames":len(frame_stats),
            "final_position":p,"goal":goal,"goal_error_m":float(np.linalg.norm(p-goal)),
            "lidar_render_wall_s":summary(render_times),"wall_s":time.monotonic()-started,
            "loop_wall_s":loop_wall_s,"sim_duration_s":len(frame_stats)/10.,"simulation_to_wall_ratio":len(frame_stats)/10./loop_wall_s if loop_wall_s else None,
            "native_profile":getattr(renderer,"last_profile",None),
            "surface_model_report":getattr(renderer,"surface_report",None),
            "footprint_evidence":evidence,
            "coverage_unknown_truth_samples":int((coverage.classify(world_truth[:,1:4])==0).sum()) if coverage is not None and len(tr) else None,
            "coverage_note":"Evaluation of evidence only; does not certify a free UAV footprint.",
            "comparison":trajectory_comparison(est,tr) if len(est)>1 and len(tr)>1 else {},
            "minimum_sampled_truth_point_clearance_m":float(renderer.tree.query(tr[:,1:4])[0].min()) if len(tr) else None,
            "noise_enabled":args.noise,"sensor_model":str(args.sensor_model) if args.sensor_model else None,
            "seed":args.seed,"renderer":args.renderer,"map":str(args.map.resolve()),"template_bag":str(args.template_bag.resolve()),
            "parameters":{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},"model":"Acceleration-limited second-order position tracker, fixed attitude; no PX4/rotor dynamics.",
            "limitations":["Unknown map geometry is absent; PCD clearance is not proof of safe free space.",
                           "Single UAV, fixed-attitude route test; not full forest mission/swarm or dynamics validation.",
                           "BVH queries nearest modeled disk/sphere intersection at each point pose; fixed/adaptive support still approximates forest geometry.",
                           "BVH uses interpolated 200Hz plant history; legacy raster modes use 10ms subscan midpoint poses.",
                           "Conservative mode reuses frame raster only at effectively stationary poses; cached mode is experimental.",
                           "No collision/contact response; point clearance measured for evaluation.",
                           "Simulation time advances at 200Hz; isolated render speed does not prove whole-loop real-time speed."]})
        print(out/"result.json",flush=True)
    if not succeeded: raise SystemExit("Closed-loop goal test failed")
if __name__=="__main__":
    main()
