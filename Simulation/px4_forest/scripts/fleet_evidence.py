"""Independent fleet gates, ROS ownership audit and continuous pair clearance."""
import re
import numpy as np

def all_ready(flags):
    flags=list(flags)
    return bool(flags) and all(flags)

def all_settled(stamps, now, hold=3.):
    stamps=list(stamps)
    return bool(stamps) and all(t is not None and now-t>=hold for t in stamps)

def pair_separation(traces, minimum, max_gap=.05, window=None):
    """Minimum distance of synchronized piecewise-linear physical trajectories."""
    result={"passed":True,"minimum_required_m":float(minimum),"pairs":[],"errors":[]}
    arrays={}
    for name,rows in traces.items():
        a=np.asarray(rows,dtype=float).reshape(-1,8)
        if len(a)<2 or not np.isfinite(a).all() or np.any(np.diff(a[:,0])<=0) or np.max(np.diff(a[:,0]))>max_gap+1e-9:
            result["errors"].append(name+": missing/invalid/gapped physical trajectory")
        else: arrays[name]=a
    if len(arrays)!=len(traces) or not arrays:
        result["passed"]=False;return result
    names=list(arrays)
    for i,name in enumerate(names):
        for other in names[i+1:]:
            a,b=arrays[name],arrays[other]
            lo=max(a[0,0],b[0,0]);hi=min(a[-1,0],b[-1,0])
            if window is not None:
                lo,hi=window
            if hi<=lo or any(x[0,0]>lo+max_gap or x[-1,0]<hi-max_gap for x in (a,b)):
                result["errors"].append(name+"/"+other+": incomplete common observation window");continue
            if window is None and (abs(a[0,0]-b[0,0])>max_gap or abs(a[-1,0]-b[-1,0])>max_gap):
                result["errors"].append(name+"/"+other+": unequal observation coverage");continue
            t=np.unique(np.r_[lo,hi,a[(a[:,0]>lo)&(a[:,0]<hi),0],b[(b[:,0]>lo)&(b[:,0]<hi),0]])
            pa=np.column_stack([np.interp(t,a[:,0],a[:,k]) for k in (1,2,3)])
            pb=np.column_stack([np.interp(t,b[:,0],b[:,k]) for k in (1,2,3)])
            relative=pa-pb;start=relative[:-1];delta=np.diff(relative,axis=0)
            norm=np.einsum("ij,ij->i",delta,delta)
            alpha=np.clip(-np.einsum("ij,ij->i",start,delta)/np.maximum(norm,1e-30),0,1)
            distance=np.linalg.norm(start+alpha[:,None]*delta,axis=1);k=int(np.argmin(distance))
            value=float(distance[k])
            result["pairs"].append({"vehicles":[name,other],"minimum_m":value,
                "stamp":float(t[k]+alpha[k]*(t[k+1]-t[k])),"passed":value>=minimum})
    result["passed"]=not result["errors"] and all(p["passed"] for p in result["pairs"])
    return result

def audit_graph(vehicles,publishers,subscribers):
    errors=[];expected={}
    navigation=set()
    for v in vehicles:
        uid=v.uav_id;ns=v.namespace;adapter="/swarm_frame_adapter_uav_%d"%uid
        navigation.update([ns+"/laserMapping","/uav_%d_ego_planner_node"%uid,
                           "/swarm_estimator_uav_%d"%uid,"/swarm_controller_uav_%d"%uid,
                           "/ego_traj_to_cmd_uav_%d"%uid,ns+"_traj_server",adapter,ns+"/mavros"])
        expected.update({ns+"/mavros/setpoint_raw/local":adapter,
                         ns+"/mavros/vision_pose/pose":"/swarm_estimator_uav_%d"%uid,
                         ns+"/planning/ego/traj_cmd":ns+"_traj_server",
                         ns+"/world/odom":adapter,ns+"/world/cloud":adapter,
                         ns+"/prometheus/drone_state":adapter,
                         v.sensor_prefix+"/mid360/lidar":"/forest_swarm",
                         v.sensor_prefix+"/mid360/imu":"/gazebo"})
    for topic,node in expected.items():
        if set(publishers.get(topic,[]))!={node}:errors.append("publisher ownership: "+topic)
    for topic,nodes in subscribers.items():
        if re.match(r"^/sim/(?:uav\d+/)?plant/odom$",topic):
            if navigation.intersection(nodes):errors.append("truth leaked to navigation: "+topic)
        if re.match(r"^/sim/uav\d+/mid360/(lidar|imu)$",topic):
            allowed={v.namespace+"/laserMapping" for v in vehicles if topic.startswith(v.sensor_prefix+"/")}
            if navigation.intersection(nodes)-allowed:errors.append("cross-aircraft sensor subscription: "+topic)
    for v in vehicles:
        for kind in ("lidar","imu"):
            if v.namespace+"/laserMapping" not in subscribers.get(v.sensor_prefix+"/mid360/"+kind,[]):
                errors.append(v.name+" missing "+kind+" subscription")
    if len(vehicles)>1:
        planners={"/uav_%d_ego_planner_node"%v.uav_id for v in vehicles}
        if not planners.issubset(publishers.get("/broadcast_bspline",[])) or not planners.issubset(subscribers.get("/broadcast_bspline",[])):
            errors.append("incomplete fleet trajectory broadcast connections")
    return {"passed":not errors,"errors":errors,"expected_publishers":expected,
            "publishers":publishers,"subscribers":subscribers}
