"""Bind Modules launch templates to simulated hardware and explicit mission settings."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import math
import numpy as np

def task_map_layout(goals,grid,planner,advanced):
    """Bound the dense simulation map around all goals with planning headroom."""
    points=np.asarray(goals,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Finite nonempty Nx3 task goals required for map sizing")
    points=np.vstack([np.zeros(3),points])
    if grid:
        bounds=np.array([[grid[k][b] for k in "xyz"] for b in ("min","max")],dtype=float)
        if not np.isfinite(bounds).all() or np.any(bounds[0]>bounds[1]):
            raise ValueError("Invalid grid bounds for map sizing")
        points=np.vstack([points,bounds])
    parameters={p.get("name"):p.get("value") for p in advanced.iter("param")}
    resolution=float(parameters["grid_map/resolution"])
    local=np.array([float(parameters["grid_map/local_update_range_"+k]) for k in "xyz"])
    margin=float(parameters["grid_map/local_map_margin"])*resolution
    ground=float(parameters["grid_map/ground_height"])
    include=next(e for e in planner.findall("include") if "advanced_param_px4" in e.get("file",""))
    include_args={a.get("name"):a.get("value") for a in include.findall("arg")}
    horizon=float(include_args["planning_horizon"])
    original={k:float(next(a.get("value",a.get("default")) for a in planner.findall("arg") if a.get("name")=="map_size_"+k)) for k in "xyz"}
    if not all(np.isfinite(v) and v>0 for v in [resolution,horizon,*local,*original.values()]) or margin<0 or not np.isfinite(margin+ground):
        raise ValueError("Invalid planner map sizing parameters")
    if points[:,2].min()<ground:
        raise ValueError("Task goal below unchanged planner ground_height")
    # Keep symmetric XY origin and include local range, map margin, horizon,
    # and 1 m headroom. Round in even voxel blocks; preserve template Z.
    block_voxels=max(2,2*int(math.ceil(8/resolution/2)))
    def round_size(value):
        cells=int(math.ceil(value/resolution))
        return math.ceil(cells/block_voxels)*block_voxels*resolution
    padding=local[:2]+margin+horizon+1.
    size=[round_size(max(32.,2*(np.max(np.abs(points[:,i]))+padding[i]))) for i in (0,1)]
    size.append(max(original["z"],round_size(points[:,2].max()-ground+horizon+1.)))
    voxels=[int(math.ceil(v/resolution)) for v in size]
    original_voxels=[int(math.ceil(original[k]/resolution)) for k in "xyz"]
    return {"size_m":dict(zip("xyz",size)),"original_size_m":original,"resolution_m":resolution,
            "task_bounds_m":{"min":points.min(axis=0).tolist(),"max":points.max(axis=0).tolist()},
            "xy_padding_m":padding.tolist(),"planning_horizon_m":horizon,"ground_height_m":ground,
            "voxel_count":math.prod(voxels),"dense_buffer_estimate_bytes":15*math.prod(voxels),
            "original_dense_buffer_estimate_bytes":15*math.prod(original_voxels),
            "estimate_scope":"GridMap six dense buffers only; excludes planner, LIO, renderer and GUI overhead",
            "z_policy":"Preserve template height, expand only if task goals need more headroom"}

def prepare_modules(root,out,goals,grid=None):
    root,out=Path(root),Path(out)
    wrapper=ET.Element("launch")
    ET.SubElement(wrapper,"param",name="use_sim_time",value="true")
    ET.SubElement(wrapper,"param",name="output_root",value=str(out))
    cfg=yaml.safe_load((root/"Modules/fast_lio2/config/mid360.yaml").read_text())
    for original,simulation in [(cfg["common"]["lid_topic"],"/sim/mid360/lidar"),(cfg["common"]["imu_topic"],"/sim/mid360/imu")]:
        ET.SubElement(wrapper,"remap",attrib={"from":original,"to":simulation})
    lio=ET.SubElement(wrapper,"include",file=str(root/"Modules/fast_lio2/launch/mapping_mid360.launch"))
    ET.SubElement(lio,"arg",name="uav_id",value="1")
    ET.SubElement(lio,"arg",name="rviz",value="false")
    control=ET.SubElement(wrapper,"include",file=str(root/"Modules/swarm_control/launch/ego_swarm_control.launch"))
    for k,v in {"uav_id":"1","swarm_num":"1","input_source":"4"}.items():
        ET.SubElement(control,"arg",name=k,value=v)
    planner=ET.parse(root/"Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch")
    include=next(e for e in planner.getroot().findall("include") if "advanced_param_px4" in e.get("file",""))
    advanced=ET.parse(root/"Modules/ego_planner_swarm/plan_manage/launch_new/advanced_param_px4.xml").getroot()
    layout=task_map_layout(goals,grid,planner.getroot(),advanced)
    for e in planner.getroot().findall("arg"):
        for axis in "xyz":
            if e.get("name")=="map_size_"+axis:e.set("value",str(layout["size_m"][axis]))
    overrides={"flight_type":"3" if grid else "2"}
    if grid:
        for axis in ("x","y","z"):
            for boundary in ("min","max"):overrides["box_"+boundary+"_"+axis]=str(grid[axis][boundary])
            overrides["step_"+axis.upper()]=str(grid[axis]["step"])
        overrides["waypointDistriFlag"]=str(grid.get("waypointDistriFlag",0))
        overrides["grid_direction"]=str(grid.get("grid_direction",0))
        if not any(e.get("name")=="grid_direction" for e in include.findall("arg")):
            ET.SubElement(include,"arg",name="grid_direction",value=overrides["grid_direction"])
    for e in include.findall("arg"):
        if e.get("name") in overrides:e.set("value",overrides[e.get("name")])
    # SITL uses the existing hardware/RC trigger gate: do not start a
    # trajectory before the module controller has completed takeoff.
    ET.SubElement(planner.getroot(),"param",name="/uav_1_ego_planner_node/fsm/realworld_experiment",value="true",type="bool")
    ET.SubElement(planner.getroot(),"param",name="/uav_1_ego_planner_node/fsm/strict_waypoint_tracking",value="true",type="bool")
    if not grid:
        # Scene-derived routes use the original PRESET_TARGET workflow.
        ET.SubElement(planner.getroot(),"param",name="/uav_1_ego_planner_node/fsm/waypoint_num",value=str(len(goals)-1),type="int")
        for i,pos in enumerate(goals[1:]):
            for axis,value in zip("xyz",pos):
                ET.SubElement(planner.getroot(),"param",name="/uav_1_ego_planner_node/fsm/waypoint%d_%s"%(i,axis),value=str(float(value)),type="double")
    planner.write(out/"planner_modules.launch",encoding="unicode")
    ET.SubElement(wrapper,"include",file=str(out/"planner_modules.launch"))
    ET.ElementTree(wrapper).write(out/"modules.launch",encoding="unicode")
    return {"lio_template":"Modules/fast_lio2/launch/mapping_mid360.launch",
            "planner_template":"Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch",
            "control_template":"Modules/swarm_control/launch/ego_swarm_control.launch",
            "flight_type":3 if grid else 2,"control_takeoff_height":1.2,
            "planner_odom":"/uav1/mavros/local_position/odom",
            "planner_cloud":"/uav1/drone_cloud_registered","traj_time_forward":1.0,
            "overrides":dict(overrides,realworld_experiment=True,strict_waypoint_tracking=True),"algorithms_modified":True,
            "simulation_map_layout":layout,
            "module_changes":["Optional strict native mission scheduling with odometry arrivals; optimizer unchanged"]}
