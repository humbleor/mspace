"""Bind Modules launch templates to simulated hardware and explicit mission settings."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

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
            "module_changes":["Optional strict native mission scheduling with odometry arrivals; optimizer unchanged"]}
