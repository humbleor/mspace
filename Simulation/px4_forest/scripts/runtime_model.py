"""Per-run PX4 startup/model generation: no external workspace mutation."""
from pathlib import Path
import shutil,subprocess,xml.etree.ElementTree as ET
import os,json

def precise_timeout_scaling(startup_text):
    return startup_text.replace('echo "$PX4_SIM_SPEED_FACTOR *',
                                'echo "scale=6; $PX4_SIM_SPEED_FACTOR *')

def prepare_model(px4,out,plugin,ports,instance=20,gps=False,speed=.5,model_name="forest_iris",pose=None,sensor_prefix=None,physics_speed=None):
    gazebo=px4/"Tools/sitl_gazebo"
    subprocess.run(["python3",str(gazebo/"scripts/jinja_gen.py"),str(gazebo/"models/iris/iris.sdf.jinja"),str(gazebo),
        "--mavlink_tcp_port",str(ports["sim"]),"--mavlink_id",str(instance+1),
        "--output-file",str(out/"iris.sdf")],check=True)
    tree=ET.parse(out/"iris.sdf");root=tree.getroot();body=root.find("model")
    body.set("name",model_name);ET.SubElement(body,"pose").text=" ".join(map(str,pose or [0,0,.25,0,0,0]))
    if not gps:
        for element in list(body):
            if (element.tag=="include" and element.findtext("uri")=="model://gps") or (element.tag=="joint" and element.get("name")=="gps0_joint"):
                body.remove(element)
    interface=next(p for p in body.findall("plugin") if p.get("name")=="mavlink_interface")
    interface.find("send_odometry").text="0";interface.find("send_vision_estimation").text="0"
    interface.find("mavlink_addr").text="INADDR_ANY"
    interface.find("qgc_udp_port").text=str(ports["unused"])
    interface.find("sdk_udp_port").text=str(ports["unused"])
    plant=ET.SubElement(body,"plugin",name="mspace_forest_plant",filename=str(plugin))
    if physics_speed is not None:
        ET.SubElement(plant,"physics_update_rate").text=str(250.*physics_speed)
    if sensor_prefix:
        ET.SubElement(plant,"sensor_prefix").text=sensor_prefix
        ET.SubElement(plant,"imu_frame").text=model_name+"/imu"
        ET.SubElement(plant,"body_frame").text=model_name+"/base_link"
    tree.write(out/"iris.sdf",encoding="unicode")
    world=ET.Element("sdf",version="1.6");w=ET.SubElement(world,"world",name="forest")
    physics=ET.SubElement(w,"physics",name="default",type="ode")
    ET.SubElement(physics,"max_step_size").text=".004"
    ET.SubElement(physics,"real_time_update_rate").text="250"
    ET.SubElement(physics,"real_time_factor").text="1"
    ET.SubElement(w,"gravity").text="0 0 -9.81"
    for uri in ("model://ground_plane","model://sun"):
        ET.SubElement(ET.SubElement(w,"include"),"uri").text=uri
    gui=ET.SubElement(w,"gui",fullscreen="0")
    camera=ET.SubElement(gui,"camera",name="user_camera")
    ET.SubElement(camera,"pose").text="12 -12 9 0 .5 2.1"
    ET.SubElement(camera,"view_controller").text="orbit"
    w.append(body);ET.ElementTree(world).write(out/"forest.world",encoding="unicode")
    shutil.copytree(px4/"build/px4_sitl_default/etc",out/"etc",symlinks=False)
    init=out/"etc/init.d-posix"
    # PX4 v1.13's bc expressions truncate .1 * .5 to zero without
    # explicit decimal precision. Preserve the intended scaled watchdogs.
    startup=init/"rcS"
    startup.write_text(precise_timeout_scaling(startup.read_text()))
    file=init/"px4-rc.simulator";s=file.read_text()
    s=s.replace("simulator_tcp_port=$((4560+px4_instance))","simulator_tcp_port="+str(ports["sim"]))
    file.write_text(s)
    (init/"px4-rc.mavlink").write_text("mavlink start -x -u %d -r 4000000 -m onboard -o %d\n"%(ports["px4"],ports["mavros"]))
    with (init/"px4-rc.params").open("a") as f:
        f.write("\n# Isolated forest SITL settings\nparam set COM_RC_IN_MODE 4\nparam set COM_RCL_EXCEPT 4\nparam set COM_ARM_WO_GPS 1\nparam set EKF2_MULTI_IMU 1\nparam set EKF2_MULTI_MAG 1\n")
        if not gps:f.write("param set EKF2_AID_MASK 24\nparam set EKF2_HGT_MODE 3\nparam set EKF2_EV_DELAY 0\n")
    return gazebo


def compose_world(models, destination):
    """Use one generated world shell and append each independently generated body."""
    paths=[Path(p) for p in models]
    if not paths: raise ValueError("At least one model is required")
    tree=ET.parse(paths[0].parent/"forest.world")
    world=tree.getroot().find("world")
    for body in list(world.findall("model")): world.remove(body)
    names=set()
    for path in paths:
        body=ET.parse(path).getroot().find("model")
        if body is None or body.get("name") in names: raise ValueError("Unique model names required")
        names.add(body.get("name"));world.append(body)
    tree.write(destination,encoding="unicode")
