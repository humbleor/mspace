"""Build/run isolated one-trunk Gazebo contact and bypass fixtures.

Uses the pinned Iris body inertia and collision geometry, no navigation/PX4,
zero gravity to isolate contact, and one initial velocity (no position forcing).
"""
import argparse
import copy
import csv
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]


def trunk_model(trunk):
    model=ET.Element("model",name="single_trunk")
    ET.SubElement(model,"static").text="true"
    ET.SubElement(model,"pose").text=f"{trunk['x']} {trunk['y']} {trunk['height']/2} 0 0 0"
    link=ET.SubElement(model,"link",name="trunk")
    for kind in ("collision","visual"):
        item=ET.SubElement(link,kind,name="trunk_"+kind)
        cylinder=ET.SubElement(ET.SubElement(item,"geometry"),"cylinder")
        ET.SubElement(cylinder,"radius").text=str(trunk["radius"])
        ET.SubElement(cylinder,"length").text=str(trunk["height"])
        if kind=="collision":
            surface=ET.SubElement(item,"surface")
            ode=ET.SubElement(ET.SubElement(surface,"contact"),"ode")
            for k,v in {"kp":1e6,"kd":100,"max_vel":1,"min_depth":.0001}.items():
                ET.SubElement(ode,k).text=str(v)
    return model


def make_world(config, iris_path, plugin, out, lateral):
    root=ET.Element("sdf",version="1.6"); w=ET.SubElement(root,"world",name="single_trunk_fixture")
    ET.SubElement(w,"gravity").text="0 0 0"
    physics=ET.SubElement(w,"physics",name="default",type="ode")
    ET.SubElement(physics,"max_step_size").text=str(config["step_size"])
    ET.SubElement(physics,"real_time_update_rate").text="0"
    ET.SubElement(ET.SubElement(w,"include"),"uri").text="model://sun"
    w.append(trunk_model(config["trunk"]))
    body=ET.SubElement(w,"model",name="iris_contact_fixture")
    ET.SubElement(body,"pose").text=f"0 {lateral} {config['body_height']} 0 0 0"
    link=ET.SubElement(body,"link",name="base_link")
    iris=ET.parse(iris_path).getroot().find("model")
    base=iris.find("link[@name='base_link']")
    link.append(copy.deepcopy(base.find("inertial")))
    # Rigidly combine the pinned Iris body and rotor contact shapes. Flight motor
    # plugins are deliberately absent; this fixture verifies contact mechanics.
    for original in iris.findall("link"):
        pose=original.findtext("pose","0 0 0 0 0 0")
        for col in original.findall("collision"):
            col=copy.deepcopy(col); col.find("pose").text=pose
            link.append(col)
            visual=ET.SubElement(link,"visual",name=col.get("name")+"_visual")
            ET.SubElement(visual,"pose").text=pose
            visual.append(copy.deepcopy(col.find("geometry")))
    probe=ET.SubElement(body,"plugin",name="single_trunk_probe",filename=str(plugin))
    for k,v in {"states":out/"states.csv","contacts":out/"contacts.csv","speed":config["launch_speed"]}.items():
        ET.SubElement(probe,k).text=str(v)
    path=out/"single_trunk.world"; ET.ElementTree(root).write(path,encoding="unicode")
    return path


def assess(states, contacts, config, lateral,expect_contact=None):
    states=np.asarray(states,dtype=float)
    if states.ndim!=2 or len(states)<2 or states.shape[1]!=10 or not np.isfinite(states).all():
        raise ValueError("Missing/invalid physical state evidence")
    if states[-1,0]<config["duration"] or np.any(np.diff(states[:,0])<=0):
        raise ValueError("Incomplete or unordered physical states")
    hit=bool(contacts)
    terminal_x=float(states[-1,1]); speed=float(np.linalg.norm(states[-1,4:7]))
    expected_x=config["launch_speed"]*states[-1,0]
    expect_contact=(lateral==0) if expect_contact is None else expect_contact
    if expect_contact and lateral==0:
        passed=hit and states[:,1].max()<config["trunk"]["x"] and speed<config["launch_speed"]*.5
    elif expect_contact:
        response=max(abs(states[:,2]-lateral).max(),abs(states[:,9]).max(),abs(states[:,4]-config["launch_speed"]).max())
        passed=hit and response>.01
    else:
        passed=not hit and abs(terminal_x-expected_x)<.03 and abs(states[-1,2]-lateral)<.03
    return {"passed":bool(passed),"contact_points":len(contacts),"first_contact":contacts[0] if hit else None,
            "max_contact_force_n":max((float(c["force_n"]) for c in contacts),default=0),
            "terminal_x_m":terminal_x,"terminal_speed_m_s":speed,"duration_sim_s":float(states[-1,0])}


def main():
    # Match build_runtime: avoid probing Windows executables during CMake discovery.
    os.environ["PATH"]=":".join(v for v in os.environ["PATH"].split(":") if not v.startswith("/mnt/"))
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--config",type=Path,default=ROOT/"Simulation/px4_forest/config/single_trunk.json")
    p.add_argument("--build",action="store_true",help="Build only the project Gazebo plugins, not PX4")
    a=p.parse_args(); out=a.output.resolve()
    if out.exists():p.error("Use a new output directory")
    out.mkdir(parents=True)
    build=ROOT/"artifacts/px4_forest/plant_build"
    if a.build:
        subprocess.run(["cmake","-S",str(ROOT/"Simulation/px4_forest/native"),"-B",str(build),"-DCMAKE_BUILD_TYPE=Release"],check=True)
        subprocess.run(["cmake","--build",str(build),"-j1"],check=True)
    plugin=build/"libmspace_single_trunk_probe.so"
    if not plugin.is_file():p.error("Build the probe first with --build")
    config=json.loads(a.config.read_text())
    if any(not np.isfinite(config[k]) or config[k]<=0 for k in ("launch_speed","duration","step_size","body_height")):
        p.error("Fixture speed/duration/step/height must be finite and positive")
    if any(not np.isfinite(config["trunk"][k]) or config["trunk"][k]<=0 for k in ("radius","height")):
        p.error("Trunk radius/height must be finite and positive")
    iris=ROOT/"artifacts/px4_forest/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf.jinja"
    results={}
    for name,entry in config["cases"].items():
        lateral=entry["y"] if isinstance(entry,dict) else entry
        case_config=dict(config)
        if isinstance(entry,dict):case_config["launch_speed"]=entry.get("speed",config["launch_speed"])
        expect_contact=entry.get("contact",False) if isinstance(entry,dict) else lateral==0
        case=out/name; case.mkdir()
        world=make_world(case_config,iris,plugin,case,lateral)
        with socket.socket() as s:s.bind(("127.0.0.1",0)); port=s.getsockname()[1]
        env=os.environ.copy(); env["GAZEBO_MASTER_URI"]=f"http://127.0.0.1:{port}"
        with (case/"gazebo.log").open("w") as log:
            proc=subprocess.Popen(["gzserver","--verbose",str(world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline=time.monotonic()+45
                while True:
                    if proc.poll() is not None:raise RuntimeError("Gazebo exited; inspect "+str(case/"gazebo.log"))
                    if time.monotonic()>deadline:raise RuntimeError("Fixture timeout")
                    file=case/"states.csv"
                    if file.exists():
                        lines=file.read_text().splitlines()
                        if len(lines)>1:
                            try:
                                if float(lines[-1].split(",")[0])>=config["duration"]:break
                            except ValueError:pass
                    time.sleep(.05)
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        states=np.loadtxt(case/"states.csv",delimiter=",",skiprows=1,ndmin=2)
        states=states[states[:,0]<=config["duration"]+config["step_size"]]
        with (case/"contacts.csv").open() as f:contacts=[c for c in csv.DictReader(f) if float(c["stamp"])<=states[-1,0]]
        results[name]=assess(states,contacts,case_config,lateral,expect_contact)
        results[name].update(launch_speed_m_s=case_config["launch_speed"],lateral_m=lateral,expected_contact=expect_contact)
        fig,axes=plt.subplots(1,2,figsize=(10,4))
        axes[0].plot(states[:,0],states[:,1]);axes[0].axhline(config["trunk"]["x"],color="brown",ls="--")
        axes[0].set(xlabel="Simulation time (s)",ylabel="Body x (m)")
        axes[1].plot(states[:,0],states[:,4]);axes[1].set(xlabel="Simulation time (s)",ylabel="Body vx (m/s)")
        for ax in axes:ax.grid(True)
        fig.suptitle(name);fig.tight_layout();fig.savefig(case/"response.png",dpi=140);plt.close(fig)
    result={"status":"ok" if all(r["passed"] for r in results.values()) else "failed","config":config,"cases":results,
            "limits":["Rigid Iris collision/inertia fixture, zero gravity, initial velocity only; no PX4/LIO/EGO flight test.",
                      "Static synthetic trunk; no foliage, damage, flex or calibrated wood contact."]}
    (out/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    text=["# Single trunk physical contact fixture","",f"Status: {result['status']}","",json.dumps(results,indent=2),""]
    text += [f"![{name} response]({name}/response.png)" for name in results]
    text += ["","Limits:"]+["- "+v for v in result["limits"]]
    (out/"report.md").write_text("\n".join(text)+"\n")
    print(out/"report.md")
    if result["status"]!="ok":raise SystemExit(1)


if __name__=="__main__":main()
