#!/usr/bin/env python3
"""Make a report from recorded PX4 experiment evidence."""
import argparse,json
from pathlib import Path
import numpy as np
from mission_evidence import ordered_arrivals
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
def main():
    p=argparse.ArgumentParser();p.add_argument("output",type=Path);a=p.parse_args()
    r=json.loads((a.output/"result.json").read_text())
    evidence={}
    try:
        from pyulog import ULog
        logs=sorted((a.output/"log").rglob("*.ulg"))
        if logs:
            u=ULog(str(logs[-1]))
            flags=next(d.data for d in u.data_list if d.name=="vehicle_status_flags")
            for key in ["position_reliant_on_vision_position","position_reliant_on_gps","gps_position_valid","global_position_valid"]:
                vals,counts=np.unique(flags[key],return_counts=True)
                evidence[key]={str(int(v)):int(n) for v,n in zip(vals,counts)}
            (a.output/"ekf_evidence.json").write_text(json.dumps(evidence,indent=2)+"\n")
            if r["stage"]=="forest" and (evidence["position_reliant_on_gps"].get("1",0) or not evidence["position_reliant_on_vision_position"].get("1",0)):
                raise RuntimeError("ULog did not confirm GNSS-denied external position fusion")
    except ImportError:
        evidence={"unavailable":"Install pyulog to inspect native EKF evidence"}
    frame_file=a.output/"frames.csv"
    frame_data=np.loadtxt(frame_file,delimiter=",",ndmin=2) if frame_file.stat().st_size>40 else np.empty((0,3))
    origin=float(frame_data[0,0]-.2) if frame_data.size else None
    if origin is None:
        truth_data=np.loadtxt(a.output/"truth.tum",ndmin=2)
        origin=float(truth_data[0,0]) if truth_data.size else 0
    fig,axes=plt.subplots(2,2,figsize=(11,8))
    for name,label in [("truth","Physics"),("lio","Fast-LIO2"),("fcu","PX4 EKF")]:
        f=a.output/(name+".tum")
        if f.stat().st_size==0:continue
        d=np.loadtxt(f,ndmin=2)
        if not d.size:continue
        axes[0,0].plot(d[:,1],d[:,2],label=label)
        d=d[d[:,0]>=origin]
        axes[0,1].plot(d[:,0]-origin,d[:,3],label=label)
    axes[0,0].set(xlabel="x (m)",ylabel="y (m)",title="Trajectory");axes[0,0].axis("equal")
    axes[0,1].set(xlabel="Elapsed simulation time (s)",ylabel="Height (m)")
    f=a.output/"imu.csv"
    if f.stat().st_size>80:
        d=np.loadtxt(f,delimiter=",",ndmin=2)
        d=d[d[:,0]>=origin]
        axes[1,0].plot(d[:,0]-origin,d[:,4:7]);axes[1,0].set(title="Physical body gyro",ylabel="rad/s")
        axes[1,1].plot(d[:,0]-origin,d[:,1:4]);axes[1,1].set(title="Physical specific force",ylabel="g")
    axes[1,0].legend(["x","y","z"]);axes[1,1].legend(["x","y","z"])
    for ax in axes.flat:ax.grid(True)
    axes[0,0].legend();axes[0,1].legend();fig.tight_layout()
    fig.savefig(a.output/"flight.png",dpi=150);plt.close(fig)
    requested=np.asarray(r["goals"],dtype=float)
    accepted=np.asarray(r.get("accepted_goals",r["goals"]),dtype=float)
    requested_evidence={"unavailable":"No task trigger/trajectory evidence"}
    lio_file=a.output/"lio.tum"
    if r.get("native_trigger_sent") and r["arrivals"] and lio_file.stat().st_size:
        requested_evidence=ordered_arrivals(np.loadtxt(lio_file,ndmin=2),requested[1:],r["arrivals"][0][1])
        truth_file=a.output/"truth.tum"
        if truth_file.stat().st_size:
            physics=np.loadtxt(truth_file,ndmin=2)
            requested_evidence["physics_requested"]=ordered_arrivals(physics,requested[1:],r["arrivals"][0][1])
            requested_evidence["physics_accepted"]=ordered_arrivals(physics,accepted[1:],r["arrivals"][0][1])
    (a.output/"requested_waypoint_evidence.json").write_text(json.dumps(requested_evidence,indent=2)+"\n")
    shifts=np.linalg.norm(accepted-requested,axis=1)
    adjustments=[{"index":int(i),"requested":requested[i].tolist(),"accepted":accepted[i].tolist(),"offset_m":float(shifts[i])} for i in np.flatnonzero(shifts>1e-6)]
    text=["# PX4 SITL validation","",f"Status: **{r['status']}**; stage: {r['stage']}.",
        f"Accepted-goal error: {r['goal_error_m']} m. Observed arrivals: {len(r['arrivals'])}/{len(r['goals'])}.",
        f"Native module binding: {r.get('module_alignment',{})}.",
        f"Native EGO goal adjustments (not simulator-selected): {adjustments}.",
        f"Independent ordered arrivals at original requested waypoints (takeoff excluded): {requested_evidence}.",
        f"Minimum LIO distance to each accepted goal: {r.get('goal_min_lio_distance_m',[])} m.",
        f"Lidar frames: {r['lidar_frames']}; LIO frames: {r['lio_frames']}; predicted poses: {r['high_freq_poses']}.",
        "",f"Verified parameters: {r['parameter_checks']}.",f"Errors: {r['errors']}.",
        "",f"Native EKF evidence (flag: sample counts): {evidence}.",
        "",f"LIO comparison: {r['comparison']}.",f"Rendering wall time: {r['render_wall_s']}.",f"Pipeline timings: {r.get('sensor_pipeline',{})}.",
        "","![Flight](flight.png)","","## Scope",""]+["- "+x for x in r["limits"]]
    (a.output/"report.md").write_text("\n".join(text)+"\n")
    print(a.output/"report.md")
if __name__=="__main__":main()
