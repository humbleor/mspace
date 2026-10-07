#!/usr/bin/env python3
"""Reconstruct sparse ray-trace evidence from replayed LIO poses/world clouds."""
import argparse
from pathlib import Path
import time
import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation
from common import write_json
from run_replay import cloud_xyz
from coverage_map import CoverageBuilder

def select_endpoints(xyz,origin,maximum=1200,resolution_deg=.8):
    delta=xyz-origin;length=np.linalg.norm(delta,axis=1)
    good=np.isfinite(delta).all(axis=1)&(length>=.5)
    xyz=xyz[good];delta=delta[good];length=length[good]
    az=np.floor((np.arctan2(delta[:,1],delta[:,0])+np.pi)/np.deg2rad(resolution_deg)).astype(int)
    el=np.floor((np.arcsin(np.clip(delta[:,2]/length,-1,1))+np.pi/2)/np.deg2rad(resolution_deg)).astype(int)
    keys=el*int(np.ceil(360/resolution_deg))+az
    order=np.lexsort((length,keys));_,first=np.unique(keys[order],return_index=True);chosen=order[first]
    if len(chosen)>maximum:chosen=chosen[np.linspace(0,len(chosen)-1,maximum).astype(int)]
    return xyz[chosen]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("observations",type=Path);ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--voxel",type=float,default=.3);ap.add_argument("--max-rays",type=int,default=1200)
    ap.add_argument("--max-range",type=float,default=30);ap.add_argument("--config",type=Path)
    args=ap.parse_args()
    if args.voxel<=0 or args.max_rays<1 or args.max_range<=.5:ap.error("Invalid geometry limits")
    if args.output.exists() and any(args.output.iterdir()):ap.error("output must be empty")
    args.output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[3]
    config=args.config or root/"Modules/fast_lio2/config/mid360.yaml"
    T=np.asarray(yaml.safe_load(config.read_text())["mapping"]["extrinsic_T"])
    poses=[]
    with rosbag.Bag(str(args.observations)) as bag:
        for _,m,_ in bag.read_messages(topics=["/lio/odom"]):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            poses.append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    if not poses:raise ValueError("No LIO poses")
    poses=np.asarray(poses);poses=poses[np.argsort(poses[:,0])]
    builder=CoverageBuilder(args.voxel);origins=[];skipped=0;rays=0;frames=0;started=time.monotonic()
    with rosbag.Bag(str(args.observations)) as bag:
        for _,m,_ in bag.read_messages(topics=["/lio/cloud"]):
            stamp=m.header.stamp.to_sec();j=np.searchsorted(poses[:,0],stamp)
            choices=[i for i in (j-1,j) if 0<=i<len(poses)]
            j=min(choices,key=lambda i:abs(poses[i,0]-stamp))
            if abs(poses[j,0]-stamp)>.02:skipped+=1;continue
            origin=poses[j,1:4]+Rotation.from_quat(poses[j,4:8]).as_matrix()@T
            points=select_endpoints(cloud_xyz(m),origin,args.max_rays)
            builder.update(origin,points,max_range=args.max_range)
            origins.append(np.r_[stamp,origin]);rays+=len(points);frames+=1
            if frames%200==0:print("Coverage: %d frames, %d selected rays"%(frames,rays),flush=True)
    coverage=builder.export();coverage.save(args.output/"coverage.npz")
    np.savetxt(args.output/"sensor_origins.csv",origins,delimiter=",",header="stamp,x,y,z")
    write_json(args.output/"coverage.json",{
        "source":str(args.observations.resolve()),"source_frame":"LIO world estimate; not independent ground truth",
        "voxel_m":args.voxel,"max_range_m":args.max_range,"max_rays_per_scan":args.max_rays,
        "frames":frames,"skipped_pose_mismatch":skipped,"selected_rays":rays,"cells":len(coverage.indices),
        "ray_trace_cells":int(((coverage.flags&1)>0).sum()),"surface_endpoint_cells":int(((coverage.flags&2)>0).sum()),
        "sensor_position_cells":int(((coverage.flags&4)>0).sum()),"wall_s":time.monotonic()-started,
        "limits":["Ray evidence does not certify a complete voxel or UAV footprint as free.",
                  "Unknown cells remain unknown; no-return beams or space beyond surfaces are never filled as free.",
                  "Subsampled observed-return endpoints provide a lower bound on captured beam evidence.",
                  "Sensor origin approximated at scan end after LIO deskew; per-point acquisition origins unavailable here.",
                  "LIO drift and registered-map error also affect this evidence."]})
    print(args.output/"coverage.json",flush=True)
if __name__=="__main__":main()
