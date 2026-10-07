#!/usr/bin/env python3
"""Prepare an estimated-flight route and yaw-aligned local simulation frame."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from common import write_json
from renderer import read_pcd
from coverage_map import CoverageMap
from scene import local_frame,to_local,footprint_evidence

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--replay",type=Path,required=True)
    ap.add_argument("--coverage",type=Path,required=True)
    ap.add_argument("--window",type=float,nargs=2,default=[125,155])
    ap.add_argument("--bag-start",type=float,default=1770518476.561167,help="This dataset's original bag epoch")
    ap.add_argument("--spacing",type=float,default=1.5)
    ap.add_argument("--body-radius",type=float,default=.25)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    if not 0<=args.window[0]<args.window[1] or args.spacing<=0:ap.error("Invalid route parameters")
    if args.output.exists():ap.error("Refusing to overwrite scene")
    poses=np.loadtxt(args.replay/"replayed_lio.tum",ndmin=2)
    poses=poses[(poses[:,0]>=args.bag_start+args.window[0])&(poses[:,0]<=args.bag_start+args.window[1])]
    if len(poses)<2:raise ValueError("Insufficient route poses")
    p0,R=local_frame(poses[0,1:4],poses[0,4:8])
    local=to_local(poses[:,1:4],p0,R)
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(local,axis=0),axis=1))]
    samples=np.r_[np.arange(args.spacing,distance[-1],args.spacing),distance[-1]]
    goals=np.column_stack([np.interp(samples,distance,local[:,k]) for k in range(3)])
    route=np.vstack([np.zeros(3),goals])
    segments=np.vstack([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)) for a,b in zip(route[:-1],route[1:])])
    world=segments@R.T+p0
    tree=cKDTree(read_pcd(args.replay/"forest_map.pcd"))
    clearance=tree.query(world)[0]
    if clearance.min()<args.body_radius:raise ValueError("Reference goal polyline intersects point-map body proxy")
    coverage=CoverageMap.load(args.coverage)
    _,evidence=footprint_evidence(coverage,world,args.body_radius)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    write_json(args.output,{"map":str((args.replay/"forest_map.pcd").resolve()),
      "coverage":str(args.coverage.resolve()),"origin_world":p0,"rotation_world_from_local":R,
      "source_initial_pose":poses[0],"source_initial_rpy_deg":Rotation.from_quat(poses[0,4:8]).as_euler("xyz",degrees=True),
      "source_window_s":args.window,"source_bag_start":args.bag_start,"goals_local":goals,
      "reference_path_length_m":float(distance[-1]),"goal_polyline_length_m":float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),
      "minimum_reference_point_clearance_m":float(clearance.min()),"body_radius_m":args.body_radius,
      "reference_evidence":evidence,
      "limits":["Recorded poses are LIO estimates, not original flight truth.",
                "Initial roll/pitch are replaced by level attitude; yaw and position align to recorded-world frame.",
                "Goals define a mission; reference trajectory is never injected as LIO odometry or position commands.",
                "Point clearance and sparse evidence do not prove safe free space or collision-free swept volume."]})
    print(args.output,flush=True)
if __name__=="__main__":main()
