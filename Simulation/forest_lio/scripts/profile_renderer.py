#!/usr/bin/env python3
"""Known surfel-model oracle and isolated PCD query profiles; no real-flight accuracy claim."""
import argparse
from pathlib import Path
import time
import numpy as np
import yaml
from renderer import PointRenderer,read_pcd
from bvh_renderer import BVHRenderer,exhaustive_ranges
from generate_mid360_bag import templates_from_bag,trajectory,trajectory_pose_samples
from common import write_json,summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map",type=Path,required=True);ap.add_argument("--bag",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True);ap.add_argument("--repeats",type=int,default=12)
    args=ap.parse_args()
    if args.repeats<3:ap.error("Need at least 3 repeats")
    if args.output.exists() and any(args.output.iterdir()):ap.error("output must be empty")
    args.output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[3]
    config=yaml.safe_load((root/"Modules/fast_lio2/config/mid360.yaml").read_text())
    T=np.asarray(config["mapping"]["extrinsic_T"]);E=np.asarray(config["mapping"]["extrinsic_R"]).reshape(3,3)
    xyz=read_pcd(args.map)
    templates=templates_from_bag(args.bag,"/livox/lidar_192_168_2_181",24,.1,window=(40,120))
    bvh=BVHRenderer(xyz,.2,30,threads=1)
    legacy=PointRenderer(xyz,.5,.2,30)
    checks=[];profiles=[]
    for t in (0.,6.,8.):
        print("Validating forest scan at %.1fs"%t,flush=True)
        rays,offsets,_,_=templates[3]
        times=t+offsets*1e-9
        positions,rotations=trajectory_pose_samples(times,12,3,.3)
        origins=positions+np.einsum("nij,j->ni",rotations,T)
        directions=np.einsum("nij,nj->ni",rotations,rays@E.T)
        exact=bvh.scan_world(directions,origins,.5)
        chosen=np.linspace(0,len(rays)-1,96).astype(int)
        began=time.perf_counter()
        oracle=exhaustive_ranges(xyz,bvh.normals,.2,directions[chosen],origins[chosen],.5,30)
        oracle_s=time.perf_counter()-began
        delta=np.abs(exact[chosen]-oracle)
        if np.any(delta>1e-8):raise AssertionError("BVH disagrees with all-disks oracle")
        old=np.zeros(len(rays));groups=np.floor(offsets*1e-6/10).astype(int)
        began=time.perf_counter()
        for g in np.unique(groups):
            m=groups==g;mid=(float(offsets[m].min())+float(offsets[m].max()))*.5e-9
            p,_,_,R,_=trajectory(t+mid,12,3,.3)
            old[m]=legacy.scan(rays[m],p+R@T,R@E,.5)
        old_wall=time.perf_counter()-began
        # Compare both models at EXACTLY matched point-time origins/directions on oracle rays.
        old_oracle=np.array([legacy.scan(rays[i:i+1],origins[i],rotations[i]@E,.5)[0] for i in chosen])
        old_errors=np.abs(old_oracle-oracle);both=(old>0)&(exact>0)
        checks.append({"time_s":t,"oracle_rays":len(chosen),"oracle_wall_s":oracle_s,
            "bvh_oracle_max_abs_m":float(delta.max()),
            "legacy_matched_time_oracle_hit_disagreement":float(np.mean((old_oracle>0)!=(oracle>0))),
            "legacy_matched_time_oracle_abs_error_m":summary(old_errors),
            "legacy_full_10ms_wall_s":old_wall,
            "legacy_vs_bvh_hit_disagreement":float(np.mean((old>0)!=(exact>0))),
            "legacy_vs_bvh_common_hit_abs_difference_m":summary(np.abs(old[both]-exact[both]))})
        for threads in (1,2,4):
            bvh.threads=threads;bvh.scan_world(directions,origins,.5) # warmup
            stages=[]
            for repeat in range(args.repeats):
                start=time.perf_counter()
                p,R=trajectory_pose_samples(times,12,3,.3)
                o=p+np.einsum("nij,j->ni",R,T);d=np.einsum("nij,nj->ni",R,rays@E.T)
                transformed=time.perf_counter()
                result=bvh.scan_world(d,o,.5)
                elapsed=time.perf_counter()-start
                np.testing.assert_allclose(result,exact,atol=1e-8,rtol=0)
                stages.append(dict(bvh.last_profile,pose_transform_s=transformed-start,frame_s=elapsed))
            profiles.append({"time_s":t,"threads":threads,"rays":len(rays),
                **{key:summary([row[key] for row in stages]) for key in
                   ("pose_transform_s","input_prepare_s","native_query_s","frame_s","aabb_tests","disk_tests")}})
    # At fixed disks, angular resolution does not exist in BVH; world index remains valid after motion.
    density=[]
    rays,offsets,_,_=templates[3];p,R=trajectory_pose_samples(8+offsets*1e-9,12,3,.3)
    o=p+np.einsum("nij,j->ni",R,T);d=np.einsum("nij,nj->ni",R,rays@E.T)
    bvh.threads=4;reference=bvh.scan_world(d,o,.5)
    _,keep=np.unique(np.floor(xyz/.3).astype(int),axis=0,return_index=True)
    for label,points,radius in [("radius_0.1",xyz,.1),("voxel_0.3m",xyz[keep],.2)]:
        candidate=BVHRenderer(points,radius,30,threads=4)
        out=candidate.scan_world(d,o,.5);both=(out>0)&(reference>0)
        density.append({"variant":label,"points":len(points),
            "hit_disagreement_fraction":float(np.mean((out>0)!=(reference>0))),
            "common_hit_abs_difference_m":summary(np.abs(out[both]-reference[both]))})
    report={"map":str(args.map.resolve()),"template_bag":str(args.bag.resolve()),
        "map_points":len(xyz),"ray_count":len(rays),"repeats":args.repeats,
        "preprocess_s":bvh.preprocess_s,"index_build_s":bvh.build_s,"native_library":bvh.library,
        "oracle":"Independent all-surfel plane/disk intersections; exact for the modeled disks, not forest ground truth.",
        "checks":checks,"profiles":profiles,"geometry_sensitivity":density,
        "limitations":["PCA normals and fixed-radius disks still approximate physical forest surfaces.",
                       "Thin branches, undersampled gaps and changed geometry are not eliminated by exact querying.",
                       "Return-direction templates omit original no-return beam directions.",
                       "Isolated render timing does not include ROS, LIO, EGO or the entire closed-loop wall time."]}
    write_json(args.output/"profile.json",report)
    print(args.output/"profile.json",flush=True)
if __name__=="__main__":main()
