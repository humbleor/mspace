#!/usr/bin/env python3
"""Matched-ray sensitivity and speed: full raster/subscan vs one raster/frame."""
import argparse
import time
from pathlib import Path
import numpy as np
import yaml
from renderer import PointRenderer, read_pcd
from generate_mid360_bag import templates_from_bag, trajectory
from common import write_json, summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map",type=Path,required=True)
    ap.add_argument("--bag",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    a=ap.parse_args()
    root=Path(__file__).resolve().parents[3]
    cfg=yaml.safe_load((root/"Modules/fast_lio2/config/mid360.yaml").read_text())
    T=np.array(cfg["mapping"]["extrinsic_T"]); E=np.array(cfg["mapping"]["extrinsic_R"]).reshape(3,3)
    xyz=read_pcd(a.map)
    templates=templates_from_bag(a.bag,"/livox/lidar_192_168_2_181",24,.1,window=(40,120))
    rays,off,_,_=templates[3]
    results=[]
    def render(renderer, t, step, cached):
        ranges=np.zeros(len(rays))
        groups=np.floor(off.astype(float)*1e-6/step).astype(int)
        scene=None
        if cached:
            p,_,_,R,_=trajectory(t+.05,12,3,.3)
            scene=renderer.prepare(p+R@T,R@E)
        for g in np.unique(groups):
            m=groups==g
            dt=(float(off[m].min())+float(off[m].max()))*.5e-9
            p,_,_,R,_=trajectory(t+dt,12,3,.3)
            origin=p+R@T; rot=R@E
            ranges[m]=renderer.cast(rays[m],scene,origin,rot,.5) if cached else renderer.scan(rays[m],origin,rot,.5)
        return ranges
    base=PointRenderer(xyz,.5,.2,30)
    _,keep=np.unique(np.floor(xyz/.3).astype(int),axis=0,return_index=True)
    sparse=PointRenderer(xyz[keep],.5,.2,30)
    for t in (0.,6.,8.):
        began=time.monotonic(); ref=render(base,t,1,False); elapsed=time.monotonic()-began
        for label,renderer,step,cached in [
            ("full_raster_5ms",base,5,False),
            ("full_raster_10ms",base,10,False),
            ("cached_voxel_0.3m",sparse,10,True),
            ("cached_5ms",base,5,True),("cached_10ms",base,10,True),
            ("cached_20ms",base,20,True),
            ("cached_radius_0.1",PointRenderer(xyz,.5,.1,30),10,True),
            ("cached_resolution_1deg",PointRenderer(xyz,1.,.2,30),10,True)]:
            began=time.monotonic(); out=render(renderer,t,step,cached); wall=time.monotonic()-began
            both=(ref>0)&(out>0)
            results.append({"time_s":t,"variant":label,"wall_s":wall,"reference_wall_s":elapsed,
                            "hit_disagreement_fraction":float(np.mean((ref>0)!=(out>0))),
                            "common_hit_range_difference_m":summary(np.abs(ref[both]-out[both]))})
    write_json(a.output,{"reference":"Same map/rays/path, full raster per 1ms subscan; numerical reference, not real truth.",
                        "map_points":len(xyz),"ray_count":len(rays),"results":results,
                        "limits":"Cached angular candidate visibility can differ under translation; no claim of exact ray tracing."})
    print(a.output)
if __name__=="__main__":
    main()
