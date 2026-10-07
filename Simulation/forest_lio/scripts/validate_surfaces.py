#!/usr/bin/env python3
"""Known physical geometry and matched-query forest sensitivity for adaptive surfel support."""
import argparse
from pathlib import Path
import time
import numpy as np
from bvh_renderer import BVHRenderer,exhaustive_ranges
from renderer import read_pcd
from common import summary,write_json

def geometry_metrics(renderer,rays,truth):
    actual=renderer.scan_world(rays,[0,0,0],.5)
    expected_hit=truth>0;hit=actual>0;common=hit&expected_hit
    return {"rays":len(rays),"false_positive_fraction":float(np.mean(hit[~expected_hit])) if np.any(~expected_hit) else 0.,
        "false_negative_fraction":float(np.mean(~hit[expected_hit])) if np.any(expected_hit) else 0.,
        "common_true_hit_error_m":summary(np.abs(actual[common]-truth[common]))}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map",type=Path,required=True);ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists() and any(args.output.iterdir()):ap.error("output must be empty")
    args.output.mkdir(parents=True,exist_ok=True);known=[]
    # Finite thin planar strip, avoid its top/bottom boundary.
    y,z=np.meshgrid(np.linspace(-.02,.02,5),np.linspace(-1,1,81))
    points=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()])
    target=np.linspace(-.3,.3,401);rays=np.column_stack([np.full(len(target),3),target,np.zeros(len(target))])
    truth=np.where(np.abs(target)<=.02,np.linalg.norm(rays,axis=1),0.)
    for model in ("fixed","adaptive"):
        r=BVHRenderer(points,.2,10,surface_model=model,threads=1)
        known.append({"scene":"thin_strip_4cm","model":model,**geometry_metrics(r,rays,truth)})
    # Thin vertical cylinder, analytic infinite-cylinder intersections at z=0.
    a,z=np.meshgrid(np.linspace(0,2*np.pi,80,endpoint=False),np.linspace(-.5,.5,101));radius=.08
    points=np.column_stack([3+radius*np.cos(a.ravel()),radius*np.sin(a.ravel()),z.ravel()])
    rays=np.column_stack([np.full(len(target),3),target,np.zeros(len(target))]);d=rays/np.linalg.norm(rays,axis=1)[:,None]
    b=-6*d[:,0];disc=b*b-4*(9-radius*radius);valid=disc>=0
    truth=np.zeros(len(rays));truth[valid]=(-b[valid]-np.sqrt(disc[valid]))/2
    for model in ("fixed","adaptive"):
        r=BVHRenderer(points,.2,10,surface_model=model,threads=1)
        known.append({"scene":"cylinder_radius_8cm","model":model,**geometry_metrics(r,rays,truth)})
    # A wall with a known circular hole and a back wall. Report wrong front/back classification.
    y,z=np.meshgrid(np.linspace(-1,1,81),np.linspace(-1,1,81))
    front=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()])
    front=front[np.linalg.norm(front[:,1:],axis=1)>=.25]
    back=np.column_stack([np.full(y.size,6.),y.ravel(),z.ravel()]);points=np.vstack([front,back])
    y,z=np.meshgrid(np.linspace(-.35,.35,31),np.linspace(-.35,.35,31))
    rays=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()]);front_hit=np.linalg.norm(rays[:,1:],axis=1)>=.25
    truth=np.linalg.norm(rays,axis=1)*np.where(front_hit,1,2)
    for model in ("fixed","adaptive"):
        r=BVHRenderer(points,.2,10,surface_model=model,threads=1)
        metrics=geometry_metrics(r,rays,truth);actual=r.scan_world(rays,[0,0,0],.5)
        metrics["incorrect_front_back_fraction"]=float(np.mean((actual>0)&((actual<4)!=front_hit)))
        known.append({"scene":"wall_hole_25cm","model":model,**metrics})
    xyz=read_pcd(args.map);rng=np.random.default_rng(53)
    rays=rng.normal(size=(4000,3));rays/=np.linalg.norm(rays,axis=1)[:,None]
    origin=np.array([-.011,-.02329,.04412]);forest=[]
    _,indices=np.unique(np.floor(xyz/.3).astype(int),axis=0,return_index=True)
    for model in ("fixed","adaptive"):
        baseline=BVHRenderer(xyz,.2,30,surface_model=model,threads=4)
        expected=baseline.scan_world(rays,origin,.5)
        selected=np.linspace(0,len(rays)-1,64).astype(int)
        oracle=exhaustive_ranges(xyz,baseline.normals,baseline.radii,rays[selected],origin,.5,30,kinds=baseline.kinds)
        np.testing.assert_allclose(expected[selected],oracle,atol=1e-8,rtol=0)
        for label,points,radius in [("baseline",xyz,.2),("radius_cap_0.1",xyz,.1),("voxel_0.3m",xyz[indices],.2)]:
            r=baseline if label=="baseline" else BVHRenderer(points,radius,30,surface_model=model,threads=4)
            samples=[]
            for repeat in range(6):
                began=time.perf_counter();actual=r.scan_world(rays,origin,.5);samples.append(time.perf_counter()-began)
            common=(actual>0)&(expected>0)
            forest.append({"model":model,"variant":label,"points":len(points),
                "hit_fraction":float(np.mean(actual>0)),"hit_disagreement_fraction":float(np.mean((actual>0)!=(expected>0))),
                "common_hit_difference_m":summary(np.abs(actual[common]-expected[common])),
                "query_s":summary(samples),"surface_model":r.surface_report})
        np.savez_compressed(args.output/(model+"_surfaces.npz"),xyz=xyz,normals=baseline.normals,
                            radii=baseline.radii,kinds=baseline.kinds)
    write_json(args.output/"surfaces.json",{"map":str(args.map.resolve()),"known_geometry":known,"forest":forest,
        "limits":["Known geometry tests evaluate model shape; forest differences have no independent range truth.",
                  "Low-confidence points become small sphere proxies, not certified physical obstacles.",
                  "Adaptive support cannot reconstruct unobserved surfaces or make sparse holes free space."]})
    print(args.output/"surfaces.json")
if __name__=="__main__":main()
