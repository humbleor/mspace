"""Evidence-limited PCD cylinder extraction and Gazebo collision geometry.

Fits observed vertical shafts; coarse branch proxies require linear tube support.
Cylinder completion is inferred from observed support, not verified unseen surface.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import label
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

DEFAULTS = {"voxel":.07,"column":.25,"height_bin":.25,"min_height":1.5,
            "min_height_bins":7,"min_points":35,"min_radius":.06,"max_radius":.55,
            "max_fit_error":.08,"min_angular_bins":5,"branch_cell":1.,
            "min_branch_length":.6,"max_branch_radius":.16,"max_colliders":500}


def extract_geometry(points, settings=None):
    cfg=dict(DEFAULTS); cfg.update(settings or {})
    for k,v in cfg.items():
        if not np.isfinite(v) or v<=0:raise ValueError("Positive finite extraction settings required: "+k)
    xyz=np.asarray(points,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError("Finite nonempty Nx3 forest points required")
    started=time.monotonic()
    _,indices=np.unique(np.floor(xyz/cfg["voxel"]).astype(np.int64),axis=0,return_index=True)
    p=xyz[np.sort(indices)]
    keys=np.floor(p[:,:2]/cfg["column"]).astype(int)
    zkeys=np.floor(p[:,2]/cfg["height_bin"]).astype(int)
    occupied=np.unique(np.column_stack([keys,zkeys]),axis=0)
    xy,count=np.unique(occupied[:,:2],axis=0,return_counts=True)
    candidates=xy[count>=cfg["min_height_bins"]]
    colliders=[]; rejected=[]
    if len(candidates):
        origin=candidates.min(axis=0); extent=candidates.max(axis=0)-origin+1
        if np.prod(extent)>2000000:raise ValueError("Extraction extent too large")
        mask=np.zeros(tuple(extent),dtype=bool)
        mask[tuple((candidates-origin).T)]=True
        labels,n=label(mask,np.ones((3,3)))
        tree=cKDTree(p[:,:2])
        for group in range(1,n+1):
            cells=np.argwhere(labels==group)+origin
            center_seed=(cells.mean(axis=0)+.5)*cfg["column"]
            radius_search=max(.6,float(np.linalg.norm((cells.max(axis=0)-cells.min(axis=0)+1)*cfg["column"])/2+.3))
            patch=p[tree.query_ball_point(center_seed,radius_search)]
            if len(patch)<cfg["min_points"]:continue
            # Use supported z bins, trimming peripheral foliage before fitting.
            heights=np.quantile(patch[:,2],[.05,.95])
            if heights[1]-heights[0]<cfg["min_height"]:continue
            patch=patch[(patch[:,2]>=heights[0])&(patch[:,2]<=heights[1])]
            initial_radius=np.clip(np.median(np.linalg.norm(patch[:,:2]-center_seed,axis=1)),cfg["min_radius"],cfg["max_radius"])
            fit=least_squares(lambda v:np.linalg.norm(patch[:,:2]-v[:2],axis=1)-v[2],
                              [*center_seed,initial_radius],loss="soft_l1",f_scale=.03,
                              bounds=([center_seed[0]-radius_search,center_seed[1]-radius_search,cfg["min_radius"]],
                                      [center_seed[0]+radius_search,center_seed[1]+radius_search,cfg["max_radius"]]))
            center=fit.x[:2]; radius=float(fit.x[2])
            error=np.abs(np.linalg.norm(patch[:,:2]-center,axis=1)-radius)
            good=error<=cfg["max_fit_error"]
            supported=patch[good]
            angular=np.floor((np.arctan2(supported[:,1]-center[1],supported[:,0]-center[0])+np.pi)/(2*np.pi)*12).astype(int)
            bins=len(np.unique(angular))
            if len(supported)<cfg["min_points"] or good.mean()<.55 or bins<cfg["min_angular_bins"] or radius>=cfg["max_radius"]-.001:
                rejected.append({"seed":center_seed.tolist(),"inlier_fraction":float(good.mean()),"angular_bins":bins})
                continue
            lo,hi=np.quantile(supported[:,2],[.02,.98])
            if hi-lo<cfg["min_height"]:continue
            colliders.append({"kind":"trunk","a":[*center,float(lo)],"b":[*center,float(hi)],
                              "radius":radius,"support_points":len(supported),
                              "fit_median_error_m":float(np.median(error[good])),"angular_bins":bins})
    # Coarse branches: local PCA tube fits outside accepted trunk surfaces. Do
    # not fit canopy blobs or bridge distant observations into large meshes.
    keep=np.ones(len(p),dtype=bool)
    for c in colliders:
        keep &= ~((np.linalg.norm(p[:,:2]-np.asarray(c["a"][:2]),axis=1)<c["radius"]+.12)&
                  (p[:,2]>=c["a"][2]-.1)&(p[:,2]<=c["b"][2]+.1))
    remaining=p[keep]
    if len(remaining):
        keys=np.floor(remaining/cfg["branch_cell"]).astype(int)
        unique,inverse,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
        order=np.argsort(inverse); starts=np.r_[0,np.cumsum(counts)]
        for i in np.flatnonzero(counts>=25):
            patch=remaining[order[starts[i]:starts[i+1]]]
            center=patch.mean(axis=0); covariance=np.cov((patch-center).T)
            eig,vec=np.linalg.eigh(covariance); axis=vec[:,-1]
            if eig[-1]<8*max(eig[-2],1e-9):continue
            projection=(patch-center)@axis
            radial=np.linalg.norm(patch-center-projection[:,None]*axis,axis=1)
            radius=float(np.quantile(radial,.9)); low,high=np.quantile(projection,[.02,.98])
            if radius<.035 or radius>cfg["max_branch_radius"] or high-low<cfg["min_branch_length"]:continue
            kind="stem" if abs(axis[2])>.85 else "branch"
            colliders.append({"kind":kind,"a":(center+low*axis).tolist(),"b":(center+high*axis).tolist(),
                              "radius":radius,"support_points":len(patch),"fit_method":"local PCA tube proxy",
                              "pca_linearity":float(eig[-1]/max(eig[-2],1e-9))})
    if len(colliders)>cfg["max_colliders"]:raise ValueError("Collider budget exceeded; no silent truncation")
    return {"schema_version":1,"frame":"world","source_points":len(xyz),"fit_points":len(p),
            "local_map_sha256":hashlib.sha256(np.ascontiguousarray(xyz,dtype="<f8").tobytes()).hexdigest(),
            "parameters":cfg,"colliders":colliders,"rejected_trunk_candidates":rejected,
            "counts":{k:sum(c["kind"]==k for c in colliders) for k in ("trunk","stem","branch")},
            "extraction_wall_s":time.monotonic()-started,
            "limits":["Cylinders complete observed arcs by assumption; rejected or unseen geometry remains unmodeled.",
                      "Branch PCA candidates and trunk fits are not semantic ground truth.",
                      "No foliage flexibility, structural damage or calibrated wood response."]}


def geometry_model(geometry, offset=(0,0,0)):
    model=ET.Element("model",name="forest_collision_geometry")
    ET.SubElement(model,"static").text="true"
    ET.SubElement(model,"pose").text=" ".join(map(str,offset))+" 0 0 0"
    for i,c in enumerate(geometry["colliders"]):
        a=np.asarray(c["a"],dtype=float); b=np.asarray(c["b"],dtype=float); d=b-a
        length=float(np.linalg.norm(d))
        if not np.isfinite([*a,*b,c["radius"]]).all() or length<=0 or c["radius"]<=0:raise ValueError("Invalid collider")
        axis=d/length; cross=np.cross([0,0,1],axis); sine=np.linalg.norm(cross)
        if sine<1e-12:rotation=Rotation.from_rotvec([np.pi,0,0] if axis[2]<0 else [0,0,0])
        else:rotation=Rotation.from_rotvec(cross/sine*np.arctan2(sine,axis[2]))
        pose=np.r_[(a+b)/2,rotation.as_euler("xyz")]
        link=ET.SubElement(model,"link",name=f"{c['kind']}_{i:04d}")
        ET.SubElement(link,"pose").text=" ".join(map(str,pose))
        for kind in ("collision","visual"):
            item=ET.SubElement(link,kind,name=kind)
            cylinder=ET.SubElement(ET.SubElement(item,"geometry"),"cylinder")
            ET.SubElement(cylinder,"radius").text=str(c["radius"])
            ET.SubElement(cylinder,"length").text=str(length)
            if kind=="collision":
                surface=ET.SubElement(item,"surface")
                ode=ET.SubElement(ET.SubElement(surface,"contact"),"ode")
                for key,value in {"kp":1e6,"kd":100,"max_vel":1,"min_depth":.0001}.items():
                    ET.SubElement(ode,key).text=str(value)
            else:
                material=ET.SubElement(item,"material")
                ET.SubElement(material,"diffuse").text="0.35 0.19 0.08 1"
    return model


def plot_geometry(points,geometry,path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    sampled=points[::max(1,int(np.ceil(len(points)/150000)))]
    fig,axes=plt.subplots(1,2,figsize=(13,6))
    axes[0].scatter(sampled[:,0],sampled[:,1],s=.3,c="gray",alpha=.3)
    axes[1].scatter(sampled[:,0],sampled[:,2],s=.3,c="gray",alpha=.3)
    for c in geometry["colliders"]:
        a=np.asarray(c["a"]);b=np.asarray(c["b"])
        color="brown" if c["kind"] in ("trunk","stem") else "green"
        if c["kind"] in ("trunk","stem"):axes[0].add_patch(Circle(a[:2],c["radius"],fill=False,color=color,lw=1))
        else:axes[0].plot([a[0],b[0]],[a[1],b[1]],color=color,lw=2)
        axes[1].plot([a[0],b[0]],[a[2],b[2]],color=color,lw=1)
    axes[0].set(xlabel="local x (m)",ylabel="local y (m)");axes[0].set_aspect("equal")
    axes[1].set(xlabel="local x (m)",ylabel="local z (m)")
    for ax in axes:ax.grid(True)
    fig.suptitle(f"PCD proxies: {geometry['counts']['trunk']} trunks / {geometry['counts'].get('stem',0)} short stems / {geometry['counts']['branch']} branches")
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(path,dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    p.add_argument("--takeoff-height",type=float,default=1.5)
    a=p.parse_args()
    if a.output.exists():p.error("Use a new output directory")
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"forest_lio/scripts"))
    from renderer import read_pcd
    from scene import to_local
    scene=json.loads(a.scene.read_text()); origin=np.asarray(scene["origin_world"],dtype=float);origin[2]-=a.takeoff_height
    points=to_local(read_pcd(scene["map"]),origin,np.asarray(scene["rotation_world_from_local"]))
    geometry=extract_geometry(points);geometry.update(scene=str(a.scene.resolve()),map_origin_world=origin.tolist())
    a.output.mkdir(parents=True)
    (a.output/"geometry.json").write_text(json.dumps(geometry,indent=2)+"\n")
    root=ET.Element("sdf",version="1.6");root.append(geometry_model(geometry))
    ET.ElementTree(root).write(a.output/"forest_collisions.sdf",encoding="unicode")
    plot_geometry(points,geometry,a.output/"geometry_overview.png")
    print(json.dumps(geometry["counts"]));print(a.output/"geometry.json")
    if not geometry["counts"]["trunk"]:raise SystemExit("No supported trunks found")


if __name__=="__main__":main()
