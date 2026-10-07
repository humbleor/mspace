"""Recorded-world/local transforms and conservative voxel evidence around a spherical UAV proxy."""
import numpy as np
from scipy.spatial.transform import Rotation
from common import summary

def local_frame(position,quaternion):
    rotation=Rotation.from_quat(quaternion).as_matrix()
    heading=np.arctan2(rotation[1,0],rotation[0,0])
    return np.asarray(position,dtype=float),Rotation.from_euler("z",heading).as_matrix()

def to_local(points,origin,rotation):
    return (np.asarray(points)-origin)@rotation

def to_world(points,origin,rotation):
    return np.asarray(points)@rotation.T+origin

def footprint_evidence(coverage,positions,radius=.25):
    if not np.isfinite(radius) or radius<=0:raise ValueError("positive footprint radius required")
    voxel=coverage.voxel
    n=int(np.ceil(radius/voxel))+1
    offsets=np.stack(np.meshgrid(*([np.arange(-n,n+1)]*3),indexing="ij"),axis=-1).reshape(-1,3)
    rows=[]
    for p in np.asarray(positions).reshape(-1,3):
        keys=np.floor(p/voxel).astype(int)+offsets
        centers=(keys+.5)*voxel
        # Exact sphere/AABB overlap, including boundary touches. Evidence does not certify volume.
        separation=np.maximum(np.abs(centers-p)-voxel/2,0)
        centers=centers[np.sum(separation*separation,axis=1)<=radius*radius+1e-12]
        flags=coverage.classify(centers)
        rows.append([int(coverage.classify(p)[0]==0),len(flags),int((flags==0).sum()),int(((flags&2)>0).sum())])
    rows=np.asarray(rows,dtype=float).reshape(-1,4)
    report={"radius_m":radius,"samples":len(rows),
      "center_unknown_samples":int(rows[:,0].sum()),
      "footprint_has_unknown_samples":int((rows[:,2]>0).sum()),
      "unknown_cell_fraction":summary(rows[:,2]/rows[:,1]) if len(rows) else {},
      "footprint_endpoint_evidence_samples":int((rows[:,3]>0).sum()),
      "meaning":"Sphere proxy vs intersecting evidence voxels; ray evidence never certifies a free volume or safe flight."}
    return rows,report


def planner_config_for_route(config,goals):
    result=dict(config)
    points=np.vstack([np.zeros(3),np.asarray(goals,dtype=float).reshape(-1,3)])
    for axis,k in enumerate(("x","y")):
        half=max(abs(points[:,axis]))+float(result["grid_map/local_update_range_"+k])+1.
        result["grid_map/map_size_"+k]=float(max(float(result["grid_map/map_size_"+k]),2.*np.ceil(half)))
    return result
