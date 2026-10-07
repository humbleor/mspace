"""Local sampling support and PCA reliability; uncertain geometry is retained as small point spheres."""
import numpy as np
from scipy.spatial import cKDTree
from common import summary

def adaptive_surfaces(xyz,max_radius=.2,min_radius=.01,spacing_scale=1.25,tree=None):
    xyz=np.asarray(xyz,dtype=float)
    if len(xyz)<8:raise ValueError("Need at least eight points for local support")
    if not np.isfinite(max_radius) or max_radius<=0 or min_radius<=0 or spacing_scale<=0:raise ValueError("positive support parameters required")
    min_radius=min(min_radius,max_radius)
    tree=tree if tree is not None else cKDTree(xyz);normals=np.empty_like(xyz);radii=np.empty(len(xyz));kinds=np.zeros(len(xyz),dtype="i4")
    scatter=np.empty(len(xyz));anisotropy=np.empty(len(xyz));spacing=np.empty(len(xyz))
    for start in range(0,len(xyz),20000):
        points=xyz[start:start+20000];distance,neighbors=tree.query(points,k=min(16,len(xyz)))
        local=xyz[neighbors];delta=local-local.mean(axis=1,keepdims=True)
        covariance=np.einsum("nki,nkj->nij",delta,delta)/local.shape[1]
        values,vectors=np.linalg.eigh(covariance);values=np.maximum(values,0)
        noise=values[:,0]/np.maximum(values.sum(axis=1),1e-12)
        shape=values[:,1]/np.maximum(values[:,2],1e-12)
        nearest=distance[:,1:min(7,distance.shape[1])]
        local_spacing=np.median(np.where(nearest>1e-6,nearest,np.nan),axis=1)
        local_spacing=np.nan_to_num(local_spacing,nan=min_radius)
        # Cap support across a locally thin strip; avoid turning narrow branches into large disks.
        minor_support=1.5*np.sqrt(values[:,1])
        supported=np.minimum(spacing_scale*local_spacing,minor_support)
        supported=np.clip(supported,min_radius,max_radius)
        reliable=(noise<=.08)&(shape>=.12)&(local_spacing>1e-6)
        # Uncertain normals never generate wide planes: retain localized obstacle proxies.
        supported[~reliable]=np.clip(.5*local_spacing[~reliable],min(.005,max_radius),min(.04,max_radius))
        end=start+len(points)
        normals[start:end]=vectors[:,:,0];radii[start:end]=supported;kinds[start:end]=(~reliable).astype("i4")
        scatter[start:end]=noise;anisotropy[start:end]=shape;spacing[start:end]=local_spacing
    report={"model":"adaptive","radii_m":summary(radii),"local_spacing_m":summary(spacing),
            "scatter":summary(scatter),"anisotropy":summary(anisotropy),
            "disk_points":int((kinds==0).sum()),"uncertain_sphere_points":int((kinds==1).sum()),
            "parameters":{"max_radius":max_radius,"min_radius":min_radius,"spacing_scale":spacing_scale,
                          "max_scatter":.08,"min_anisotropy":.12,"uncertain_max_radius":.04},
            "limits":"Sampling-based support is not a calibrated physical surface or proof of free space."}
    return normals,radii,kinds,report
