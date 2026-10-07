"""Exact nearest intersection of modeled surfel disks, using an immutable world BVH."""
import ctypes
import time
import numpy as np
from scipy.spatial import cKDTree
from renderer import normals_for_map
from native_build import load_shared
from surface_model import adaptive_surfaces

def native_library():
    lib,library=load_shared("surfel_bvh.cpp")
    ptr=ctypes.POINTER(ctypes.c_double)
    lib.surfel_create.argtypes=[ptr,ptr,ptr,ctypes.POINTER(ctypes.c_int32),ctypes.c_int];lib.surfel_create.restype=ctypes.c_void_p
    lib.surfel_destroy.argtypes=[ctypes.c_void_p]
    lib.surfel_cast.argtypes=[ctypes.c_void_p,ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_double,
                             ctypes.c_int,ptr,ctypes.POINTER(ctypes.c_uint64)]
    lib.surfel_cast.restype=None
    return lib,str(library)

def pointer(a):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

class BVHRenderer:
    def __init__(self,xyz,radius=.2,max_range=30,normals=None,threads=4,radii=None,kinds=None,surface_model="fixed"):
        self.xyz=np.ascontiguousarray(xyz,dtype=np.float64)
        if self.xyz.ndim!=2 or self.xyz.shape[1]!=3 or len(self.xyz)<1 or not np.isfinite(self.xyz).all():
            raise ValueError("Need finite nonempty Nx3 points")
        if not np.isfinite(radius) or not np.isfinite(max_range) or radius<=0 or max_range<=0 or threads<1:raise ValueError("radius/range/threads must be positive")
        self.radius=float(radius);self.max_range=float(max_range);self.threads=int(threads)
        started=time.perf_counter();self.tree=cKDTree(self.xyz)
        if surface_model=="adaptive":
            if any(x is not None for x in (normals,radii,kinds)):raise ValueError("Adaptive model cannot also supply explicit geometry")
            n,radii,kinds,self.surface_report=adaptive_surfaces(self.xyz,max_radius=radius,tree=self.tree)
        elif surface_model=="fixed":
            n=normals_for_map(self.xyz,self.tree) if normals is None else np.asarray(normals,dtype=float)
            self.surface_report={"model":"fixed"}
        else:raise ValueError("Unknown surface model")
        self.surface_model=surface_model
        self.radii=np.ascontiguousarray(np.full(len(self.xyz),radius) if radii is None else radii,dtype=float)
        self.kinds=np.ascontiguousarray(np.zeros(len(self.xyz)) if kinds is None else kinds,dtype="<i4")
        if self.radii.shape!=(len(self.xyz),) or not np.isfinite(self.radii).all() or np.any(self.radii<=0):raise ValueError("Invalid per-point radii")
        if self.kinds.shape!=(len(self.xyz),) or np.any((self.kinds!=0)&(self.kinds!=1)):raise ValueError("Invalid geometry kinds")
        if n.shape!=self.xyz.shape or not np.isfinite(n).all() or np.any(np.linalg.norm(n,axis=1)<1e-12):
            raise ValueError("Need finite nonzero normals matching XYZ")
        self.normals=np.ascontiguousarray(n/np.linalg.norm(n,axis=1)[:,None])
        self.preprocess_s=time.perf_counter()-started
        self.lib,self.library=native_library()
        started=time.perf_counter()
        self.handle=self.lib.surfel_create(pointer(self.xyz),pointer(self.normals),pointer(self.radii),self.kinds.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),len(self.xyz))
        if not self.handle:raise RuntimeError("Native BVH construction failed")
        self.build_s=time.perf_counter()-started;self.last_profile={}
    def close(self):
        if getattr(self,"handle",None):
            self.lib.surfel_destroy(self.handle);self.handle=None
    def __del__(self):self.close()
    def scan_world(self,directions,origins,blind):
        if not self.handle:raise RuntimeError("Renderer is closed")
        started=time.perf_counter()
        d=np.ascontiguousarray(directions,dtype=np.float64)
        o=np.ascontiguousarray(np.broadcast_to(origins,d.shape),dtype=np.float64)
        if d.ndim!=2 or d.shape[1]!=3 or not np.isfinite(d).all() or not np.isfinite(o).all():
            raise ValueError("Expected finite Nx3 rays/origins")
        length=np.linalg.norm(d,axis=1)
        if np.any(length<1e-12) or not np.isfinite(blind) or blind<0:raise ValueError("Nonzero rays and nonnegative blind required")
        d=np.ascontiguousarray(d/length[:,None])
        output=np.empty(len(d),dtype=np.float64);stats=np.zeros(2,dtype=np.uint64)
        prepared=time.perf_counter()
        self.lib.surfel_cast(self.handle,pointer(o),pointer(d),len(d),blind,self.max_range,self.threads,
                             pointer(output),stats.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)))
        ended=time.perf_counter()
        self.last_profile={"input_prepare_s":prepared-started,"native_query_s":ended-prepared,
                           "total_s":ended-started,"aabb_tests":int(stats[0]),"disk_tests":int(stats[1]),
                           "rays":len(d),"threads":self.threads}
        return output
    def scan(self,rays,origin,rotation,blind):
        return self.scan_world(np.asarray(rays)@np.asarray(rotation).T,origin,blind)

def exhaustive_ranges(xyz,normals,radius,directions,origins,blind,max_range,kinds=None):
    """Independent vectorized all-disks oracle for small validation ray sets."""
    xyz=np.asarray(xyz);normals=np.asarray(normals)
    directions=np.asarray(directions);directions=directions/np.linalg.norm(directions,axis=1)[:,None]
    origins=np.broadcast_to(origins,directions.shape);out=[]
    radii=np.broadcast_to(np.asarray(radius,dtype=float),(len(xyz),))
    kinds=np.zeros(len(xyz),dtype=int) if kinds is None else np.asarray(kinds)
    for ray,origin in zip(directions,origins):
        denominator=normals@ray;good=np.abs(denominator)>1e-12
        numerator=np.einsum("ij,ij->i",xyz-origin,normals)
        t=np.full(len(xyz),np.inf);t[good]=numerator[good]/denominator[good]
        good&=(t>=0)&(t<=max_range)
        indices=np.flatnonzero(good)
        distance=np.linalg.norm(origin+t[indices,None]*ray-xyz[indices],axis=1)
        accepted=indices[(distance<=radii[indices]*(1+1e-12))&(kinds[indices]==0)]
        candidates=t[accepted]
        sphere=np.flatnonzero(kinds==1)
        if len(sphere):
            offset=origin-xyz[sphere];projection=offset@ray
            discriminant=projection**2-np.einsum("ij,ij->i",offset,offset)+radii[sphere]**2
            valid=discriminant>=0
            roots=-projection[valid]-np.sqrt(discriminant[valid])
            far=-projection[valid]+np.sqrt(discriminant[valid])
            roots=np.where(roots>=0,roots,far);roots=roots[(roots>=0)&(roots<=max_range)]
            candidates=np.r_[candidates,roots]
        nearest=float(candidates.min()) if len(candidates) else 0.
        out.append(nearest if nearest>=blind else 0.)
    return np.array(out)
