"""Sparse observed-ray/surface evidence; absence or ray evidence never certifies free volume."""
import ctypes
from pathlib import Path
import numpy as np
from native_build import load_shared
INDEX_DTYPE=np.dtype([("x","<i4"),("y","<i4"),("z","<i4")])

def index_records(indices):
    return np.ascontiguousarray(indices,dtype="<i4").view(INDEX_DTYPE).reshape(-1)

class CoverageBuilder:
    def __init__(self,voxel=.3):
        if not np.isfinite(voxel) or voxel<=0:raise ValueError("positive voxel required")
        self.voxel=float(voxel);self.lib,self.library=load_shared("coverage.cpp")
        ptr=ctypes.POINTER(ctypes.c_double)
        self.lib.coverage_create.argtypes=[ctypes.c_double];self.lib.coverage_create.restype=ctypes.c_void_p
        self.lib.coverage_destroy.argtypes=[ctypes.c_void_p]
        self.lib.coverage_update.argtypes=[ctypes.c_void_p,ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_double]
        self.lib.coverage_size.argtypes=[ctypes.c_void_p];self.lib.coverage_size.restype=ctypes.c_uint64
        self.lib.coverage_export.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_uint8)]
        self.handle=self.lib.coverage_create(self.voxel)
    def close(self):
        if getattr(self,"handle",None):self.lib.coverage_destroy(self.handle);self.handle=None
    def __del__(self):self.close()
    def update(self,origin,endpoints,blind=.5,max_range=30):
        if not self.handle:raise RuntimeError("closed coverage builder")
        o=np.ascontiguousarray(origin,dtype=float);p=np.ascontiguousarray(endpoints,dtype=float)
        if o.shape!=(3,) or p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(o).all() or not np.isfinite(p).all():
            raise ValueError("finite origin and Nx3 endpoints required")
        if blind<0 or max_range<=blind:raise ValueError("invalid ranges")
        self.lib.coverage_update(self.handle,o.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            p.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(p),blind,max_range)
    def export(self):
        size=self.lib.coverage_size(self.handle)
        indices=np.empty((size,3),dtype="<i4");flags=np.empty(size,dtype="u1")
        self.lib.coverage_export(self.handle,indices.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                                flags.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
        return CoverageMap(indices,flags,self.voxel)
class CoverageMap:
    def __init__(self,indices,flags,voxel):
        indices=np.ascontiguousarray(indices,dtype="<i4").reshape(-1,3)
        order=np.argsort(index_records(indices))
        self.indices=indices[order];self.flags=np.asarray(flags,dtype="u1")[order];self.voxel=float(voxel)
        self.records=index_records(self.indices)
    def classify(self,points):
        p=np.asarray(points,dtype=float).reshape(-1,3)
        if not np.isfinite(p).all():raise ValueError("finite points required")
        keys=index_records(np.floor(p/self.voxel).astype("<i4"))
        pos=np.searchsorted(self.records,keys);valid=pos<len(self.records)
        out=np.zeros(len(p),dtype="u1")
        if len(self.records):
            safe=np.minimum(pos,len(self.records)-1);valid&=self.records[safe]==keys
            out[valid]=self.flags[safe[valid]]
        return out
    def save(self,path):
        np.savez_compressed(path,indices=self.indices,flags=self.flags,voxel=self.voxel,
                            meaning="1=ray-trace evidence,2=surface endpoint,4=sensor position;0=unknown;no free-space certification")
    @classmethod
    def load(cls,path):
        with np.load(path) as a:return cls(a["indices"],a["flags"],float(a["voxel"]))
