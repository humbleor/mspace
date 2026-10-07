"""generateWps grid order, matching EGO generateGridWaypoints."""
import json
from pathlib import Path
import numpy as np

EPS=1e-4

def generate_wps(config):
    if config.get("waypointDistriFlag",0)!=0:
        raise ValueError("Grid min/max/step requires waypointDistriFlag=0")
    direction=config.get("grid_direction",0)
    if direction not in (0,1):raise ValueError("grid_direction must be 0 or 1")
    limits={}
    for key in ("x","y","z"):
        spec=config[key]
        lo,hi,step=(float(spec[k]) for k in ("min","max","step"))
        if not np.isfinite([lo,hi,step]).all() or hi<lo or step<=0:
            raise ValueError(key+": finite min<=max and step>0 required")
        limits[key]=(lo,hi,step)
    # EGO always starts each layer's sweep at its minimum. Only the
    # progression direction changes between layers; reverse rows start at max.
    sweep,progress=("x","y") if direction==0 else ("y","x")
    def values(lo,hi,step,reverse=False):
        v=hi if reverse else lo
        while v>=lo-EPS if reverse else v<=hi+EPS:
            yield v
            if hi<=lo+EPS:break
            v+=-step if reverse else step
    result=[]
    def emit(p):
        if len(result)>=2000:raise ValueError("Route exceeds 2000 goals")
        result.append([p["x"],p["y"],p["z"]])
    for layer,z in enumerate(values(*limits["z"])):
        lo,hi,step=limits[progress]
        backwards=layer%2==1
        row=hi if backwards else lo
        while row>=lo-EPS if backwards else row<=hi+EPS:
            for v in values(*limits[sweep]):
                p={"z":z,progress:row,sweep:v}
                emit(p)
            if hi<=lo+EPS:break
            next_row=row-step if backwards else row+step
            if next_row>=lo-EPS if backwards else next_row<=hi+EPS:
                for v in values(*limits[sweep],reverse=True):
                    p={"z":z,progress:next_row,sweep:v}
                    emit(p)
            row+=-2*step if backwards else 2*step
            if len(result)>2000:raise ValueError("Route exceeds 2000 goals")
        if len(result)>2000:raise ValueError("Route exceeds 2000 goals")
    return np.asarray(result,dtype=float).reshape(-1,3)

def load_wps(path):
    config=json.loads(Path(path).read_text())
    return config,generate_wps(config)
