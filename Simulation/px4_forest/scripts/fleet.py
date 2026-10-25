"""Validated per-aircraft configuration for a shared forest experiment."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import socket

@dataclass(frozen=True)
class Vehicle:
    uav_id: int
    instance: int
    initial: tuple
    goals: tuple

    @property
    def name(self): return "uav%d" % self.uav_id
    @property
    def namespace(self): return "/" + self.name
    @property
    def sensor_prefix(self): return "/sim/" + self.name
    @property
    def model_name(self): return "forest_iris_%d" % self.uav_id
    @property
    def system_id(self): return self.instance + 1

def load_fleet(source):
    """Goals and initial [x,y,z,yaw] use the shared local forest world."""
    data=json.loads(Path(source).read_text()) if isinstance(source,(str,Path)) else source
    rows=data.get("vehicles") if isinstance(data,dict) else None
    if not isinstance(rows,list) or not 1<=len(rows)<=50:
        raise ValueError("vehicles must contain 1..50 aircraft")
    result=[]
    for row in rows:
        if not isinstance(row,dict): raise ValueError("vehicle must be an object")
        uid,instance=row.get("uav_id"),row.get("instance")
        if type(uid) is not int or type(instance) is not int or not 0<=instance<254:
            raise ValueError("uav_id and instance must be integers; instance must be 0..253")
        initial=row.get("initial")
        goals=row.get("goals")
        if not isinstance(initial,list) or len(initial)!=4 or not all(type(x) in (int,float) and math.isfinite(x) for x in initial):
            raise ValueError("initial must be finite [x,y,z,yaw]")
        if initial[2]!=0: raise ValueError("current world adapter requires initial z=0")
        if not isinstance(goals,list) or not goals:
            raise ValueError("each aircraft needs nonempty world goals")
        if any(not isinstance(g,list) or len(g)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in g) or g[2]<.5 for g in goals):
            raise ValueError("goals must contain finite world XYZ above 0.5 m")
        result.append(Vehicle(uid,instance,tuple(float(x) for x in initial),
                              tuple(tuple(float(x) for x in g) for g in goals)))
    result.sort(key=lambda v:v.uav_id)
    if [v.uav_id for v in result]!=list(range(1,len(result)+1)):
        raise ValueError("current EGO predecessor protocol requires consecutive uav_id starting at 1")
    if len({v.instance for v in result})!=len(result):
        raise ValueError("PX4 instances must be unique")
    for i,a in enumerate(result):
        for b in result[i+1:]:
            if math.hypot(a.initial[0]-b.initial[0],a.initial[1]-b.initial[1])<1.2:
                raise ValueError("initial aircraft separation must be at least 1.2 m")
    return result

def allocate_ports(vehicles):
    """Hold every socket until the complete unique port set is allocated."""
    sockets=[];used=set()
    def take(kind):
        while True:
            s=socket.socket(socket.AF_INET,kind);s.bind(("127.0.0.1",0))
            number=s.getsockname()[1]
            if number not in used:
                sockets.append(s);used.add(number);return number
            s.close()
    try:
        return {"ros":take(socket.SOCK_STREAM),"gazebo":take(socket.SOCK_STREAM),
                "vehicles":{v.name:{"sim":take(socket.SOCK_STREAM),"px4":take(socket.SOCK_DGRAM),
                                   "mavros":take(socket.SOCK_DGRAM),"unused":take(socket.SOCK_DGRAM)} for v in vehicles}}
    finally:
        for s in sockets:s.close()


def gazebo_environment(environment):
    """PX4 v1.13 model Load scales global physics; neutralize per-model scaling."""
    return dict(environment,PX4_SIM_SPEED_FACTOR="1")


def bounded_call(function, timeout=3.):
    """A stalled ROS service must never prevent this run's process cleanup."""
    import queue
    import threading
    result=queue.Queue(maxsize=1)
    def invoke():
        try:result.put((True,function()))
        except Exception as exc:result.put((False,exc))
    threading.Thread(target=invoke,daemon=True).start()
    try:ok,value=result.get(timeout=timeout)
    except queue.Empty:raise TimeoutError("Service/audit exceeded %.2f s"%timeout)
    if not ok:raise value
    return value
