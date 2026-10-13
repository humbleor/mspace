"""Independent Gazebo tree-contact evidence; clear requires a live observer."""
import math


def contact_records(message):
    rows=[]
    for state in message.states:
        for i,p in enumerate(state.contact_positions):
            wrench=state.wrenches[i] if i<len(state.wrenches) else state.total_wrench
            rows.append({"sim_time_s":message.header.stamp.to_sec(),"frame":"gazebo_world",
                         "collision1":state.collision1_name,"collision2":state.collision2_name,
                         "position":[p.x,p.y,p.z],"depth_m":state.depths[i] if i<len(state.depths) else None,
                         "force_n":[wrench.force.x,wrench.force.y,wrench.force.z],
                         "torque_nm":[wrench.torque.x,wrench.torque.y,wrench.torque.z]})
    return rows


def contact_acceptance(rows,monitor_frames,enabled=True,timestamps=None,window=None,max_gap=.05):
    if not enabled:return {"status":"disabled","passed":False,"monitor_frames":monitor_frames}
    if rows:
        return {"status":"contact","passed":False,"monitor_frames":monitor_frames,"contact_points":len(rows),
                "first_contact":min(rows,key=lambda r:r["sim_time_s"]),
                "max_force_n":max(math.sqrt(sum(x*x for x in r["force_n"])) for r in rows)}
    r={"status":"clear" if monitor_frames else "unavailable","passed":bool(monitor_frames),
       "monitor_frames":monitor_frames,"contact_points":0,"first_contact":None,"max_force_n":0}
    if timestamps is not None:
        times=[t for t in timestamps if window and window[0]<=t<=window[1]]
        if not window or len(times)<2 or any(not math.isfinite(t) for t in times) or any(b<=a for a,b in zip(times,times[1:])):
            r.update(status="unavailable",passed=False,reason="Invalid/incomplete contact observer timeline")
        else:
            gap=max(b-a for a,b in zip(times,times[1:]));r["maximum_observer_gap_s"]=gap
            if gap>max_gap+1e-12 or times[0]-window[0]>max_gap or window[1]-times[-1]>max_gap:
                r.update(status="unavailable",passed=False,reason="Contact observer does not cover the physical run")
    return r
