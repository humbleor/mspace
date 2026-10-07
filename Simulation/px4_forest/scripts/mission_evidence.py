"""Independent ordered waypoint evidence from recorded localization samples."""
import numpy as np

def ordered_arrivals(samples,goals,start_stamp,radius=.5):
    samples=np.asarray(samples);goals=np.asarray(goals)
    arrivals=[]
    for row in samples:
        if row[0]<start_stamp or len(arrivals)==len(goals):continue
        distance=float(np.linalg.norm(row[1:4]-goals[len(arrivals)]))
        if distance<radius:
            arrivals.append({"index":len(arrivals),"stamp":float(row[0]),"distance_m":distance})
    return {"complete":len(arrivals)==len(goals),"arrivals":arrivals,"total":len(goals),"radius_m":radius}
