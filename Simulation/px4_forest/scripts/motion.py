"""Pose history interpolation for per-point moving lidar, with no visibility cache."""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def lidar_motion(history,times,initial,extrinsic_T,extrinsic_R,rays):
    history=np.asarray(history,dtype=float)
    if len(history)<2 or np.any(np.diff(history[:,0])<=0):raise ValueError("Strictly increasing pose history required")
    positions=np.column_stack([np.interp(times,history[:,0],history[:,k]) for k in range(1,4)])-initial
    rotations=Slerp(history[:,0],Rotation.from_quat(history[:,4:8]))(times)
    return positions+rotations.apply(extrinsic_T),rotations.apply(rays@extrinsic_R.T)
