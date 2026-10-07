#!/usr/bin/env python3
"""Estimate empirical IMU scatter and flight segments; not an Allan calibration."""
import argparse
import json
from pathlib import Path
import numpy as np
from common import write_json, summary

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--audit", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    meta = json.loads((a.audit/"audit.json").read_text())
    imu = np.genfromtxt(a.audit/"imu.csv", delimiter=",", names=True)
    lidar = np.genfromtxt(a.audit/"lidar_frames.csv", delimiter=",", names=True)
    # Conservative windows in this bag: preflight, autonomous training, held-out time segment.
    mask = (imu["relative_s"] >= 2.2) & (imu["relative_s"] < 17)
    gyro = np.column_stack([imu[k][mask] for k in ("gx","gy","gz")])
    acc = np.column_stack([imu[k][mask] for k in ("ax","ay","az")])
    if len(gyro) < 1000 or np.percentile(np.linalg.norm(gyro,axis=1),95) > .05:
        raise ValueError("Stationary preflight window failed gyro gate")
    result = {
        "bag":meta["metadata"]["bag"],
        "manual_switch_note":"Operator confirmed deliberate RC switch to MANUAL; not an autonomous failure.",
        "segments_s":{"stationary":[2.2,17],"autonomous_training":[40,120],
                      "autonomous_temporal_validation":[120,195],"manual_from":201.3003},
        "stationary_samples":len(gyro),
        "gyro_bias_rad_s":gyro.mean(axis=0),
        "gyro_sample_std_rad_s":gyro.std(axis=0,ddof=1),
        "acc_sample_std_g":acc.std(axis=0,ddof=1),
        "acc_mean_g":acc.mean(axis=0),
        "range_noise_m":None,
        "limitations":["Sample scatter contains vibration and correlated noise; not noise density or bias random walk.",
                       "Gravity/attitude confound accelerometer bias; mean acceleration is not a calibrated bias.",
                       "No independent range truth: range noise cannot be identified from this bag alone.",
                       "Temporal validation is same-flight data, not independent forest/generalization validation.",
                       "Missing beam directions cannot be recovered from zero XYZ samples."]}
    for name,(start,end) in list(result["segments_s"].items())[:3]:
        m=(lidar["relative_s"]>=start)&(lidar["relative_s"]<end)
        result[name+"_lidar"]={"frames":int(m.sum()),"zero_fraction":summary(lidar["zero_fraction"][m]),
                               "range_median_m":summary(lidar["range_median_m"][m])}
    a.output.mkdir(parents=True,exist_ok=True)
    write_json(a.output/"sensor_model.json",result)
    print(json.dumps(result,default=lambda x:x.tolist(),indent=2))
if __name__=="__main__":
    main()
