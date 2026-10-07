"""ROS1 bag decoding and trajectory utilities shared by offline tools."""
import json
import struct
from pathlib import Path
import numpy as np

POINT_DTYPE = np.dtype([
    ("offset_time", "<u4"), ("xyz", "<f4", (3,)),
    ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1"),
])

def header(data):
    seq, sec, nsec, length = struct.unpack_from("<IIII", data)
    end = 16 + length
    return seq, sec + nsec * 1e-9, data[16:end].decode(), end

def custom_points(data):
    seq, stamp, frame, pos = header(data)
    timebase, declared, lidar_id = struct.unpack_from("<QIB", data, pos)
    count = struct.unpack_from("<I", data, pos + 16)[0]
    if len(data) != pos + 20 + count * POINT_DTYPE.itemsize:
        raise ValueError("CustomMsg payload length does not match point array")
    points = np.frombuffer(data, dtype=POINT_DTYPE, count=count, offset=pos + 20)
    return seq, stamp, frame, timebase, declared, lidar_id, points

def imu_values(data):
    seq, stamp, frame, pos = header(data)
    values = np.frombuffer(data, dtype="<f8", count=37, offset=pos)
    return seq, stamp, frame, values[13:16].copy(), values[25:28].copy()

def odom_values(data):
    _, stamp, _, pos = header(data)
    length = struct.unpack_from("<I", data, pos)[0]
    values = np.frombuffer(data, dtype="<f8", count=7, offset=pos + 4 + length)
    return np.r_[stamp, values]

def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"count": 0}
    return dict(count=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()), mean=float(a.mean()))

def timing(stamps, seqs=None):
    t = np.asarray(stamps)
    dt = np.diff(t)
    positive = dt[dt > 0]
    nominal = float(np.median(positive)) if len(positive) else None
    result = {"count": len(t), "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0,
              "interval_s": summary(dt), "nonincreasing": int(np.sum(dt <= 0)),
              "gap_threshold_s": 1.5 * nominal if nominal else None,
              "gaps_over_1_5x_median": int(np.sum(dt > 1.5 * nominal)) if nominal else 0}
    if nominal:
        result["median_hz"] = 1 / nominal
        result["mean_hz"] = (len(t) - 1) / (t[-1] - t[0])
    if seqs is not None:
        diff = np.diff(np.asarray(seqs, dtype=np.int64))
        result["sequence_missing"] = int(np.maximum(diff - 1, 0).sum())
        result["sequence_resets_or_duplicates"] = int(np.sum(diff <= 0))
    return result

def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False, default=lambda x: x.item() if isinstance(x, np.generic) else x.tolist()) + "\n")

def save_trajectory(path, rows):
    a = np.asarray(rows, dtype=float).reshape((-1, 8))
    np.savetxt(path, a, fmt="%.9f", header="timestamp x y z qx qy qz qw")
    return a

def trajectory_comparison(estimate, reference, max_gap=0.2):
    """Timestamp interpolation without fitting away translation/yaw differences."""
    estimate, reference = np.asarray(estimate), np.asarray(reference)
    if len(estimate) < 2 or len(reference) < 2:
        return {"status": "insufficient_samples"}
    reference = reference[np.argsort(reference[:, 0])]
    reference = reference[np.r_[True, np.diff(reference[:, 0]) > 0]]
    idx = np.searchsorted(reference[:, 0], estimate[:, 0])
    inside = (idx > 0) & (idx < len(reference))
    idx = np.clip(idx, 1, len(reference) - 1)
    inside &= (reference[idx, 0] - reference[idx - 1, 0]) <= max_gap
    e = estimate[inside]
    if not len(e):
        return {"status": "no_timestamp_overlap"}
    r = np.column_stack([np.interp(e[:, 0], reference[:, 0], reference[:, k]) for k in range(1, 4)])
    errors = np.linalg.norm(e[:, 1:4] - r, axis=1)
    result = {"status": "ok", "matched_samples": len(e), "matched_fraction": len(e) / len(estimate),
            "position_difference_m": summary(errors), "position_rmse_m": float(np.sqrt(np.mean(errors**2))),
            "alignment": "none; same timestamps and coordinates"}

    if np.all(np.linalg.norm(reference[:, 4:8], axis=1) > 1e-8) and np.all(np.linalg.norm(e[:, 4:8], axis=1) > 1e-8):
        from scipy.spatial.transform import Rotation, Slerp
        rotation = Slerp(reference[:, 0]-reference[0, 0], Rotation.from_quat(reference[:, 4:8]))
        target = rotation(e[:, 0]-reference[0, 0])
        delta = (target.inv()*Rotation.from_quat(e[:, 4:8])).magnitude()*180/np.pi
        result["orientation_difference_deg"] = summary(delta)
        result["orientation_rmse_deg"] = float(np.sqrt(np.mean(delta**2)))
    return result
