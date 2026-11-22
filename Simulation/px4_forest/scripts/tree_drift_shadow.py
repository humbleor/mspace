"""Evaluate observer-only planar registration of incoming EGO trajectories."""

import copy
import csv
from dataclasses import dataclass
from math import atan2, cos, sin
from threading import Lock
from typing import Optional

import numpy as np
from scipy.interpolate import BSpline


NS = "/tree_drift_shadow"
MIN_TRUNK_OBSERVATIONS = 2
MAX_SOURCE_AGE = 6.0  # ROS seconds, including the Fast-LIO2 trunk accumulation window.
HORIZON = 0.5
SAMPLE_STEP = 0.1


def apply_planar(points, transform):
    x, y, yaw = transform
    c, s = cos(yaw), sin(yaw)
    result = np.array(points, dtype=float, copy=True)
    result[..., :2] = result[..., :2] @ np.array([[c, s], [-s, c]]) + [x, y]
    return result


def inverse_planar(transform):
    x, y, yaw = transform
    return (-cos(yaw) * x - sin(yaw) * y, sin(yaw) * x - cos(yaw) * y, -yaw)


@dataclass(frozen=True)
class Registration:
    stamp: float
    transform: tuple
    pairs: int
    rms: float


@dataclass(frozen=True)
class Prediction:
    observer: int
    peer: int
    received: float
    start: float
    traj_id: int
    knots: tuple
    points: np.ndarray
    registration: Optional[Registration]


def truth_at(rows, stamp, max_gap):
    index = np.searchsorted(rows[:, 0], stamp)
    if index < len(rows) and abs(rows[index, 0] - stamp) < 1e-9:
        return rows[index, 1:4]
    if index == 0 or index == len(rows):
        return None
    left, right = rows[index - 1:index + 1]
    gap = right[0] - left[0]
    if gap > max_gap:
        return None
    return left[1:4] + (right[1:4] - left[1:4]) * ((stamp - left[0]) / gap)


def error_summary(errors):
    if not errors:
        return None
    values = np.asarray(errors)
    return {
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def score_predictions(predictions, truth, drift, separation, max_gap):
    """Score both predictions against nominal-world truth at identical ROS timestamps."""
    traces = {}
    for uid, samples in truth.items():
        samples = np.asarray(samples, dtype=float)[:, :4] if len(samples) else np.empty((0, 4))
        stamps, indices = np.unique(samples[:, 0], return_index=True)
        traces[uid] = np.column_stack((stamps, samples[indices, 1:4]))

    results, rows = {}, []
    for uid, own_truth in traces.items():
        observer_frame = drift.get(uid, (0, 0, 0))
        incoming = sorted((p for p in predictions if p.observer == uid), key=lambda p: p.received)
        observed = []
        for index, p in enumerate(incoming):
            spline = BSpline(p.knots, p.points, 3, extrapolate=False)
            reg = p.registration
            corrected = (BSpline(p.knots, apply_planar(p.points, reg.transform), 3, extrapolate=False)
                         if reg is not None else None)
            lower, upper = p.knots[3], p.knots[len(p.points)]
            stop = p.received + HORIZON
            if index + 1 < len(incoming):
                stop = min(stop, incoming[index + 1].received)
            for step in range(round(HORIZON / SAMPLE_STEP) + 1):
                stamp = max(p.start, p.received) + SAMPLE_STEP * step
                u = min(upper, stamp - p.start + lower)
                if stamp > stop + 1e-9 or stamp - p.start > upper - lower + 1e-9:
                    break
                own = truth_at(own_truth, stamp, max_gap)
                neighbor = truth_at(traces[p.peer], stamp, max_gap)
                if own is None or neighbor is None:
                    continue
                own, neighbor = apply_planar([own, neighbor], observer_frame)
                raw = spline(u)
                fixed = corrected(u) if reg is not None and stamp - reg.stamp <= MAX_SOURCE_AGE else None
                observed.append({
                    "observer": uid, "peer": p.peer, "traj_id": p.traj_id,
                    "received_s": p.received, "start_s": p.start, "sample_s": stamp,
                    "match_source_s": reg.stamp if reg else None,
                    "match_pairs": reg.pairs if reg else None, "match_fit_rms_m": reg.rms if reg else None,
                    "truth_distance_m": float(np.linalg.norm(own - neighbor)),
                    "raw_distance_m": float(np.linalg.norm(own - raw)),
                    "corrected_distance_m": float(np.linalg.norm(own - fixed)) if fixed is not None else None,
                    "raw_error_m": float(np.linalg.norm(raw - neighbor)),
                    "corrected_error_m": float(np.linalg.norm(fixed - neighbor)) if fixed is not None else None,
                })
        paired = [row for row in observed if row["corrected_error_m"] is not None]
        dangerous = [row for row in paired if row["truth_distance_m"] < separation]
        hits = sum(p.registration is not None for p in incoming)
        results[str(uid)] = {
            "received_trajectories": len(incoming),
            "trajectories_with_registration": hits,
            "registration_coverage": hits / len(incoming) if incoming else None,
            "truth_scored_samples": len(observed),
            "paired_samples": len(paired),
            "paired_danger_samples": len(dangerous),
            "paired_raw_error_m": error_summary([r["raw_error_m"] for r in paired]),
            "paired_corrected_error_m": error_summary([r["corrected_error_m"] for r in paired]),
            "paired_truth_min_distance_m": min((r["truth_distance_m"] for r in paired), default=None),
            "paired_raw_min_distance_m": min((r["raw_distance_m"] for r in paired), default=None),
            "paired_corrected_min_distance_m": min((r["corrected_distance_m"] for r in paired), default=None),
            "raw_missed_danger_samples": sum(r["raw_distance_m"] >= separation for r in dangerous),
            "corrected_missed_danger_samples": sum(r["corrected_distance_m"] >= separation for r in dangerous),
        }
        rows.extend(observed)
    scored = sum(r["paired_samples"] > 0 for r in results.values())
    status = "insufficient_paired_samples"
    if scored:
        status = "scored" if scored == len(results) else "partially_scored"
    return {
        "status": status,
        "injected_world_drift_xy_m_yaw_rad": drift,
        "danger_threshold_m": separation,
        "forecast_horizon_s": HORIZON,
        "sample_step_s": SAMPLE_STEP,
        "max_truth_gap_s": max_gap,
        "max_match_source_age_s": MAX_SOURCE_AGE,
        "min_trunk_observations": MIN_TRUNK_OBSERVATIONS,
        "self_reference": "Gazebo executed position at the sample time",
        "observers": results,
    }, rows


def write_samples(path, rows):
    fields = ("observer", "peer", "traj_id", "received_s", "start_s", "sample_s",
              "match_source_s", "match_pairs", "match_fit_rms_m", "truth_distance_m",
              "raw_distance_m", "corrected_distance_m", "raw_error_m", "corrected_error_m")
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class TreeDriftShadow:
    def __init__(self, rospy, ids, use_bridge, drift):
        import tf2_ros
        from fast_lio.msg import TreeCylinderArray
        from drone_detect_lidar.msg import TreeDetection, TreeRelativePose
        from std_msgs.msg import Bool
        from traj_utils.msg import Bspline

        self.ros = rospy
        self.drift = drift
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.tf_error = (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)
        self.lock = Lock()
        self.ready_since = {}
        self.registrations = {}
        self.predictions = []
        self.seen = {}
        self.tree_counts = dict.fromkeys(ids, 0)
        self.trunk_inputs = {uid: {"snapshots": 0, "max_cylinders": 0, "max_confirmed": 0}
                             for uid in ids}
        self.match_counts = dict.fromkeys(ids, 0)
        self.handles = []
        for uid in ids:
            peer = next(i for i in ids if i != uid)
            ns = f"{NS}/uav{uid}"
            trees = rospy.Publisher(ns + "/world_trees", TreeDetection, queue_size=1)
            topic = f"/uav{uid}/bridge/rx/bspline" if use_bridge else "/broadcast_bspline"
            self.handles.extend([
                trees,
                rospy.Subscriber(f"/uav{uid}/tree_trunks_data", TreeCylinderArray,
                                 lambda m, uid=uid: self.observe_trunks(uid, m), queue_size=1),
                rospy.Subscriber(f"/uav{uid}/frame_ready", Bool,
                                 lambda m, uid=uid: self.set_ready(uid, m.data), queue_size=1),
                rospy.Subscriber(ns + "/tree_detection", TreeDetection,
                                 lambda m, uid=uid, pub=trees: self.publish_trees(uid, pub, m), queue_size=1),
                rospy.Subscriber(ns + "/tree_relative_pose_msg", TreeRelativePose,
                                 lambda m, uid=uid, peer=peer: self.register(uid, peer, m), queue_size=10),
                rospy.Subscriber(topic, Bspline,
                                 lambda m, uid=uid, peer=peer: self.capture(uid, peer, m), queue_size=100),
            ])

    def observe_trunks(self, uid, msg):
        with self.lock:
            stats = self.trunk_inputs[uid]
            stats["snapshots"] += 1
            stats["max_cylinders"] = max(stats["max_cylinders"], len(msg.trees))
            confirmed = sum(tree.observations >= MIN_TRUNK_OBSERVATIONS for tree in msg.trees)
            stats["max_confirmed"] = max(stats["max_confirmed"], confirmed)

    def set_ready(self, uid, ready):
        with self.lock:
            if ready:
                self.ready_since.setdefault(uid, self.ros.Time.now().to_sec())
            else:
                self.ready_since.pop(uid, None)
                self.registrations.clear()

    def publish_trees(self, uid, pub, msg):
        frame = f"uav{uid}/lio_init"
        if msg.drone_id != uid or msg.header.frame_id != frame or not msg.trees:
            return
        try:
            tf = self.tf.lookup_transform("world", frame, self.ros.Time(0)).transform
        except self.tf_error:
            return
        q = tf.rotation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        points = apply_planar([(tree.x, tree.y) for tree in msg.trees],
                              (tf.translation.x, tf.translation.y, yaw))
        points = apply_planar(points, self.drift.get(uid, (0, 0, 0)))
        if not np.isfinite(points).all():
            return
        normalized = copy.deepcopy(msg)
        normalized.header.frame_id = "world"
        for tree, (x, y) in zip(normalized.trees, points):
            tree.x, tree.y = x, y
            tree.z_base += tf.translation.z
        with self.lock:
            if not self.ready_since.get(uid) or msg.header.stamp.to_sec() < self.ready_since[uid]:
                return
            self.tree_counts[uid] += 1
        pub.publish(normalized)

    def register(self, uid, peer, msg):
        values = (msg.dx, msg.dy, msg.yaw, msg.rms_residual)
        if ((msg.src_drone_id, msg.dst_drone_id) != (uid, peer) or msg.header.frame_id != "world"
                or not np.isfinite(values).all() or msg.num_matched_trees < 3 or msg.rms_residual < 0):
            return
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            now = self.ros.Time.now().to_sec()
            if (uid not in self.ready_since or peer not in self.ready_since
                    or stamp < max(self.ready_since.values()) or not 0 <= now - stamp <= MAX_SOURCE_AGE):
                return
            previous = self.registrations.get(uid)
            if previous is None or stamp > previous.stamp:
                self.registrations[uid] = Registration(stamp, inverse_planar(values[:3]),
                                                       msg.num_matched_trees, msg.rms_residual)
                self.match_counts[uid] += 1

    def capture(self, uid, peer, msg):
        start = msg.start_time.to_sec()
        received = self.ros.Time.now().to_sec()
        knots = tuple(msg.knots)
        if (msg.drone_id != peer or msg.order != 3 or start <= 0 or received <= 0
                or len(msg.pos_pts) < 4 or len(knots) != len(msg.pos_pts) + 4
                or not np.isfinite(knots).all() or np.any(np.diff(knots) <= 0)
                or not start - 0.1 <= received < start + knots[len(msg.pos_pts)] - knots[3]):
            return
        points = np.asarray([(p.x, p.y, p.z) for p in msg.pos_pts])
        if not np.isfinite(points).all():
            return
        key = (start, msg.traj_id)
        with self.lock:
            if key <= self.seen.get(uid, (0, 0)):
                return
            self.seen[uid] = key
            reg = self.registrations.get(uid)
            if reg is not None and not 0 <= received - reg.stamp <= MAX_SOURCE_AGE:
                reg = None
            points = apply_planar(points, self.drift.get(peer, (0, 0, 0)))
            self.predictions.append(Prediction(uid, peer, received, start, msg.traj_id, knots, points, reg))
