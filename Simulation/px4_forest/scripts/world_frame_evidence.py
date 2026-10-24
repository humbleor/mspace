"""Acceptance evidence for the Modules world-frame adapter.

Checks only observable properties of the published data, so it does not trust the
adapter's own log: the world<-FCU transform must be a single fixed yaw+translation,
the controller's world setpoints must map to the adapter's FCU setpoints by the
inverse of that transform, the world cloud must stay on the physical map, and the
initial world pose must sit at the configured initial placement. Pure numpy.
"""
import numpy as np
from collections import Counter
from scipy.spatial import cKDTree

DEFAULT_THRESHOLDS = {
    "angle_rad": 0.01,          # 0.57 deg; a republished odometry pair is exact
    "translation_m": 0.02,
    "command_position_m": 2e-3,
    "command_velocity_mps": 2e-3,
    "command_yaw_rad": 2e-3,
    "anchor_position_m": 0.25,
    "anchor_yaw_rad": 0.175,    # 10 deg
    "cloud_median_m": 0.15,
    "cloud_tolerance_m": 0.3,
    "cloud_fraction": 0.8,
    "min_pairs": 10,
}


def wrap(angle):
    return (np.asarray(angle, dtype=float) + np.pi) % (2 * np.pi) - np.pi


def yaw_from_rows(rows):
    """rows: [stamp,x,y,z,qx,qy,qz,qw] -> (stamp, yaw, position)."""
    a = np.asarray(rows, dtype=float).reshape(-1, 8)
    if not len(a):
        raise ValueError("No poses")
    if not np.isfinite(a).all():
        raise ValueError("Non-finite pose values")
    q = a[:, 4:8]
    norm = np.linalg.norm(q, axis=1)
    if np.any(norm < 1e-6):
        raise ValueError("Zero-length quaternion")
    x, y, z, w = (q / norm[:, None]).T
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return a[:, 0], yaw, a[:, 1:4]


def yaw_matrix(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _pair_by_stamp(pose_stamps, world_stamps):
    """Match only stamps that are unique on both sides; duplicates are re-publications."""
    counts = Counter(round(float(s), 6) for s in pose_stamps)
    world_counts = Counter(round(float(s), 6) for s in world_stamps)
    pose = {round(float(s), 6): i for i, s in enumerate(pose_stamps) if counts[round(float(s), 6)] == 1}
    world = {round(float(s), 6): i for i, s in enumerate(world_stamps) if world_counts[round(float(s), 6)] == 1}
    keys = sorted(set(pose) & set(world))
    return (np.array([pose[k] for k in keys], dtype=int),
            np.array([world[k] for k in keys], dtype=int))


def fixed_transform(pose_rows, world_rows, thresholds=None):
    """Verify p_W = Rz(angle) p_P + t holds for every co-registered pose."""
    limits = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    tp, yaw_p, pos_p = yaw_from_rows(pose_rows)
    tw, yaw_w, pos_w = yaw_from_rows(world_rows)
    i, j = _pair_by_stamp(tp, tw)
    if len(i) < limits["min_pairs"]:
        raise ValueError("Only %d co-registered poses; need %d" % (len(i), limits["min_pairs"]))
    angle = wrap(yaw_w[j] - yaw_p[i])
    mean = float(np.arctan2(np.sin(angle).mean(), np.cos(angle).mean()))
    c, s = np.cos(mean), np.sin(mean)
    translation = np.array([np.median(pos_w[j, 0] - (c * pos_p[i, 0] - s * pos_p[i, 1])),
                            np.median(pos_w[j, 1] - (s * pos_p[i, 0] + c * pos_p[i, 1])),
                            np.median(pos_w[j, 2] - pos_p[i, 2])])
    predicted = pos_p[i].dot(yaw_matrix(mean).T) + translation
    deviation = np.linalg.norm(predicted - pos_w[j], axis=1) if len(i) else np.zeros(1)
    angle_deviation = np.abs(wrap(angle - mean)) if len(angle) else np.zeros(1)
    result = {"pairs": int(len(i)), "angle_rad": mean, "angle_deg": float(np.degrees(mean)),
              "translation": translation.tolist(),
              "max_angle_deviation_rad": float(angle_deviation.max()),
              "max_position_deviation_m": float(deviation.max()),
              "required_angle_rad": limits["angle_rad"],
              "required_translation_m": limits["translation_m"]}
    result["passed"] = bool(result["max_angle_deviation_rad"] <= limits["angle_rad"]
                            and result["max_position_deviation_m"] <= limits["translation_m"])
    return result


def command_round_trip(world_targets, fcu_targets, angle, translation, thresholds=None):
    """Pair controller world setpoints with adapter FCU setpoints by header stamp."""
    limits = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    index = {round(float(t["stamp"]), 6): t for t in fcu_targets}
    inverse = yaw_matrix(angle).T
    pairs = 0
    idle_passthrough = 0
    worst_position = worst_velocity = worst_yaw = 0.0
    for target in world_targets:
        # Pre-arm idle heartbeat (0x4000) is forwarded verbatim by design, not transformed.
        if int(target["mask"]) == 0x4000:
            idle_passthrough += 1
            continue
        other = index.get(round(float(target["stamp"]), 6))
        if other is None:
            continue
        mask = int(target["mask"])
        position_enabled = [k for k in range(3) if not mask & (1 << k)]
        velocity_enabled = [k for k in range(3) if not mask & (1 << (k + 3))]
        if len(position_enabled) == 3:
            expected = inverse.dot(np.asarray(target["position"], float) - translation)
            worst_position = max(worst_position, float(np.abs(expected - np.asarray(other["position"], float)).max()))
        if len(velocity_enabled) == 3:
            expected = inverse.dot(np.asarray(target["velocity"], float))
            worst_velocity = max(worst_velocity, float(np.abs(expected - np.asarray(other["velocity"], float)).max()))
        if not mask & (1 << 11):
            worst_yaw = max(worst_yaw, float(abs(wrap(target["yaw"] - angle - other["yaw"]))))
        pairs += 1
    result = {"pairs": pairs, "idle_passthrough": idle_passthrough, "max_position_error_m": worst_position,
              "max_velocity_error_mps": worst_velocity, "max_yaw_error_rad": worst_yaw,
              "required_position_m": limits["command_position_m"],
              "required_velocity_mps": limits["command_velocity_mps"],
              "required_yaw_rad": limits["command_yaw_rad"]}
    result["passed"] = bool(pairs and worst_position <= limits["command_position_m"]
                            and worst_velocity <= limits["command_velocity_mps"]
                            and worst_yaw <= limits["command_yaw_rad"])
    return result


def cloud_alignment(cloud, map_points, thresholds=None, samples=3000):
    """Distance from cloud points to the physical forest surfels in local coordinates."""
    limits = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    ground = np.asarray(map_points, dtype=float).reshape(-1, 3)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    ground = ground[np.isfinite(ground).all(axis=1)]
    if len(cloud) < 10 or len(ground) < 10:
        raise ValueError("Cloud alignment needs at least 10 finite points on both sides")
    stride = max(1, len(ground) // 100000)
    tree = cKDTree(ground[::stride])
    take = np.linspace(0, len(cloud) - 1, min(samples, len(cloud))).astype(int)
    distance, _ = tree.query(cloud[take], k=1)
    result = {"samples": int(len(take)), "map_points": int(len(ground[::stride])),
              "median_m": float(np.median(distance)), "p90_m": float(np.percentile(distance, 90)),
              "max_m": float(distance.max()),
              "within_tolerance": float(np.mean(distance <= limits["cloud_tolerance_m"])),
              "tolerance_m": limits["cloud_tolerance_m"]}
    result["passed"] = bool(result["median_m"] <= limits["cloud_median_m"]
                            and result["within_tolerance"] >= limits["cloud_fraction"])
    return result


def initial_anchor(world_rows, ready_sim, thresholds=None):
    """The first world pose after readiness must sit at the configured placement."""
    limits = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    stamps, yaw, position = yaw_from_rows(world_rows)
    if ready_sim is None:
        raise ValueError("Adapter never reported readiness")
    after = np.flatnonzero(stamps >= float(ready_sim) - 1e-6)
    if not len(after):
        raise ValueError("No world pose published after readiness")
    first = int(after[0])
    result = {"stamp": float(stamps[first]), "position": position[first].tolist(),
              "yaw_rad": float(wrap(yaw[first])), "position_error_m": float(np.linalg.norm(position[first])),
              "required_position_m": limits["anchor_position_m"],
              "required_yaw_rad": limits["anchor_yaw_rad"]}
    result["passed"] = bool(result["position_error_m"] <= limits["anchor_position_m"]
                            and abs(result["yaw_rad"]) <= limits["anchor_yaw_rad"])
    return result


def assess(world_rows, fcu_rows, world_targets, fcu_targets, world_clouds, lio_clouds,
           local_map, ready_sim=None, thresholds=None):
    """Collect every world-frame check; a failed check fails the SITL run.

    The cloud check is relative: the transformed cloud must be at least as well
    aligned with the physical map as the untransformed LIO cloud of the same
    moment. Absolute distances depend on how the simulated map was sampled.
    """
    limits = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    result = {"world_frame": True, "frame_ready_first_sim_s": ready_sim,
              "samples": {"world_odom": len(world_rows), "fcu_odom": len(fcu_rows),
                          "world_targets": len(world_targets), "fcu_targets": len(fcu_targets),
                          "world_clouds": len(world_clouds), "lio_clouds": len(lio_clouds)},
              "thresholds": limits}
    failures = []
    try:
        transform = fixed_transform(fcu_rows, world_rows, limits)
        result["fcu_transform"] = transform
        if not transform["passed"]:
            failures.append("world<-FCU transform is not a single fixed yaw+translation")
    except ValueError as exc:
        result["fcu_transform"] = {"passed": False, "error": str(exc)}
        failures.append("world<-FCU transform: " + str(exc))
        transform = None
    try:
        result["initial_anchor"] = initial_anchor(world_rows, ready_sim, limits)
        if not result["initial_anchor"]["passed"]:
            failures.append("world origin does not sit at the configured initial placement")
    except ValueError as exc:
        result["initial_anchor"] = {"passed": False, "error": str(exc)}
        failures.append("initial anchor: " + str(exc))
    if transform:
        try:
            trip = command_round_trip(world_targets, fcu_targets, transform["angle_rad"],
                                      np.asarray(transform["translation"]), limits)
            result["command_round_trip"] = trip
            if not trip["passed"]:
                failures.append("world setpoints do not invert to the FCU setpoints")
        except ValueError as exc:
            result["command_round_trip"] = {"passed": False, "error": str(exc)}
            failures.append("command round trip: " + str(exc))
    else:
        result["command_round_trip"] = {"passed": False, "error": "no transform estimate"}
    for name, clouds in (("world", world_clouds), ("lio", lio_clouds)):
        entry = {"passed": False, "error": "no cloud sampled"}
        for cloud in clouds[:1]:
            try:
                entry = cloud_alignment(cloud, local_map, limits)
            except ValueError as exc:
                entry = {"passed": False, "error": str(exc)}
            break
        result["cloud_%s" % name] = entry
    world_cloud = result["cloud_world"]
    lio_cloud = result["cloud_lio"]
    if "median_m" in world_cloud and "median_m" in lio_cloud:
        worse = (world_cloud["median_m"] > lio_cloud["median_m"] + 0.02 or
                 world_cloud["within_tolerance"] < lio_cloud["within_tolerance"] - 0.05)
        result["cloud_relative"] = {"world_minus_lio_median_m": world_cloud["median_m"] - lio_cloud["median_m"],
                                   "world_minus_lio_within_tolerance": world_cloud["within_tolerance"] - lio_cloud["within_tolerance"],
                                   "allowed_median_m": 0.02, "allowed_within_tolerance": 0.05,
                                   "passed": not worse}
        if worse:
            failures.append("world cloud is worse aligned than the untransformed LIO cloud")
    else:
        result["cloud_relative"] = {"passed": False, "error": "both cloud samples required"}
        failures.append("world cloud comparison needs one world and one LIO cloud sample")
    result["failures"] = failures
    result["passed"] = not failures
    result["meaning"] = ("Fixed-transform, command-inverse, anchor and cloud checks are necessary "
                         "conditions for coordinate unification; they are independent of the adapter log "
                         "but do not replace measured multi-aircraft flight accuracy.")
    return result
