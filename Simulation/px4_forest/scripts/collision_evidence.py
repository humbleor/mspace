"""Independent swept-sphere acceptance against PCD obstacle proxies.

Every PCD point represents a closed ball of obstacle_radius. This conservatively
encloses the renderer's surfel disks; it is not reconstructed solid tree geometry.
The body sphere is orientation invariant. Between truth samples we assume linear
translation, and reject missing/invalid evidence rather than certify free space.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree


def validate_parameters(body_radius, obstacle_radius, max_gap):
    if not all(np.isfinite(v) and v > 0 for v in (body_radius, max_gap)) or not np.isfinite(obstacle_radius) or obstacle_radius < 0:
        raise ValueError("Body radius/gap must be positive; obstacle radius must be nonnegative and finite")


def collision_acceptance(points, trajectory, body_radius=.4, obstacle_radius=.2,
                         max_gap=.05, tree=None):
    validate_parameters(body_radius, obstacle_radius, max_gap)
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Collision map must be finite nonempty Nx3 points")
    report = {
        "status": "unavailable", "passed": False, "frame": "world",
        "model": "body sphere swept against closed PCD point spheres",
        "body_radius_m": body_radius, "obstacle_radius_m": obstacle_radius,
        "max_truth_gap_s": max_gap, "map_points": len(points),
        "local_map_sha256": hashlib.sha256(np.ascontiguousarray(points, dtype="<f8").tobytes()).hexdigest(),
        "checked_segments": 0, "colliding_segments": 0,
        "minimum_clearance_m": None, "first_collision": None,
        "minimum_clearance_location": None, "intersection_intervals": [],
        "limits": ["Conservative obstacle proxies may reject near-surface flight.",
                   "No tree impact response; unobserved PCD regions are not proven free.",
                   "Piecewise linear translation between physical truth samples."],
    }
    rows = np.asarray(trajectory, dtype=float)
    if rows.ndim != 2 or rows.shape[1] < 4 or len(rows) < 2:
        report["reason"] = "At least two physical truth poses are required"
        return report
    if not np.isfinite(rows[:, :4]).all() or np.any(np.diff(rows[:, 0]) <= 0):
        report["reason"] = "Physical truth positions/timestamps are invalid or unordered"
        return report
    gaps = np.diff(rows[:, 0])
    report.update(start_sim_s=float(rows[0, 0]), end_sim_s=float(rows[-1, 0]),
                  largest_truth_gap_s=float(gaps.max()), truth_samples=len(rows))
    tree = cKDTree(points) if tree is None else tree
    radius = body_radius + obstacle_radius
    minimum = np.inf
    # A nearest midpoint point bounds the best segment distance. Any potentially
    # closer point lies within midpoint distance + half the segment length.
    # Include radius as well to retrieve every possible collision entry candidate.
    for start in range(0, len(rows)-1, 2048):
        a = rows[start:start+2048, 1:4]
        b = rows[start+1:start+1+len(a), 1:4]
        a = a[:len(b)]
        mid = (a+b)/2
        half = np.linalg.norm(b-a, axis=1)/2
        nearest, _ = tree.query(mid)
        neighborhoods = tree.query_ball_point(mid, np.maximum(nearest, radius)+half+1e-10)
        for offset, ids in enumerate(neighborhoods):
            i = start+offset
            origin = a[offset]
            delta = b[offset]-origin
            length2 = float(delta @ delta)
            obstacles = points[np.asarray(ids, dtype=int)]
            relative = obstacles-origin
            fraction = np.clip(relative @ delta / length2, 0, 1) if length2 else np.zeros(len(ids))
            distances2 = np.sum((relative-fraction[:, None]*delta)**2, axis=1)
            closest = int(np.argmin(distances2))
            clearance = float(np.sqrt(distances2[closest])-radius)
            if clearance < minimum:
                minimum = clearance
                alpha = float(fraction[closest])
                report["minimum_clearance_location"] = {
                    "segment_index": i, "sim_time_s": float(rows[i, 0]+alpha*gaps[i]),
                    "body_center": (origin+alpha*delta).tolist(),
                    "obstacle_point": obstacles[closest].tolist(), "map_point_index": int(ids[closest])}
            hit = distances2 <= radius**2+1e-12
            report["checked_segments"] += 1
            if not hit.any():
                continue
            report["colliding_segments"] += 1
            intervals = report["intersection_intervals"]
            if intervals and intervals[-1]["last_segment"] == i-1:
                intervals[-1].update(last_segment=i, end_sim_s=float(rows[i+1, 0]),
                                     minimum_clearance_m=min(intervals[-1]["minimum_clearance_m"], clearance))
            else:
                intervals.append({"first_segment": i, "last_segment": i,
                                  "start_sim_s": float(rows[i, 0]), "end_sim_s": float(rows[i+1, 0]),
                                  "minimum_clearance_m": clearance})
            if report["first_collision"] is not None:
                continue
            rel = relative[hit]
            inside = np.sum(rel**2, axis=1) <= radius**2+1e-12
            entry = np.zeros(len(rel))
            if length2:
                projection = rel @ delta / length2
                perpendicular2 = np.sum(rel**2, axis=1)-projection**2*length2
                entry = np.clip(projection-np.sqrt(np.maximum(0, radius**2-perpendicular2)/length2), 0, 1)
                entry[inside] = 0
            j = int(np.argmin(entry))
            alpha = float(entry[j])
            point_index = int(np.asarray(ids)[hit][j])
            report["first_collision"] = {
                "segment_index": i, "sim_time_s": float(rows[i, 0]+alpha*gaps[i]),
                "body_center": (origin+alpha*delta).tolist(),
                "obstacle_point": points[point_index].tolist(), "map_point_index": point_index,
            }
    report["minimum_clearance_m"] = minimum
    if report["first_collision"] is not None:
        report["status"] = "collision"
    elif gaps.max() > max_gap+1e-12:
        report["reason"] = "Physical truth sampling gap exceeds acceptance limit"
    else:
        report.update(status="clear", passed=True)
    return report


def ego_acceptance(points, trajectory, grid_params, max_gap=.05, tree=None):
    """Nominal radial clearance from EGO's recorded obstacle inflation parameter.

    Not an exact reproduction of EGO's anisotropic voxel cloudCallback, nor a
    physical-body test. No surfel support or additional body radius is added.
    """
    radius = float(grid_params["obstacles_inflation"])
    resolution = float(grid_params["resolution"])
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("Invalid recorded EGO map resolution")
    r = collision_acceptance(points, trajectory, radius, 0, max_gap, tree)
    r.update(model="EGO nominal radial clearance to PCD points", acceptance_kind="planner_clearance",
             parameter_source="/uav_1_ego_planner_node/grid_map/obstacles_inflation",
             ego_obstacles_inflation_m=radius, ego_resolution_m=resolution,
             ego_cloud_inflation_steps_xy=int(np.ceil(radius/resolution)), ego_cloud_inflation_steps_z=1)
    r["limits"] = ["Planner clearance acceptance, not actual body/tree contact.",
                   "EGO cloudCallback dilates voxel indices in XY and one voxel in Z; this radial check is not identical.",
                   "Full static PCD is not the planner's time-dependent observed map; unknown regions are not certified free.",
                   "Piecewise linear translation between recorded samples."]
    return r


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path, help="Existing forest run directory (read only)")
    p.add_argument("--output", type=Path, required=True, help="New evidence JSON; never overwrite")
    p.add_argument("--body-radius", type=float, default=.4)
    p.add_argument("--obstacle-radius", type=float, default=.2)
    p.add_argument("--model", choices=["ego", "proxy"], default="ego")
    p.add_argument("--max-truth-gap", type=float, default=.05)
    a = p.parse_args()
    if a.model == "proxy" and a.obstacle_radius < .2:
        p.error("Obstacle proxy must enclose the current 0.2 m sensor surfel support")
    if a.output.exists():
        p.error("Evidence output already exists")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"forest_lio/scripts"))
    from renderer import read_pcd
    from scene import to_local
    manifest = json.loads((a.run/"manifest.json").read_text())
    if manifest["parameters"]["stage"] != "forest":
        p.error("Collision evidence requires a forest run")
    scene = json.loads(Path(manifest["parameters"]["scene"]).read_text())
    points = to_local(read_pcd(scene["map"]), np.asarray(manifest["map_origin_world"]),
                      np.asarray(manifest["rotation_world_from_local"]))
    rows = np.loadtxt(a.run/"truth.tum", ndmin=2)
    result = json.loads((a.run/"result.json").read_text())
    if result.get("flight_end_sim_s") is not None and rows.size:
        rows = rows[rows[:, 0] <= result["flight_end_sim_s"]]
    if a.model == "ego":
        params = json.loads((a.run/"module_params.json").read_text())
        evidence = ego_acceptance(points, rows, params["/uav_1_ego_planner_node"]["grid_map"], a.max_truth_gap)
    else:
        evidence = collision_acceptance(points, rows, a.body_radius, a.obstacle_radius, a.max_truth_gap)
    evidence["source_run"] = str(a.run.resolve())
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("x") as f:
        json.dump(evidence, f, indent=2)
        f.write("\n")
    print(a.output)
    if not evidence["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
