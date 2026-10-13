"""Compare archived physical/estimated/reference paths near clearance violations.

Archived runs have no time-resolved EGO occupancy or original B-spline archive.
Do not claim a definitive planning/perception/control cause from this evidence.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from collision_evidence import ego_acceptance
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"forest_lio/scripts"))
from renderer import read_pcd
from scene import to_local


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error("Use a new output directory")
    a.output.mkdir(parents=True)
    manifest = json.loads((a.run/"manifest.json").read_text())
    result = json.loads((a.run/"result.json").read_text())
    params = json.loads((a.run/"module_params.json").read_text())["/uav_1_ego_planner_node"]
    scene = json.loads(Path(manifest["parameters"]["scene"]).read_text())
    points = to_local(read_pcd(scene["map"]), np.asarray(manifest["map_origin_world"]),
                      np.asarray(manifest["rotation_world_from_local"]))
    tree = cKDTree(points)
    paths = {name: np.loadtxt(a.run/(name+".tum"), ndmin=2) for name in ("truth", "lio", "fcu")}
    paths["reference"] = np.loadtxt(a.run/"commands.csv", delimiter=",", ndmin=2)
    reference_rows = len(paths["reference"])
    # Diagnostic references can repeat header stamps. Keep the last publication
    # at each stamp; never apply this repair to physical acceptance evidence.
    _, reversed_indices = np.unique(paths["reference"][::-1,0], return_index=True)
    paths["reference"] = paths["reference"][reference_rows-1-reversed_indices]
    paths["reference"] = paths["reference"][np.argsort(paths["reference"][:,0],kind="stable")]
    evidence = {}
    for name, rows in paths.items():
        rows = rows[rows[:, 0] <= result["flight_end_sim_s"]]
        paths[name] = rows
        evidence[name] = ego_acceptance(points, rows, params["grid_map"], .05 if name=="truth" else .3, tree)
    physical = evidence["truth"]
    witness = physical["minimum_clearance_location"]
    t = witness["sim_time_s"]
    positions = {name: [float(np.interp(t, rows[:, 0], rows[:, k])) for k in (1, 2, 3)]
                 for name, rows in paths.items()}
    positions["truth"] = witness["body_center"]
    obstacle = np.asarray(witness["obstacle_point"])
    center = np.asarray(positions["truth"])
    neighborhood = points[tree.query_ball_point(obstacle, 1.2)]
    final_map = a.run/"PCD/scans.pcd"
    final_distance = None
    if final_map.exists():
        final_distance = float(cKDTree(read_pcd(final_map)).query(obstacle)[0])
    report = {
        "source_run": str(a.run.resolve()), "ego_parameters": params["grid_map"],
        "reference_duplicate_stamps_removed": reference_rows-len(paths["reference"]),
        "clearance_evidence": evidence, "worst_time_positions": positions,
        "worst_time_center_distances_to_obstacle_m": {n: float(np.linalg.norm(np.asarray(v)-obstacle)) for n,v in positions.items()},
        "worst_time_estimate_to_physics_m": {n: float(np.linalg.norm(np.asarray(v)-center)) for n,v in positions.items()},
        "final_lio_map_nearest_to_obstacle_m": final_distance,
        "limits": ["No historical time-resolved local occupancy, lidar clouds or original B-spline archive.",
                   "commands.csv is traj_server's preview reference, not an unbiased same-time planned body path.",
                   "Final LIO map can establish eventual geometric proximity, not when EGO saw/occupied that obstacle.",
                   "PCD is not semantically labeled: tree trunk/branch/leaf/noise require inspection."]}
    (a.output/"diagnosis.json").write_text(json.dumps(report, indent=2)+"\n")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, (x,y) in zip(axes, [(0,1),(0,2),(1,2)]):
        ax.scatter(neighborhood[:,x], neighborhood[:,y], s=2, color="gray", alpha=.4, label="PCD")
        for name, rows in paths.items():
            selected = rows[np.abs(rows[:,0]-t) <= 2]
            ax.plot(selected[:,x+1], selected[:,y+1], label=name)
            ax.scatter(positions[name][x], positions[name][y], s=35)
        ax.scatter(obstacle[x], obstacle[y], marker="x", c="red", s=70, label="nearest obstacle")
        ax.set(xlabel="xyz"[x]+" (m)", ylabel="xyz"[y]+" (m)",
               xlim=(obstacle[x]-1.2,obstacle[x]+1.2), ylim=(obstacle[y]-1.2,obstacle[y]+1.2))
        ax.set_aspect("equal"); ax.grid(True)
    axes[0].legend(fontsize=8)
    fig.suptitle(f"{a.run.name}: worst physical clearance at {t:.3f} s; nominal EGO radius {params['grid_map']['obstacles_inflation']} m")
    fig.tight_layout(); fig.savefig(a.output/"worst_region.png", dpi=160); plt.close(fig)
    lines = ["# Historical clearance diagnosis", "", f"Run: {a.run.name}",
             f"Physical status: {physical['status']}; minimum nominal clearance: {physical['minimum_clearance_m']:.6f} m.",
             f"Intersection intervals: {len(physical['intersection_intervals'])}; segments: {physical['colliding_segments']}.",
             "", "At worst physical clearance:", "", "| Path | Distance to obstacle (m) | Distance to physics (m) |", "|---|---:|---:|"]
    for n in positions:
        lines.append(f"| {n} | {report['worst_time_center_distances_to_obstacle_m'][n]:.4f} | {report['worst_time_estimate_to_physics_m'][n]:.4f} |")
    lines += ["", f"Final LIO map nearest obstacle distance: {final_distance} m.", "", "![Region](worst_region.png)", "", "Evidence limits:"]
    lines += ["- "+v for v in report["limits"]]
    (a.output/"report.md").write_text("\n".join(lines)+"\n")
    print(a.output/"report.md")


if __name__=="__main__":main()
