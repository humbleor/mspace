"""Evidence-limited PCD cylinder extraction and Gazebo collision geometry.

Fits tilted/curved trunks, anchors to observed terrain, and retains attached supported branches.
Cylinder completion is inferred from observed support, not verified unseen surface.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from trunk_extraction import DEFAULTS, extract_trunks, extract_branches


def extract_geometry(points, settings=None):
    cfg = dict(DEFAULTS)
    unknown = set(settings or {}) - set(cfg)
    if unknown:
        raise ValueError("Unknown extraction settings: " + str(sorted(unknown)))
    cfg.update(settings or {})
    for k, v in cfg.items():
        if not np.isfinite(v) or v <= 0:
            raise ValueError("Positive finite extraction settings required: " + k)
    for key in (
        "normal_neighbors",
        "slice_min_points",
        "sparse_slice_min_points",
        "sparse_angular_bins",
        "min_points",
        "ransac_iterations",
        "max_colliders",
        "min_angular_bins",
    ):
        if cfg[key] != int(cfg[key]):
            raise ValueError("Integer setting required: " + key)
    if (
        cfg["normal_neighbors"] < 3
        or not 3 <= cfg["min_angular_bins"] <= 12
        or not 3 <= cfg["sparse_angular_bins"] <= 12
        or cfg["max_tilt_deg"] >= 60
    ):
        raise ValueError("Invalid neighborhood, angular support or tilt limit")
    if (
        cfg["min_radius"] >= cfg["max_radius"]
        or cfg["min_branch_radius"] >= cfg["max_branch_radius"]
    ):
        raise ValueError("Radius bounds must be ordered")
    if any(
        cfg[key] > 1
        for key in (
            "normal_curvature",
            "sparse_normal_curvature",
            "normal_z",
            "normal_agreement",
        )
    ):
        raise ValueError("Normal thresholds must not exceed one")
    xyz = np.asarray(points, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) < 3 or not np.isfinite(xyz).all():
        raise ValueError("Finite Nx3 forest points required (at least three points)")
    if (
        cfg["min_track_occupancy"] > 1
        or cfg["sparse_slice_height"] < cfg["slice_height"]
    ):
        raise ValueError("Invalid height occupancy or sparse slice scale")
    started = time.monotonic()
    trunks, rejected, p, n, curvature, surface, stats = extract_trunks(xyz, cfg)
    branches, branch_stats = extract_branches(p, n, surface, trunks, cfg)
    colliders = trunks + branches
    if len(colliders) > cfg["max_colliders"]:
        raise ValueError("Collider budget exceeded; no silent truncation")
    stats.update(branch_stats)
    return {
        "schema_version": 3,
        "frame": "world",
        "source_points": len(xyz),
        "fit_points": len(p),
        "local_map_sha256": hashlib.sha256(
            np.ascontiguousarray(xyz, dtype="<f8").tobytes()
        ).hexdigest(),
        "parameters": cfg,
        "colliders": colliders,
        "rejected_trunk_candidates": rejected,
        "candidate_diagnostics": stats.pop("candidate_diagnostics"),
        "counts": dict(
            trunk=len(trunks),
            stem=0,
            branch=len(branches),
            trees=stats["accepted_trees"],
            grounded_trees=stats["grounded_trees"],
            observed_only_trees=stats["observed_only_trees"],
        ),
        "extraction_statistics": stats,
        "extraction_wall_s": time.monotonic() - started,
        "limits": [
            "Cylinders infer unseen surfaces; fitting residuals are not semantic ground truth.",
            "Missing terrain never removes measured wood; only supported bases are extended.",
            "Detached, weak, rejected or unseen geometry remains unmodeled.",
            "No foliage flexibility, structural damage or calibrated wood response.",
        ],
    }


def geometry_model(geometry, offset=(0, 0, 0)):
    model = ET.Element("model", name="forest_collision_geometry")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(map(str, offset)) + " 0 0 0"
    for i, c in enumerate(geometry["colliders"]):
        a = np.asarray(c["a"], dtype=float)
        b = np.asarray(c["b"], dtype=float)
        d = b - a
        length = float(np.linalg.norm(d))
        if (
            not np.isfinite([*a, *b, c["radius"]]).all()
            or length <= 0
            or c["radius"] <= 0
        ):
            raise ValueError("Invalid collider")
        axis = d / length
        cross = np.cross([0, 0, 1], axis)
        sine = np.linalg.norm(cross)
        if sine < 1e-12:
            rotation = Rotation.from_rotvec([np.pi, 0, 0] if axis[2] < 0 else [0, 0, 0])
        else:
            rotation = Rotation.from_rotvec(cross / sine * np.arctan2(sine, axis[2]))
        pose = np.r_[(a + b) / 2, rotation.as_euler("xyz")]
        link = ET.SubElement(model, "link", name=f"{c['kind']}_{i:04d}")
        ET.SubElement(link, "pose").text = " ".join(map(str, pose))
        item = ET.SubElement(link, "collision", name="collision")
        cylinder = ET.SubElement(ET.SubElement(item, "geometry"), "cylinder")
        ET.SubElement(cylinder, "radius").text = str(c["radius"])
        ET.SubElement(cylinder, "length").text = str(length)
        surface = ET.SubElement(item, "surface")
        ode = ET.SubElement(ET.SubElement(surface, "contact"), "ode")
        for key, value in {
            "kp": 1e6,
            "kd": 100,
            "max_vel": 1,
            "min_depth": 0.0001,
        }.items():
            ET.SubElement(ode, key).text = str(value)
        # Legacy unverified proxies stay hidden; v2 publishes only screened tree geometry.
        if geometry.get("schema_version", 1) >= 2:
            visual = ET.SubElement(link, "visual", name="visual")
            ET.SubElement(visual, "geometry").append(
                ET.fromstring(ET.tostring(cylinder))
            )
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = "0.28 0.16 0.08 1"
            ET.SubElement(material, "diffuse").text = "0.38 0.23 0.12 1"
    return model


def plot_geometry(points, geometry, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    sampled = points[:: max(1, int(np.ceil(len(points) / 150000)))]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    axes[0].scatter(sampled[:, 0], sampled[:, 1], s=0.3, c="gray", alpha=0.3)
    axes[1].scatter(sampled[:, 0], sampled[:, 2], s=0.3, c="gray", alpha=0.3)
    for c in geometry["colliders"]:
        a = np.asarray(c["a"])
        b = np.asarray(c["b"])
        color = "brown" if c["kind"] in ("trunk", "stem") else "green"
        if c["kind"] in ("trunk", "stem"):
            axes[0].add_patch(Circle(a[:2], c["radius"], fill=False, color=color, lw=1))
        else:
            axes[0].plot([a[0], b[0]], [a[1], b[1]], color=color, lw=2)
        axes[1].plot([a[0], b[0]], [a[2], b[2]], color=color, lw=1)
    axes[0].set(xlabel="local x (m)", ylabel="local y (m)")
    axes[0].set_aspect("equal")
    axes[1].set(xlabel="local x (m)", ylabel="local z (m)")
    for ax in axes:
        ax.grid(True)
    fig.suptitle(
        f"PCD fit: {geometry['counts'].get('trees',0)} trees / {geometry['counts']['trunk']} trunk segments / {geometry['counts']['branch']} attached branches"
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_tree_details(points, geometry, path):
    """Orthogonal PCD/cylinder overlays for the three longest accepted trees."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not geometry["colliders"]:
        return
    p = points
    g = geometry
    trees = {}
    for c in g["colliders"]:
        if c["kind"] == "trunk":
            trees.setdefault(c["tree_id"], []).append(c)
    ids = sorted(
        trees,
        key=lambda i: sum(np.linalg.norm(np.array(c["b"]) - c["a"]) for c in trees[i]),
        reverse=True,
    )[:3]
    fig = plt.figure(figsize=(15, 6))
    for j, i in enumerate(ids):
        cs = trees[i]
        a = np.array(cs[0]["a"])
        b = np.array(cs[-1]["b"])
        center = (a + b) / 2
        mask = (
            (np.linalg.norm(p[:, :2] - center[:2], axis=1) < 1.5)
            & (p[:, 2] > a[2] - 0.4)
            & (p[:, 2] < b[2] + 0.6)
        )
        q = p[mask]
        q = q[:: max(1, int(np.ceil(len(q) / 16000)))]
        ax = fig.add_subplot(1, 3, j + 1)
        horizontal = int(np.argmax(abs(b[:2] - a[:2])))
        ax.scatter(q[:, horizontal], q[:, 2], s=0.6, c="gray", alpha=0.25)
        for c in cs:
            aa = np.array(c["a"])
            bb = np.array(c["b"])
            d = bb - aa
            axis = d / np.linalg.norm(d)
            u = np.cross(axis, [0.0, 1, 0])
            u /= np.linalg.norm(u)
            v = np.cross(axis, u)
            theta = np.linspace(0, 2 * np.pi, 25)
            for t in np.linspace(0, 1, 6):
                ring = (
                    aa
                    + t * d
                    + c["radius"]
                    * (np.cos(theta)[:, None] * u + np.sin(theta)[:, None] * v)
                )
                ax.plot(ring[:, horizontal], ring[:, 2], c="saddlebrown", lw=0.7)
            ax.plot([aa[horizontal], bb[horizontal]], [aa[2], bb[2]], c="red", lw=1)
        ground = cs[0].get("ground")
        if ground is not None:
            gx, gy, gz = ground["coefficients"]
            ref = np.array(ground["point"])
            coords = np.linspace(ref[horizontal] - 0.6, ref[horizontal] + 0.6, 5)
            ax.plot(
                coords,
                gz + ground["coefficients"][horizontal] * (coords - ref[horizontal]),
                color="green",
                lw=1,
            )
        ax.set(
            xlabel="local " + ("x" if horizontal == 0 else "y") + " (m)",
            ylabel="local z (m)",
            title="Tree %d: %d connected segments\nmedian radial error %.3f m"
            % (i, len(cs), np.median([c["fit_median_error_m"] for c in cs])),
        )
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fig.suptitle(
        "Observed PCD (gray), fitted cylinders (brown), axis (red), local terrain fit (green)"
    )
    fig.tight_layout(rect=[0, 0, 1, 0.88])
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--takeoff-height", type=float, default=1.5)
    p.add_argument(
        "--compare-geometry",
        type=Path,
        help="Baseline geometry JSON for same-PCD support comparison",
    )
    a = p.parse_args()
    if a.output.exists():
        p.error("Use a new output directory")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "forest_lio/scripts"))
    from renderer import read_pcd
    from scene import to_local

    scene = json.loads(a.scene.read_text())
    origin = np.asarray(scene["origin_world"], dtype=float)
    origin[2] -= a.takeoff_height
    points = to_local(
        read_pcd(scene["map"]), origin, np.asarray(scene["rotation_world_from_local"])
    )
    geometry = extract_geometry(points)
    geometry.update(scene=str(a.scene.resolve()), map_origin_world=origin.tolist())
    a.output.mkdir(parents=True)
    (a.output / "geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    root = ET.Element("sdf", version="1.6")
    root.append(geometry_model(geometry))
    ET.ElementTree(root).write(a.output / "forest_collisions.sdf", encoding="unicode")
    plot_geometry(points, geometry, a.output / "geometry_overview.png")
    plot_tree_details(points, geometry, a.output / "tree_closeups.png")
    from geometry_diagnostics import write_candidate_diagnostics, compare_geometry

    write_candidate_diagnostics(points, geometry, a.output)
    if a.compare_geometry:
        compare_geometry(
            points, json.loads(a.compare_geometry.read_text()), geometry, a.output
        )
    print(json.dumps(geometry["counts"]))
    print(a.output / "geometry.json")
    if not geometry["counts"]["trunk"]:
        raise SystemExit("No supported trunks found")


if __name__ == "__main__":
    main()
