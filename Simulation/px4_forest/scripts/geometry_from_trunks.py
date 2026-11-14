"""Build Gazebo collision geometry from online TrunkMapper CSV snapshots.

The online extractor writes one CSV row per tracked trunk in its LIO frame.
This converter reuses the scene transform and map hash expected by run_sitl.py,
so the output is interchangeable with forest_geometry.py for --forest-geometry.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

from forest_geometry import geometry_model

REQUIRED = (
    "id",
    "x",
    "y",
    "z_base",
    "height",
    "radius",
    "axis_x",
    "axis_y",
    "axis_z",
    "coverage",
    "observations",
)


def _number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid online trunk field %s: %r" % (key, row)) from exc


def parse_trunks_text(text):
    """Parse a TrunkWorker CSV snapshot; return (frame_id, stamp, trees)."""
    frame = None
    stamp = None
    header = None
    rows = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            if line.startswith("# frame_id="):
                frame = line.split("=", 1)[1].strip()
            elif line.startswith("# stamp="):
                stamp = float(line.split("=", 1)[1])
            continue
        cells = [cell.strip() for cell in line.split(",")]
        if header is None:
            header = cells
            missing = [key for key in REQUIRED if key not in header]
            if missing:
                raise ValueError("Online trunk CSV missing columns: " + str(missing))
            continue
        if len(cells) != len(header):
            raise ValueError("Online trunk CSV row width mismatch: " + line)
        rows.append(dict(zip(header, cells)))
    if header is None:
        raise ValueError("Online trunk CSV is empty")
    trees = []
    for row in rows:
        values = {key: _number(row, key) for key in REQUIRED if key != "id"}
        values["id"] = int(_number(row, "id"))
        axis = np.array(
            [values["axis_x"], values["axis_y"], values["axis_z"]], dtype=float
        )
        norm = float(np.linalg.norm(axis))
        if not np.isfinite([*values.values()]).all() or norm < 1e-6:
            raise ValueError("Non-finite or zero-axis online trunk row: " + str(row))
        if values["radius"] <= 0 or values["height"] <= 0:
            raise ValueError("Non-positive radius/height in online trunk row: " + str(row))
        if values["observations"] < 1:
            raise ValueError("Online trunk row has no observations: " + str(row))
        axis /= norm
        a = np.array([values["x"], values["y"], values["z_base"]], dtype=float)
        trees.append(
            {
                "id": values["id"],
                "a": a,
                "b": a + values["height"] * axis,
                "radius": values["radius"],
                "axis": axis,
                "coverage": values["coverage"],
                "observations": values["observations"],
            }
        )
    return frame, stamp, trees


def parse_trunks(path):
    return parse_trunks_text(Path(path).read_text())


def transform_trees_to_local(trees, scene, takeoff_height):
    """Map trunk parameters from the source world into the scene local frame."""
    origin = np.asarray(scene["origin_world"], dtype=float)
    origin[2] -= takeoff_height
    rotation = np.asarray(scene["rotation_world_from_local"], dtype=float)
    for tree in trees:
        tree["a"] = (np.asarray(tree["a"], dtype=float) - origin) @ rotation
        tree["b"] = (np.asarray(tree["b"], dtype=float) - origin) @ rotation
        tree["axis"] = np.asarray(tree["axis"], dtype=float) @ rotation
    return trees


def colliders_from_trunks(trees, z_offset=0.0):
    colliders = []
    for tree in trees:
        a = np.asarray(tree["a"], dtype=float).copy()
        b = np.asarray(tree["b"], dtype=float).copy()
        a[2] += z_offset
        b[2] += z_offset
        if not np.isfinite([*a, *b, tree["radius"]]).all():
            raise ValueError("Non-finite collider")
        colliders.append(
            {
                "kind": "trunk",
                "tree_id": int(tree["id"]),
                "a": a.tolist(),
                "b": b.tolist(),
                "radius": float(tree["radius"]),
                "coverage": float(tree["coverage"]),
                "observations": int(tree["observations"]),
                # The online extractor does not report a fit residual; keep a
                # neutral value so existing diagnostics still run.
                "fit_median_error_m": 0.0,
                "ground_extension_m": 0.0,
            }
        )
    return colliders


def build_geometry(trees, local_map, source_trunks, frame_id=None, z_offset=0.0):
    local_map = np.asarray(local_map, dtype=float)
    colliders = colliders_from_trunks(trees, z_offset=z_offset)
    trees_count = len({collider["tree_id"] for collider in colliders})
    return {
        "schema_version": 3,
        "frame": "world",
        "source_points": len(local_map),
        "local_map_sha256": hashlib.sha256(
            np.ascontiguousarray(local_map, dtype="<f8").tobytes()
        ).hexdigest(),
        "parameters": {
            "source": "online_trunks",
            "trunk_file": str(source_trunks),
            "trunk_frame_id": frame_id,
            "z_offset_m": z_offset,
        },
        "colliders": colliders,
        "rejected_trunk_candidates": [],
        "candidate_diagnostics": {},
        "counts": {
            "trunk": len(colliders),
            "stem": 0,
            "branch": 0,
            "trees": trees_count,
            "grounded_trees": 0,
            "observed_only_trees": trees_count,
        },
        "extraction_statistics": {
            "source": "online_trunks",
            "observations": sum(collider["observations"] for collider in colliders),
        },
        "limits": [
            "Online trunk positions are expressed in the extractor's LIO frame; "
            "they must match the scene local frame before use.",
            "Only observed trunk segments are modeled; occluded or missed trunks and branches are unmodeled.",
            "Online coverage and observation counts are evidence, not verified unseen surface.",
            "The online extractor does not report fit residuals; fit_median_error_m is not meaningful.",
        ],
    }


def _local_map_from_scene(scene, takeoff_height):
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[2] / "forest_lio/scripts")
    )
    from renderer import read_pcd
    from scene import to_local

    origin = np.asarray(scene["origin_world"], dtype=float)
    origin[2] -= takeoff_height
    return to_local(
        read_pcd(scene["map"]),
        origin,
        np.asarray(scene["rotation_world_from_local"]),
    )


def validate_trunk_frame(frame_id, transform_from_source, assume_local_frame):
    if frame_id is None:
        raise ValueError("Online trunk CSV is missing the '# frame_id=' header")
    if frame_id == "world" and not transform_from_source and not assume_local_frame:
        raise ValueError(
            "Online trunk CSV frame_id is 'world'; pass --transform-from-source "
            "for a source-bag world or --assume-local-frame if it is already scene-local"
        )


def filter_trunks(trees, min_observations):
    if min_observations < 1:
        raise ValueError("min_observations must be at least 1")
    return [tree for tree in trees if tree["observations"] >= min_observations]


def geometry_from_trunks(
    scene_path,
    trunks_path,
    local_map,
    takeoff_height=1.5,
    z_offset=0.0,
    transform_from_source=False,
    assume_local_frame=False,
    min_observations=2,
):
    scene = json.loads(Path(scene_path).read_text())
    if local_map is None:
        local_map = _local_map_from_scene(scene, takeoff_height)
    frame_id, _, trees = parse_trunks(trunks_path)
    validate_trunk_frame(frame_id, transform_from_source, assume_local_frame)
    trees = filter_trunks(trees, min_observations)
    if not trees:
        raise ValueError(
            "Online trunk file contains no trunks at observations >= %d"
            % min_observations
        )
    if transform_from_source:
        trees = transform_trees_to_local(trees, scene, takeoff_height)
    geometry = build_geometry(
        trees, local_map, trunks_path, frame_id=frame_id, z_offset=z_offset
    )
    geometry["parameters"]["min_observations"] = min_observations
    geometry["scene"] = str(Path(scene_path).resolve())
    return geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--trunks", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--takeoff-height", type=float, default=1.5)
    parser.add_argument(
        "--z-offset",
        type=float,
        default=0.0,
        help="Added to a/b z to align the LIO origin with the scene local frame",
    )
    parser.add_argument(
        "--transform-from-source",
        action="store_true",
        help="Trunk CSV is in the source bag world; map it with the scene origin/rotation",
    )
    parser.add_argument(
        "--assume-local-frame",
        action="store_true",
        help="Treat a frame_id=world CSV as already in the scene local frame",
    )
    parser.add_argument(
        "--min-observations",
        type=int,
        default=2,
        help="Drop tracks seen by fewer batches (default 2)",
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    if args.min_observations < 1:
        parser.error("--min-observations must be at least 1")
    geometry = geometry_from_trunks(
        args.scene,
        args.trunks,
        None,
        args.takeoff_height,
        args.z_offset,
        transform_from_source=args.transform_from_source,
        assume_local_frame=args.assume_local_frame,
        min_observations=args.min_observations,
    )
    args.output.mkdir(parents=True)
    (args.output / "geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    root = ET.Element("sdf", version="1.6")
    root.append(geometry_model(geometry))
    ET.ElementTree(root).write(args.output / "forest_collisions.sdf", encoding="unicode")
    print(json.dumps(geometry["counts"]))
    print(args.output / "geometry.json")


if __name__ == "__main__":
    main()
