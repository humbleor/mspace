"""Startup timing, validated geometry reuse, and tracked fleet windows."""

from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np
from forest_geometry import DEFAULTS, extract_geometry, geometry_model

SOURCES = [
    Path(__file__).with_name(name)
    for name in ("forest_geometry.py", "trunk_extraction.py")
]


def algorithm_fingerprint():
    return hashlib.sha256(b"".join(path.read_bytes() for path in SOURCES)).hexdigest()


class StartupStages:
    def __init__(self, output):
        self.output = output
        self.started = time.monotonic()
        self.records = []

    @contextmanager
    def stage(self, name):
        started = time.monotonic()
        print("[startup +%.1fs] %s ..." % (started - self.started, name), flush=True)
        status = "failed"
        try:
            yield
            status = "ok"
        finally:
            elapsed = time.monotonic() - started
            self.records.append(
                {
                    "stage": name,
                    "status": status,
                    "wall_s": elapsed,
                    "elapsed_s": time.monotonic() - self.started,
                }
            )
            (self.output / "startup_timing.json").write_text(
                json.dumps(self.records, indent=2) + "\n"
            )
            print(
                "[startup +%.1fs] %s: %s (%.2fs)"
                % (time.monotonic() - self.started, name, status, elapsed),
                flush=True,
            )


def cached_geometry(points, cache_dir, previous_root=None, explicit=None):
    digest = hashlib.sha256(
        np.ascontiguousarray(points, dtype="<f8").tobytes()
    ).hexdigest()
    algorithm = algorithm_fingerprint()
    identity = {
        "map": digest,
        "algorithm": algorithm,
        "parameters": DEFAULTS,
        "schema": 3,
        "cache_format": 2,
    }
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    cache_path = cache_dir / (key + ".json")

    def geometry_hash(data):
        return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()

    def validate(data, strict=True):
        if (
            not isinstance(data, dict)
            or data.get("local_map_sha256") != digest
            or data.get("frame") != "world"
            or not data.get("colliders")
        ):
            raise ValueError("Forest geometry does not match shared local map")
        if strict and (
            data.get("parameters") != DEFAULTS or data.get("schema_version") != 3
        ):
            raise ValueError("Forest geometry extraction parameters/schema changed")
        geometry_model(data)  # Also validate finite, positive collider geometry.
        return data

    if explicit is not None:
        return validate(json.loads(explicit.read_text()), strict=False), {
            "source": "explicit",
            "path": str(explicit),
            "map_sha256": digest,
        }
    try:
        entry = json.loads(cache_path.read_text())
        if entry["identity"] == identity:
            return validate(entry["geometry"]), {
                "source": "cache",
                "cache_path": str(cache_path),
                "map_sha256": digest,
                "algorithm_sha256": algorithm,
                "geometry_sha256": geometry_hash(entry["geometry"]),
            }
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        pass
    geometry = None
    source = "extracted"
    previous_path = None
    if previous_root is not None:
        candidates = sorted(
            previous_root.glob("*/forest_geometry.json"),
            key=lambda p: p.stat().st_mtime_ns,
            reverse=True,
        )
        for candidate in candidates:
            try:
                provenance = json.loads(
                    candidate.with_name("geometry_cache.json").read_text()
                )
                data = json.loads(candidate.read_text())
                if (
                    not isinstance(provenance, dict)
                    or provenance.get("source")
                    not in ("extracted", "cache", "previous_run")
                    or provenance.get("algorithm_sha256") != algorithm
                    or provenance.get("geometry_sha256") != geometry_hash(data)
                ):
                    continue
                geometry = validate(data)
                source, previous_path = "previous_run", str(candidate)
                break
            except (OSError, ValueError, KeyError, TypeError, IndexError):
                continue
    if geometry is None:
        geometry = validate(extract_geometry(points))
    cache_dir.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=cache_dir, suffix=".tmp", delete=False
        ) as f:
            temporary = Path(f.name)
            json.dump({"identity": identity, "geometry": geometry}, f)
        os.replace(str(temporary), str(cache_path))
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return geometry, {
        "source": source,
        "cache_path": str(cache_path),
        "previous_path": previous_path,
        "map_sha256": digest,
        "algorithm_sha256": algorithm,
        "geometry_sha256": geometry_hash(geometry),
    }


def open_windows(mode, rviz_config, launch):
    windows = []
    if mode in ("both", "rviz"):
        windows.append(launch(["rviz", "-d", rviz_config], "rviz.log", critical=False))
    if mode in ("both", "gazebo"):
        windows.append(launch(["gzclient"], "gazebo_gui.log", critical=False))
    return windows


def contact_message(name, contact):
    return "%s: physical contact at %.3fs: %s <-> %s" % (
        name,
        contact["sim_time_s"],
        contact["collision1"],
        contact["collision2"],
    )
