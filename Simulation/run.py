#!/usr/bin/env python3
"""Unified entry point for Mspace simulation tools."""
import argparse
from pathlib import Path
import subprocess
import sys

MODULE = Path(__file__).resolve().parent/"forest_lio"
COMMANDS = {
    "px4": Path(__file__).resolve().parent/"px4_forest/run.py",
    "audit": MODULE/"scripts/audit_bag.py",
    "calibrate": MODULE/"scripts/calibrate_model.py",
    "replay": MODULE/"scripts/run_replay.py",
    "generate": MODULE/"scripts/generate_mid360_bag.py",
    "closed-loop": MODULE/"scripts/run_closed_loop.py",
    "benchmark": MODULE/"scripts/benchmark_renderer.py",
    "test": MODULE/"tests/run_tests.py",
    "surfaces": MODULE/"scripts/validate_surfaces.py",
    "prepare-scene": MODULE/"scripts/prepare_scene.py",
    "coverage": MODULE/"scripts/build_coverage.py",
    "profile": MODULE/"scripts/profile_renderer.py",
    "report-route": MODULE/"scripts/summarize_route.py",
    "report-map": MODULE/"scripts/summarize_map_geometry.py",
    "report-renderer": MODULE/"scripts/summarize_bvh.py",
    "report-phase1": MODULE/"scripts/summarize_results.py",
    "report-phase2": MODULE/"scripts/summarize_closed_loop.py",
}

def main():
    parser=argparse.ArgumentParser(description=__doc__,
        epilog="Use: python3 Simulation/run.py COMMAND --help. Run from the project root; relative data paths refer to the current directory.")
    parser.add_argument("command",choices=sorted(COMMANDS))
    argv=sys.argv[1:]
    if not argv or argv[0] in ("-h","--help"):
        parser.parse_args(argv)
        return
    args=parser.parse_args(argv[:1])
    if args.command=="test" and not any(a in ("-h","--help") for a in argv[1:]):
        codes=[subprocess.call([sys.executable,str(COMMANDS["test"])]+argv[1:]),
               subprocess.call([sys.executable,str(COMMANDS["px4"]),"test"])]
        raise SystemExit(int(any(codes)))
    raise SystemExit(subprocess.call([sys.executable,str(COMMANDS[args.command])]+argv[1:]))

if __name__=="__main__":
    main()
