#!/usr/bin/env python3
"""
Visualize EGO planner waypoint generation patterns.

Supports 3 distribution modes (matching ego_replan_fsm.cpp):
  0 - Grid (boustrophedon / lawnmower)
  1 - Sinusoidal (alternating Z)
  2 - Spiral

Usage examples:
  # Grid mode — Y-advance + X-return (default)
  python3 Scripts/visualize_waypoints.py --mode 0 \
    --min-x -10 --max-x 10 --min-y 0 --max-y 30 \
    --step-x 2.0 --step-y 2.0 --step-z 0.5

  # Grid mode — X-advance + Y-return
  python3 Scripts/visualize_waypoints.py --mode 0 --grid-direction 1 \
    --min-x -10 --max-x 10 --min-y 0 --max-y 30 \
    --step-x 2.0 --step-y 2.0 --step-z 0.5

  # Sinusoidal mode
  python3 visualize_waypoints.py --mode 1 \
    --min-x -5 --max-x 5 --min-y -5 --max-y 5 --min-z 2 --max-z 4 \
    --step-xy 1.0

  # Spiral mode
  python3 visualize_waypoints.py --mode 2 \
    --center-x 0 --center-y 0 --radius 5 --num-segments 8 \
    --min-z 2 --max-z 6 --step-z 0.5
"""

import argparse
import signal
import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for cleaner Ctrl+C exit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Handle Ctrl+C gracefully — avoid matplotlib hang on close
signal.signal(signal.SIGINT, signal.SIG_DFL)


def generate_grid_waypoints(min_x, max_x, min_y, max_y, min_z, max_z, step_x, step_y, step_z, grid_direction=0):
    """Grid waypoints in boustrophedon (lawnmower) pattern.

    Matches EGOReplanFSM::generateGridWaypoints.
    grid_direction: 0 = Y-advance + X-return, 1 = X-advance + Y-return
    """
    waypoints = []
    eps = 1e-4

    if grid_direction == 0:
        # Y-axis advance + X-axis return
        z = min_z
        while z <= max_z + eps:
            y = min_y
            while y <= max_y + eps:
                x = min_x
                while x <= max_x + eps:
                    waypoints.append([x, y, z])
                    if max_x <= min_x + eps:
                        break
                    x += step_x
                if max_y > min_y + eps:
                    next_y = y + step_y
                    if next_y <= max_y + eps:
                        x = max_x
                        while x >= min_x - eps:
                            waypoints.append([x, next_y, z])
                            if max_x <= min_x + eps:
                                break
                            x -= step_x
                if max_y <= min_y + eps:
                    break
                y += 2 * step_y
            if max_z <= min_z + eps:
                break
            z += step_z
            if z > max_z + eps:
                break
            y = max_y
            while y >= min_y - eps:
                x = min_x
                while x <= max_x + eps:
                    waypoints.append([x, y, z])
                    if max_x <= min_x + eps:
                        break
                    x += step_x
                if max_y > min_y + eps:
                    next_y = y - step_y
                    if next_y >= min_y - eps:
                        x = max_x
                        while x >= min_x - eps:
                            waypoints.append([x, next_y, z])
                            if max_x <= min_x + eps:
                                break
                            x -= step_x
                if max_y <= min_y + eps:
                    break
                y -= 2 * step_y
            z -= step_z
            z += 2 * step_z
    else:
        # X-axis advance + Y-axis return
        z = min_z
        while z <= max_z + eps:
            x = min_x
            while x <= max_x + eps:
                y = min_y
                while y <= max_y + eps:
                    waypoints.append([x, y, z])
                    if max_y <= min_y + eps:
                        break
                    y += step_y
                if max_x > min_x + eps:
                    next_x = x + step_x
                    if next_x <= max_x + eps:
                        y = max_y
                        while y >= min_y - eps:
                            waypoints.append([next_x, y, z])
                            if max_y <= min_y + eps:
                                break
                            y -= step_y
                if max_x <= min_x + eps:
                    break
                x += 2 * step_x
            if max_z <= min_z + eps:
                break
            z += step_z
            if z > max_z + eps:
                break
            x = max_x
            while x >= min_x - eps:
                y = min_y
                while y <= max_y + eps:
                    waypoints.append([x, y, z])
                    if max_y <= min_y + eps:
                        break
                    y += step_y
                if max_x > min_x + eps:
                    next_x = x - step_x
                    if next_x >= min_x - eps:
                        y = max_y
                        while y >= min_y - eps:
                            waypoints.append([next_x, y, z])
                            if max_y <= min_y + eps:
                                break
                            y -= step_y
                if max_x <= min_x + eps:
                    break
                x -= 2 * step_x
            z -= step_z
            z += 2 * step_z
    return np.array(waypoints)


def generate_sin_waypoints(min_x, max_x, min_y, max_y, min_z, max_z, step_xy):
    """Sinusoidal waypoints with alternating Z height.

    Matches EGOReplanFSM::generateSinWaypoints (lines 250-282).
    """
    waypoints = []
    is_top = False
    y = min_y
    while y < max_y:
        x = min_x + step_xy / 2
        while x < max_x:
            z = min_z if not is_top else max_z
            is_top = not is_top
            waypoints.append([x, y, z])
            x += step_xy
        x = max_x - step_xy / 2
        while x > min_x:
            z = max_z if is_top else min_z
            is_top = not is_top
            waypoints.append([x, y + step_xy, z])
            x -= step_xy
        y += 2 * step_xy
    return np.array(waypoints)


def generate_spiral_waypoints(center_x, center_y, radius, num_segments, min_z, max_z, step_z):
    """Spiral waypoints around a circle center.

    Matches EGOReplanFSM::generateSpiralWaypoints (lines 284-319).
    """
    circle_points = []
    delta_angle = 2 * np.pi / num_segments
    for i in range(num_segments):
        circle_points.append([
            center_x + radius * np.cos(delta_angle * i),
            center_y + radius * np.sin(delta_angle * i),
        ])
    circle_points = np.array(circle_points)

    waypoints = []
    circle_idx = 0
    z = min_z
    while z < max_z:
        current_idx = circle_idx % len(circle_points)
        waypoints.append([
            circle_points[current_idx, 0],
            circle_points[current_idx, 1],
            z,
        ])
        circle_idx += 1
        z += step_z
    return np.array(waypoints)


def plot_waypoints(waypoints, mode_name, title_extra=""):
    """Plot waypoints in 3D with connected lines and numbered markers."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    if len(waypoints) == 0:
        print("No waypoints generated.")
        plt.show()
        return

    xs = waypoints[:, 0]
    ys = waypoints[:, 1]
    zs = waypoints[:, 2]

    # Connected trajectory line
    ax.plot(xs, ys, zs, "b-", linewidth=1.5, alpha=0.6, label="path")

    # Waypoint markers
    ax.scatter(xs, ys, zs, c="red", s=50, marker="o", label="waypoints")

    # Start point (green, larger)
    ax.scatter(xs[0], ys[0], zs[0], c="green", s=120, marker="^", label="start")

    # End point (orange, larger)
    ax.scatter(xs[-1], ys[-1], zs[-1], c="orange", s=120, marker="s", label="end")

    # Number labels
    for i in range(len(waypoints)):
        ax.text(xs[i], ys[i], zs[i], f" {i}", fontsize=8, color="gray")

    title = f"Waypoint Generation — {mode_name} ({len(waypoints)} points)"
    if title_extra:
        title += f"\n{title_extra}"
    ax.set_title(title)
    ax.set_xlabel("X (m)", color="red", fontweight="bold")
    ax.set_ylabel("Y (m)", color="green", fontweight="bold")
    ax.set_zlabel("Z (m)", color="blue", fontweight="bold")
    ax.legend()

    # Equal aspect ratio
    max_range = max(xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min()) / 2.0
    mid_x = (xs.max() + xs.min()) / 2.0
    mid_y = (ys.max() + ys.min()) / 2.0
    mid_z = (zs.max() + zs.min()) / 2.0
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # RViz-style axis arrows at scene bottom-front-left corner (X=red, Y=green, Z=blue)
    # Origin at (min_x, min_y, min_z) corner — outside data area
    arrow_len = max_range * 0.3
    ox = mid_x - max_range * 0.9
    oy = mid_y - max_range * 0.9
    oz = mid_z - max_range * 0.9
    ax.quiver(ox, oy, oz, arrow_len, 0, 0, color='red',   arrow_length_ratio=0.2, linewidth=2.5)
    ax.quiver(ox, oy, oz, 0, arrow_len, 0, color='green', arrow_length_ratio=0.2, linewidth=2.5)
    ax.quiver(ox, oy, oz, 0, 0, arrow_len, color='blue',  arrow_length_ratio=0.2, linewidth=2.5)
    ax.text(ox + arrow_len * 1.15, oy, oz, 'X', color='red',   fontweight='bold', fontsize=12, ha='center', va='center')
    ax.text(ox, oy + arrow_len * 1.15, oz, 'Y', color='green', fontweight='bold', fontsize=12, ha='center', va='center')
    ax.text(ox, oy, oz + arrow_len * 1.15, 'Z', color='blue',  fontweight='bold', fontsize=12, ha='center', va='center')

    # RViz default view: elev=30, azim=-60 (looking from front-right-up, Z up)
    ax.view_init(elev=30, azim=-60)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize EGO planner waypoint generation patterns"
    )
    parser.add_argument(
        "--mode", type=int, required=True, choices=[0, 1, 2],
        help="Distribution mode: 0=Grid, 1=Sinusoidal, 2=Spiral"
    )

    # Common params
    parser.add_argument("--min-x", type=float, default=-5.0)
    parser.add_argument("--max-x", type=float, default=5.0)
    parser.add_argument("--min-y", type=float, default=-5.0)
    parser.add_argument("--max-y", type=float, default=5.0)
    parser.add_argument("--min-z", type=float, default=2.0)
    parser.add_argument("--max-z", type=float, default=4.0)

    # Grid mode params
    parser.add_argument("--step-x", type=float, default=1.0, help="Grid step X (mode 0)")
    parser.add_argument("--step-y", type=float, default=1.0, help="Grid step Y (mode 0)")
    parser.add_argument("--step-z", type=float, default=1.0, help="Grid/Spiral step Z (mode 0/2)")
    parser.add_argument("--grid-direction", type=int, default=0, choices=[0, 1],
                        help="Grid direction (mode 0): 0=Y-advance+X-return, 1=X-advance+Y-return")

    # Sin mode params
    parser.add_argument("--step-xy", type=float, default=1.0, help="Sin step XY (mode 1)")

    # Spiral mode params
    parser.add_argument("--center-x", type=float, default=0.0, help="Spiral center X (mode 2)")
    parser.add_argument("--center-y", type=float, default=0.0, help="Spiral center Y (mode 2)")
    parser.add_argument("--radius", type=float, default=5.0, help="Spiral radius (mode 2)")
    parser.add_argument("--num-segments", type=int, default=8, help="Spiral segments per circle (mode 2)")

    args = parser.parse_args()

    mode_names = {0: "Grid (Boustrophedon)", 1: "Sinusoidal", 2: "Spiral"}

    if args.mode == 0:
        waypoints = generate_grid_waypoints(
            args.min_x, args.max_x, args.min_y, args.max_y,
            args.min_z, args.max_z, args.step_x, args.step_y, args.step_z,
            args.grid_direction
        )
        dir_label = "Y-advance + X-return" if args.grid_direction == 0 else "X-advance + Y-return"
        title_extra = (
            f"box=[{args.min_x},{args.max_x}]x[{args.min_y},{args.max_y}]x[{args.min_z},{args.max_z}], "
            f"step=[{args.step_x},{args.step_y},{args.step_z}], dir={dir_label}"
        )
    elif args.mode == 1:
        waypoints = generate_sin_waypoints(
            args.min_x, args.max_x, args.min_y, args.max_y,
            args.min_z, args.max_z, args.step_xy
        )
        title_extra = (
            f"box=[{args.min_x},{args.max_x}]x[{args.min_y},{args.max_y}]x[{args.min_z},{args.max_z}], "
            f"step_xy={args.step_xy}"
        )
    elif args.mode == 2:
        waypoints = generate_spiral_waypoints(
            args.center_x, args.center_y, args.radius,
            args.num_segments, args.min_z, args.max_z, args.step_z
        )
        title_extra = (
            f"center=({args.center_x},{args.center_y}), R={args.radius}, "
            f"segments={args.num_segments}, z=[{args.min_z},{args.max_z}], step_z={args.step_z}"
        )

    print(f"Mode: {mode_names[args.mode]}")
    print(f"Generated {len(waypoints)} waypoints")
    if len(waypoints) > 0:
        print(f"  Start: {waypoints[0]}")
        print(f"  End:   {waypoints[-1]}")

    try:
        plot_waypoints(waypoints, mode_names[args.mode], title_extra)
    except KeyboardInterrupt:
        plt.close('all')
        sys.exit(0)


if __name__ == "__main__":
    main()
