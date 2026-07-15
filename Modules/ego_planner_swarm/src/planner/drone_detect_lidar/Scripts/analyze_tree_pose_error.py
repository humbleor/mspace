#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
统计 tree_pose_error 数值 (从日志文件或实时 rostopic)

geometry_msgs::Vector3: x=dx(m), y=dy(m), z=yaw(deg)

用法:
  # 从日志文件解析 (tree_loc_node.cpp 日志格式)
  python analyze_tree_pose_error.py --log <log_file>

  # 实时采集 (需要 roscore 运行)
  python analyze_tree_pose_error.py              # 统计两个UAV
  python analyze_tree_pose_error.py 1            # 仅统计UAV1
  python analyze_tree_pose_error.py 2            # 仅统计UAV2
"""

import math
import re
import sys
import os


def parse_log_line(line):
    """
    Parse: [TreeLocNode] drone_dist=3.2m, matched=3, trans=(-0.054, 0.077, 2.346), yaw=-2.35 deg, rms=0.117m
    Returns (timestamp, dx, dy, yaw, matched) or None
    """
    try:
        ts_match = re.search(r'\[(\d+\.\d+)\]:', line)
        if not ts_match:
            return None
        if 'drone_dist=' not in line or 'trans=(' not in line:
            return None

        ts = float(ts_match.group(1))

        trans_match = re.search(r'trans=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
        if not trans_match:
            return None
        dx = float(trans_match.group(1))
        dy = float(trans_match.group(2))
        dz_yaw = float(trans_match.group(3))

        yaw_match = re.search(r'yaw=([-\d.]+)\s*deg', line)
        if not yaw_match:
            return None
        yaw = float(yaw_match.group(1))

        matched_match = re.search(r'matched=(\d+)', line)
        matched = int(matched_match.group(1)) if matched_match else 0

        return (ts, dx, dy, yaw, matched)
    except (ValueError, IndexError, AttributeError):
        return None


def parse_log_file(filepath):
    """
    Parse log file. UAVs alternate in timestamp-ordered blocks.
    We sort all matching lines by timestamp, then detect alternation
    to assign UAV IDs.
    """
    all_entries = []

    with open(filepath, 'r') as f:
        for line in f:
            result = parse_log_line(line)
            if result:
                all_entries.append(result)

    if not all_entries:
        return {}

    # Sort by timestamp
    all_entries.sort(key=lambda x: x[0])

    # Detect alternation: consecutive lines from same UAV have close timestamps
    # Different UAVs have slightly different clock offsets, creating visible gaps
    # Simple heuristic: assign alternating blocks
    # Look at the gap pattern between consecutive entries
    if len(all_entries) < 2:
        return {'1': [all_entries[0]]}

    # Compute gaps between consecutive entries
    gaps = []
    for i in range(1, len(all_entries)):
        gap = all_entries[i][0] - all_entries[i-1][0]
        gaps.append(gap)

    # Find the median gap - small gaps = same UAV, larger gaps = different UAV
    sorted_gaps = sorted(gaps)
    median_gap = sorted_gaps[len(sorted_gaps) // 2]

    # Classify gaps: if gap > 2*median, it's a UAV switch
    threshold = max(median_gap * 2, 0.1)

    # Assign UAV IDs
    data = {'1': [], '2': []}
    current_uav = '1'
    data['1'].append(all_entries[0])

    for i in range(1, len(all_entries)):
        gap = all_entries[i][0] - all_entries[i-1][0]
        if gap > threshold:
            current_uav = '2' if current_uav == '1' else '1'
        data[current_uav].append(all_entries[i])

    # Remove empty
    data = {k: v for k, v in data.items() if v}
    return data


def print_stats(name, entries):
    """entries: list of (ts, dx, dy, yaw, matched)"""
    if not entries:
        print(f"    No data.")
        return
    n = len(entries)
    for key_idx, lbl in [(1, 'dx (m)'), (2, 'dy (m)'), (3, 'yaw (deg)')]:
        vals = [e[key_idx] for e in entries]
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / n
        std = math.sqrt(var)
        max_abs = max(abs(v) for v in vals)
        max_val = max(vals)
        min_val = min(vals)
        print(f"    {lbl:12s}: mean={mean:+.4f}, std={std:.4f}, max_abs={max_abs:.4f}, min={min_val:+.4f}, max={max_val:+.4f}")
    print(f"    Total samples: {n}")


def print_stats_filtered(entries, min_matched, label):
    filtered = [e for e in entries if e[4] >= min_matched]
    print(f"\n  --- {label} ({len(filtered)} samples) ---")
    print_stats(label, filtered)


def print_header(title):
    print(f"\n{'='*70}")
    print(f" {title}")
    print(f"{'='*70}")


def main():
    if '--log' in sys.argv:
        idx = sys.argv.index('--log')
        if idx + 1 >= len(sys.argv):
            print("Error: --log requires a file path argument")
            return
        log_file = sys.argv[idx + 1]
        if not os.path.exists(log_file):
            print(f"Error: file not found: {log_file}")
            return

        print_header(f"Parsing log file: {log_file}")
        data = parse_log_file(log_file)

        if not data:
            print("No matching data found in log file.")
            return

        # Count
        for uav_id in sorted(data.keys()):
            print(f"  UAV{uav_id}: {len(data[uav_id])} matching events")

        # Overall
        all_data = []
        for entries in data.values():
            all_data.extend(entries)

        print_header("Overall Statistics (all UAVs, all matches)")
        print_stats("Combined", all_data)

        # Per-UAV
        print_header("Per-UAV Statistics (all matches)")
        for uav_id in sorted(data.keys()):
            print(f"\n  --- UAV{uav_id} ({len(data[uav_id])} samples) ---")
            print_stats(f"UAV{uav_id}", data[uav_id])

        # Filtered: matched >= 5
        print_header("Statistics (matched >= 5, higher quality)")
        for uav_id in sorted(data.keys()):
            print_stats_filtered(data[uav_id], 5, f"UAV{uav_id} matched>=5")

        # Filtered: matched >= 7
        print_header("Statistics (matched >= 7, best quality)")
        for uav_id in sorted(data.keys()):
            print_stats_filtered(data[uav_id], 7, f"UAV{uav_id} matched>=7")

        return

    # ROS live mode
    try:
        import rospy
        from geometry_msgs.msg import Vector3
        import signal
    except ImportError:
        print("Error: rospy not available. Use --log mode for offline analysis:")
        print("  python analyze_tree_pose_error.py --log <log_file>")
        return

    mode = sys.argv[1] if len(sys.argv) > 1 else 'all'

    stats = {}
    rospy.init_node('tree_pose_error_stats', anonymous=True)

    if mode in ['1', '2']:
        topic = f'/uav{mode}/tree_pose_error'
        stats[mode] = {'x': [], 'y': [], 'z': []}
        rospy.Subscriber(topic, Vector3, lambda msg, uav=mode: (stats[uav]['x'].append(msg.x), stats[uav]['y'].append(msg.y), stats[uav]['z'].append(msg.z)))
        print(f"Subscribing to {topic}")
    elif mode == 'all':
        for uav in ['1', '2']:
            topic = f'/uav{uav}/tree_pose_error'
            stats[uav] = {'x': [], 'y': [], 'z': []}
            rospy.Subscriber(topic, Vector3, lambda msg, uav=uav: (stats[uav]['x'].append(msg.x), stats[uav]['y'].append(msg.y), stats[uav]['z'].append(msg.z)))
            print(f"Subscribing to {topic}")
    else:
        print(f"Unknown mode: {mode}")
        print("Usage: python analyze_tree_pose_error.py [1|2|all]")
        print("       python analyze_tree_pose_error.py --log <log_file>")
        return

    def shutdown_handler(sig, frame):
        for uav in sorted(stats.keys()):
            d = stats[uav]
            print(f"\n  --- UAV{uav} ---")
            for key, lbl in [('x', 'dx (m)'), ('y', 'dy (m)'), ('z', 'yaw (deg)')]:
                vals = d[key]
                if not vals:
                    print(f"    {lbl}: No data")
                    continue
                n = len(vals)
                mean = sum(vals) / n
                var = sum((v - mean) ** 2 for v in vals) / n
                std = math.sqrt(var)
                max_abs = max(abs(v) for v in vals)
                print(f"    {lbl:12s}: mean={mean:+.4f}, std={std:.4f}, max_abs={max_abs:.4f}")
            print(f"    Total samples: {len(d['x'])}")
        print()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("\nCollecting tree_pose_error data... Press Ctrl+C to show stats.")
    rospy.spin()


if __name__ == '__main__':
    main()
