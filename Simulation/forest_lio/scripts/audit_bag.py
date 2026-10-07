#!/usr/bin/env python3
"""Audit all raw Mid-360/IMU messages without a running ROS master."""
import argparse
import csv
from pathlib import Path
import time
import numpy as np
import rosbag
from common import custom_points, imu_values, odom_values, summary, timing, write_json, save_trajectory

def intervals(rows, key, threshold):
    groups = []
    group = []
    for row in rows:
        if row[key] >= threshold:
            group.append(row)
        elif group:
            groups.append(group)
            group = []
    if group:
        groups.append(group)
    return [{"start_s": g[0]["relative_s"], "end_s": g[-1]["relative_s"],
             "frames": len(g), "max_fraction": max(x[key] for x in g)} for g in groups]

def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--lidar-topic", default="/livox/lidar_192_168_2_181")
    ap.add_argument("--imu-topic", default="/livox/imu_192_168_2_181")
    ap.add_argument("--reference-topic", default="/drone_Odometry")
    ap.add_argument("--blind", type=float, default=0.5)
    args = ap.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        ap.error("output directory must be empty (preserve previous results)")
    args.output.mkdir(parents=True, exist_ok=True)
    lidar, imu, reference = [], [], []
    state_changes, last_states = [], {}
    angular = np.zeros((36, 72), dtype=np.int64)
    line_counts, tag_counts, frames = {}, {}, set()
    start_wall = time.monotonic()
    with rosbag.Bag(str(args.bag)) as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, typ in [(args.lidar_topic, "livox_ros_driver2/CustomMsg"),
                           (args.imu_topic, "sensor_msgs/Imu")]:
            if topic not in info or info[topic].msg_type != typ:
                raise ValueError("Missing/wrong type: " + topic)
        start = bag.get_start_time()
        metadata = {"bag": str(args.bag.resolve()), "bytes": args.bag.stat().st_size,
                    "start_unix": start, "duration_s": bag.get_end_time() - start,
                    "topics": {t: {"type": x.msg_type, "messages": x.message_count} for t, x in info.items()}}
        for topic, raw, recorded in bag.read_messages(
                topics=[args.lidar_topic, args.imu_topic, args.reference_topic,
                        "/uav1/mavros/state", "/uav1/mavros/extended_state"], raw=True):
            data = raw[1]
            if topic in ("/uav1/mavros/state", "/uav1/mavros/extended_state"):
                msg = raw[4]().deserialize(data)
                values = {"armed": msg.armed, "mode": msg.mode} if topic.endswith("/state") else {"landed_state": msg.landed_state}
                if last_states.get(topic) != values:
                    state_changes.append({"topic": topic, "relative_s": recorded.to_sec()-start, **values})
                    last_states[topic] = values
            elif topic == args.reference_topic:
                reference.append(odom_values(data))
            elif topic == args.imu_topic:
                seq, stamp, frame, gyro, acc = imu_values(data)
                imu.append(dict(relative_s=stamp-start, stamp=stamp, recorded_stamp=recorded.to_sec(),
                                seq=seq, gx=gyro[0], gy=gyro[1], gz=gyro[2],
                                ax=acc[0], ay=acc[1], az=acc[2],
                                acc_norm=float(np.linalg.norm(acc)), gyro_norm=float(np.linalg.norm(gyro))))
                frames.add(frame)
            else:
                seq, stamp, frame, timebase, declared, _, pts = custom_points(data)
                xyz = pts["xyz"]
                finite = np.isfinite(xyz).all(axis=1)
                radius = np.linalg.norm(xyz, axis=1)
                nonzero = finite & (radius > 0)
                accepted = finite & (radius >= args.blind) & (pts["line"] < 4) & ((pts["tag"] & 0x30) <= 0x10)
                offsets = pts["offset_time"].astype(np.int64)
                # Match the active non-feature Livox branch's valid-count downsampling.
                retained = np.flatnonzero(accepted & (np.arange(len(pts)) > 0))[2::3]
                usable = radius[accepted]
                lidar.append(dict(relative_s=stamp-start, stamp=stamp, recorded_stamp=recorded.to_sec(),
                    seq=seq, declared_points=declared, points=len(pts),
                    declared_mismatch=int(declared != len(pts)), finite_fraction=float(finite.sum()/max(len(pts), 1)),
                    zero_fraction=float(np.sum(finite & (radius == 0))/max(len(pts), 1)),
                    accepted_fraction=float(accepted.sum()/max(len(pts), 1)),
                    offset_min_ns=int(offsets.min()) if len(pts) else 0,
                    offset_max_ns=int(offsets.max()) if len(pts) else 0,
                    offset_last_ns=int(offsets[-1]) if len(pts) else 0,
                    retained_last_minus_max_ms=float((offsets[retained[-1]]-offsets[retained].max())/1e6) if len(retained) else 0,
                    offset_backwards=int(np.sum(np.diff(offsets) < 0)),
                    timebase_minus_header_s=timebase/1e9-stamp,
                    range_p05_m=float(np.percentile(usable, 5)) if len(usable) else 0,
                    range_median_m=float(np.median(usable)) if len(usable) else 0,
                    range_p95_m=float(np.percentile(usable, 95)) if len(usable) else 0))
                if np.any(nonzero):
                    p = xyz[nonzero]; r = radius[nonzero]
                    az = np.arctan2(p[:, 1], p[:, 0]); el = np.arcsin(np.clip(p[:, 2]/r, -1, 1))
                    angular += np.histogram2d(el, az, bins=(36, 72),
                                             range=((-np.pi/2, np.pi/2), (-np.pi, np.pi)))[0].astype(np.int64)
                for field, counts in [("line", line_counts), ("tag", tag_counts)]:
                    v, c = np.unique(pts[field], return_counts=True)
                    for a, b in zip(v, c):
                        counts[int(a)] = counts.get(int(a), 0) + int(b)
                frames.add(frame)
                if len(lidar) % 250 == 0:
                    print("Audited %d LiDAR frames" % len(lidar), flush=True)
    if not lidar or not imu:
        raise ValueError("Empty raw sensor topics")
    liostamps = np.array([x["stamp"] for x in lidar])
    imustamps = np.array([x["stamp"] for x in imu])
    coverage = []
    for x in lidar:
        end = x["stamp"] + x["offset_max_ns"] * 1e-9
        coverage.append(imustamps[0] <= x["stamp"] and imustamps[-1] >= end)
    imu_array = np.array([[x["stamp"], x["gx"], x["gy"], x["gz"], x["ax"], x["ay"], x["az"]] for x in imu])
    static_candidates = []
    # Five-second windows with low gyro and low acceleration variation; provisional only.
    for begin in np.arange(imustamps[0], imustamps[-1]-5, 5):
        a = imu_array[(imustamps >= begin) & (imustamps < begin+5)]
        if len(a) < 800:
            continue
        gyro = np.linalg.norm(a[:, 1:4], axis=1)
        acc_std = a[:, 4:7].std(axis=0)
        if np.percentile(gyro, 95) < 0.05 and np.max(acc_std) < 0.02:
            static_candidates.append({"start_s": float(begin-start), "duration_s": 5,
                "samples": len(a), "gyro_mean_rad_s": a[:, 1:4].mean(axis=0).tolist(),
                "gyro_std_rad_s": a[:, 1:4].std(axis=0).tolist(),
                "acc_mean_native": a[:, 4:7].mean(axis=0).tolist(),
                "acc_std_native": acc_std.tolist()})
    report = {"metadata": metadata, "frames": sorted(frames),
              "lidar_timing": timing(liostamps, [x["seq"] for x in lidar]),
              "imu_timing": timing(imustamps, [x["seq"] for x in imu]),
              "lidar_statistics": {k: summary([x[k] for x in lidar]) for k in [
                  "points", "zero_fraction", "accepted_fraction", "offset_max_ns",
                  "retained_last_minus_max_ms", "timebase_minus_header_s", "offset_backwards"]},
              "record_minus_header_s": {
                  "lidar": summary([x["recorded_stamp"]-x["stamp"] for x in lidar]),
                  "imu": summary([x["recorded_stamp"]-x["stamp"] for x in imu])},
              "point_count_mismatch_frames": sum(x["declared_mismatch"] for x in lidar),
              "lidar_frames_bracketed_by_imu": sum(coverage),
              "line_counts": line_counts, "tag_counts": tag_counts,
              "high_zero_intervals": intervals(lidar, "zero_fraction", 0.8),
              "imu_acc_norm_native": summary([x["acc_norm"] for x in imu]),
              "imu_gyro_norm_rad_s": summary([x["gyro_norm"] for x in imu]),
              "imu_acc_above_3_native": sum(x["acc_norm"] > 3 for x in imu),
              "imu_axes_near_4_native": sum(max(abs(x[k]) for k in ["ax", "ay", "az"]) >= 3.9 for x in imu),
              "state_changes": state_changes,
              "static_candidates": static_candidates,
              "static_candidates_note": "Threshold-selected candidates, not confirmed rest or calibrated noise density/bias.",
              "reference_note": "Recorded LIO output is a repeatability reference, not independent ground truth.",
              "elapsed_wall_s": time.monotonic()-start_wall}
    write_csv(args.output/"lidar_frames.csv", lidar)
    write_csv(args.output/"imu.csv", imu)
    save_trajectory(args.output/"recorded_lio.tum", reference)
    np.savez_compressed(args.output/"observed_angular_histogram.npz", counts=angular,
                        note="Only returned points; not complete emitted beam directions")
    write_json(args.output/"audit.json", report)
    md = ["# Mid-360 原始数据审计", "", "全量扫描原始雷达与 IMU；记录定位仅用于重复性对比。", "",
          "| 项目 | 结果 |", "|---|---|",
          "| Bag 时长 | %.3f s |" % metadata["duration_s"],
          "| 雷达帧 | %d |" % len(lidar),
          "| IMU 消息 | %d |" % len(imu),
          "| 雷达平均频率 | %.3f Hz |" % report["lidar_timing"]["mean_hz"],
          "| IMU 平均频率 | %.3f Hz |" % report["imu_timing"]["mean_hz"],
          "| 雷达序号缺失 | %d |" % report["lidar_timing"]["sequence_missing"],
          "| IMU 序号缺失 | %d |" % report["imu_timing"]["sequence_missing"],
          "| 雷达/IMU 时间不递增次数 | %d / %d |" % (report["lidar_timing"]["nonincreasing"], report["imu_timing"]["nonincreasing"]),
          "| 零坐标比例中位数 | %.2f%% |" % (report["lidar_statistics"]["zero_fraction"]["median"]*100),
          "| 雷达帧有首尾 IMU 覆盖 | %d / %d |" % (sum(coverage), len(lidar)),
          "| 暂定静止窗口 | %d 个（每个 5 s） |" % len(static_candidates), "",
          "## 解释边界", "",
          "- bag 记录时间减帧头时间含整帧采集/传输等待，不等于跨传感器时间偏差。",
          "- IMU 原始加速度模长接近 1 提示 g 量级；需与驱动及 Fast-LIO 初始化归一化一致。",
          "- 角度直方图仅包含实际回波，不能恢复零坐标点的射线方向。",
          "- 静止候选不等于确认静止，短片段无法完成 Allan 方差及偏置随机游走标定。",
          "- 高零坐标片段原因未确定，不能自动解释成森林漏点概率。",
          "- 本脚本的 accepted_fraction 是线号、tag、盲区筛选，不完全复现后续特征提取。", "",
          "## 零坐标比例超过 80% 的连续片段", ""]
    for x in report["high_zero_intervals"]:
        md.append("- %.3f～%.3f s：%d 帧，最高 %.2f%%。" % (x["start_s"], x["end_s"], x["frames"], x["max_fraction"]*100))
    md += ["", "## IMU / flight-state observations", "",
           "- Peak acceleration norm (native): %.6f; peak angular speed: %.6f rad/s." %
           (report["imu_acc_norm_native"]["max"], report["imu_gyro_norm_rad_s"]["max"]),
           "- Samples with an acceleration axis near +/-4 native units: %d. Possible clipping requires device-range verification." % report["imu_axes_near_4_native"],
           "- Large motion and flight-mode changes coincide with poor LiDAR returns; cause is not determined from these statistics."]
    for change in state_changes:
        md.append("- %.3f s: %s" % (change["relative_s"], str(change)))
    (args.output/"report.md").write_text("\n".join(md)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot([x["relative_s"] for x in lidar], [x["zero_fraction"] for x in lidar])
    axs[0].set_ylabel("Zero XYZ fraction")
    axs[1].plot([x["relative_s"] for x in lidar], [x["offset_max_ns"]/1e6 for x in lidar])
    axs[1].set_ylabel("Scan end offset (ms)")
    axs[2].plot([x["relative_s"] for x in imu], [x["acc_norm"] for x in imu], linewidth=0.5)
    axs[2].set_ylabel("IMU accel norm (native)")
    axs[2].set_xlabel("Time since bag start (s)")
    fig.tight_layout(); fig.savefig(args.output/"sensor_audit.png", dpi=150); plt.close(fig)
    print("Report:", args.output/"report.md")

if __name__ == "__main__":
    main()
