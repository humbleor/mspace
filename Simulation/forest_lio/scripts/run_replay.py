#!/usr/bin/env python3
"""Run raw-only Fast-LIO2 replay on a private ROS master and save a baseline."""
import argparse
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import threading
import xmlrpc.client
import numpy as np
import rosbag
from common import custom_points, odom_values, save_trajectory, summary, trajectory_comparison, write_json

def stop(proc, timeout=20):
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()

def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if any(fields[k].datatype != 7 for k in ("x", "y", "z")):
        raise ValueError("Expected float32 XYZ")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"]*3,
                      "offsets": [fields[k].offset for k in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.row_step, msg.point_step)).reshape(-1)
    return np.column_stack([a[k] for k in ("x", "y", "z")])

def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype="<f4").reshape((-1, 3))
    h = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
         "COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n") % (len(xyz), len(xyz))
    with open(path, "wb") as f:
        f.write(h.encode()); f.write(xyz.tobytes())

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--lidar-topic", default="/livox/lidar_192_168_2_181")
    ap.add_argument("--imu-topic", default="/livox/imu_192_168_2_181")
    ap.add_argument("--reference-topic", default="/drone_Odometry")
    ap.add_argument("--truth", type=Path)
    ap.add_argument("--config", type=Path)
    ap.add_argument("--rate", type=float, default=0.5)
    ap.add_argument("--duration", type=float, help="Seconds from first raw sensor record")
    ap.add_argument("--map-voxel", type=float, default=0.15)
    ap.add_argument("--no-map", action="store_true")
    ap.add_argument("--capture-clouds",action="store_true",help="Save estimated world clouds and odometry for coverage reconstruction")
    ap.add_argument("--map-window",type=float,nargs=2,help="Map/capture interval relative to source bag start; replay still includes initialization")
    ap.add_argument("--timeout", type=float, help="Wall-clock timeout")
    args = ap.parse_args()
    if args.rate <= 0 or args.map_voxel <= 0 or (args.duration is not None and args.duration <= 0):
        ap.error("rate/map-voxel/duration must be positive")
    if args.output.exists() and any(args.output.iterdir()):
        ap.error("output directory must be empty")
    root = Path(__file__).resolve().parents[3]
    args.output = args.output.resolve()
    for name in ("Log", "PCD", "ros_home"):
        (args.output/name).mkdir(parents=True, exist_ok=True)
    config = (args.config or root/"Modules/fast_lio2/config/mid360.yaml").resolve()
    shutil.copyfile(config, args.output/"input_config.yaml")
    if args.map_window and not 0 <= args.map_window[0] < args.map_window[1]:ap.error("Invalid map-window")
    reference, expected = [], []
    raw_topics = [args.lidar_topic, args.imu_topic]
    with rosbag.Bag(str(args.bag)) as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, typ in zip(raw_topics, ["livox_ros_driver2/CustomMsg", "sensor_msgs/Imu"]):
            if topic not in info or info[topic].msg_type != typ:
                raise ValueError("Missing/wrong type: " + topic)
        first_raw = next(bag.read_messages(topics=raw_topics))[2].to_sec()
        bag_start = bag.get_start_time()
        end = first_raw + args.duration if args.duration else bag.get_end_time()+1
        for topic, raw, recorded in bag.read_messages(
                topics=[args.lidar_topic, args.reference_topic], raw=True,
                start_time=__import__("genpy").Time.from_sec(first_raw),
                end_time=__import__("genpy").Time.from_sec(end)):
            if topic == args.reference_topic:
                reference.append(odom_values(raw[1]))
            else:
                _, stamp, _, _, _, _, pts = custom_points(raw[1])
                expected.append(stamp + (float(pts["offset_time"].max())*1e-9 if len(pts) else 0))
        sensor_duration = min(end, bag.get_end_time())-first_raw
    if not expected:
        raise ValueError("No LiDAR frames in requested interval")
    save_trajectory(args.output/"recorded_lio.tum", reference)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0)); port = sock.getsockname()[1]
    env = os.environ.copy()
    env.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_IP="127.0.0.1",
               ROS_HOME=str(args.output/"ros_home"))
    env.pop("ROS_HOSTNAME", None)
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.update(env)
    procs, handles = [], []
    rows, clouds, voxels = [], [], {}
    callback_errors = []
    started = time.monotonic()
    succeeded = False
    clock_stop = threading.Event()
    clock_thread = None
    player_return = None
    capture_lock=threading.Lock()
    captured=rosbag.Bag(str(args.output/"observations.bag"),"w") if args.capture_clouds else None
    def in_window(stamp):
        return not args.map_window or args.map_window[0] <= stamp-bag_start < args.map_window[1]
    try:
        def launch(cmd, log):
            handle = open(args.output/log, "w")
            handles.append(handle)
            proc = subprocess.Popen(cmd, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            procs.append(proc)
            return proc
        master = launch(["roscore", "-p", str(port)], "roscore.log")
        proxy = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        deadline = time.monotonic()+30
        while True:
            if master.poll() is not None:
                raise RuntimeError("roscore exited; see roscore.log")
            try:
                if proxy.getPid("/forest_lio_runner")[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("ROS master startup timeout")
            time.sleep(0.1)
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        rospy.set_param("/use_sim_time", True)
        rospy.init_node("forest_lio_capture", anonymous=True, disable_signals=True)
        def odom_cb(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            rows.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if captured is not None and in_window(msg.header.stamp.to_sec()):
                with capture_lock:captured.write("/lio/odom",msg,msg.header.stamp)
        def cloud_cb(msg):
            try:
                xyz = cloud_xyz(msg)
                clouds.append([msg.header.stamp.to_sec(), len(xyz)])
                if captured is not None and in_window(msg.header.stamp.to_sec()):
                    with capture_lock:captured.write("/lio/cloud",msg,msg.header.stamp)
                if not args.no_map and in_window(msg.header.stamp.to_sec()):
                    xyz = xyz[np.isfinite(xyz).all(axis=1)]
                    keys = np.floor(xyz/args.map_voxel).astype("<i4")
                    _, idx = np.unique(keys, axis=0, return_index=True)
                    for k, p in zip(keys[idx], xyz[idx]):
                        voxels[k.tobytes()] = p.copy()
            except Exception as exc:
                callback_errors.append(str(exc))
        sub_odom = rospy.Subscriber("/replay/lio/odom", Odometry, odom_cb, queue_size=10000)
        sub_cloud = rospy.Subscriber("/replay/lio/cloud", PointCloud2, cloud_cb, queue_size=1000)
        mapping = launch(["roslaunch", str(root/"Simulation/forest_lio/launch/mapping_mid360_replay.launch"),
                          "lidar_topic:="+args.lidar_topic, "imu_topic:="+args.imu_topic,
                          "output_root:="+str(args.output), "config:="+str(config)], "mapping.log")
        deadline = time.monotonic()+45
        while True:
            if mapping.poll() is not None:
                raise RuntimeError("Mapping launch exited; see mapping.log")
            code, _, state = proxy.getSystemState("/forest_lio_runner")
            subscribed = {t for t, nodes in state[1] if any("mapping" in n for n in nodes)}
            if set(raw_topics) <= subscribed:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("Mapping did not subscribe to raw sensor topics")
            time.sleep(0.1)
        import yaml
        (args.output/"effective_config.yaml").write_text(yaml.safe_dump(
            rospy.get_param("/forest_lio_replay"), allow_unicode=True))
        cmd = ["rosbag", "play", str(args.bag.resolve()), "--clock", "--hz", "100",
               "--rate", str(args.rate), "--delay", "2",
               "--start", str(max(0, first_raw-bag_start)), "--wait-for-subscribers"]
        if args.duration:
            cmd += ["--duration", str(args.duration)]
        cmd += ["--topics"]+raw_topics
        write_json(args.output/"provenance.json", {"bag": str(args.bag.resolve()), "bag_bytes": args.bag.stat().st_size,
                   "raw_topics_only": raw_topics, "private_ros_master": env["ROS_MASTER_URI"],
                   "player_command": cmd, "config": str(config), "requested_duration_s": args.duration,
                   "expected_lidar_frames": len(expected), "map_voxel_m": args.map_voxel,
                   "map_window_relative_to_bag_s":args.map_window,"capture_clouds":args.capture_clouds})
        print("Raw-only replay on", env["ROS_MASTER_URI"], flush=True)
        player = launch(cmd, "player.log")
        deadline = time.monotonic()+(args.timeout or sensor_duration/args.rate+120)
        while player.poll() is None:
            if mapping.poll() is not None:
                raise RuntimeError("Mapping exited during playback")
            if time.monotonic() > deadline:
                raise RuntimeError("Playback timeout")
            time.sleep(0.2)
        player_return = player.returncode
        if player_return != 0:
            raise RuntimeError("rosbag play returned %d" % player_return)
        # Keep /clock moving after EOF so Rate.sleep() cannot strand the last scan.
        from rosgraph_msgs.msg import Clock
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        clock_base = max(expected[-1], rospy.Time.now().to_sec()) + 0.1
        def advance_clock():
            wall_base = time.monotonic()
            while not clock_stop.wait(0.02):
                clock_pub.publish(Clock(clock=rospy.Time.from_sec(clock_base+time.monotonic()-wall_base)))
        clock_thread = threading.Thread(target=advance_clock, daemon=True)
        clock_thread.start()
        # Drain queued scans before signalling the mapping process.
        last_count, stable = -1, time.monotonic()
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            if len(rows) != last_count:
                last_count, stable = len(rows), time.monotonic()
            if time.monotonic()-stable > 3:
                break
            time.sleep(0.2)
        succeeded = bool(rows) and bool(clouds) and not callback_errors and abs(expected[-1]-rows[-1][0]) < 0.02
    finally:
        # The C++ node handles SIGINT itself and may be blocked in Rate.sleep()
        # after rosbag stops /clock. Advance only time so its shutdown can save logs.
        if "rospy" in locals() and len(procs) >= 2 and clock_thread is None:
            from rosgraph_msgs.msg import Clock
            clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
            clock_base = max(expected[-1], rospy.Time.now().to_sec()) + 0.1
            def advance_clock():
                wall_base = time.monotonic()
                while not clock_stop.wait(0.02):
                    clock_pub.publish(Clock(clock=rospy.Time.from_sec(clock_base+time.monotonic()-wall_base)))
            clock_thread = threading.Thread(target=advance_clock, daemon=True)
            clock_thread.start()
        for proc in reversed(procs):
            stop(proc)
        clock_stop.set()
        if clock_thread:
            clock_thread.join(timeout=2)
        if "rospy" in locals():
            rospy.signal_shutdown("Replay complete")
        for handle in handles:
            handle.close()
        if captured is not None:
            with capture_lock:captured.close()
        estimate = save_trajectory(args.output/"replayed_lio.tum", rows)
        if voxels:
            write_pcd(args.output/"forest_map.pcd", list(voxels.values()))
        result = {"status": "ok" if succeeded else "failed", "player_returncode": player_return,
                  "expected_lidar_frames": len(expected), "estimated_odom_messages": len(rows),
                  "registered_cloud_messages": len(clouds), "voxel_map_points": len(voxels),
                  "cloud_points": summary([x[1] for x in clouds]), "callback_errors": callback_errors,
                  "wall_time_s": time.monotonic()-started,
                  "last_expected_stamp": expected[-1], "last_estimate_stamp": rows[-1][0] if rows else None,
                  "last_stamp_difference_s": expected[-1]-rows[-1][0] if rows else None,
                  "recorded_lio_comparison": trajectory_comparison(estimate, reference),
                  "comparison_note": "Recorded LIO is a repeatability reference, not independent truth."}
        if args.truth:
            truth = np.loadtxt(args.truth, ndmin=2)
            result["synthetic_truth_comparison"] = trajectory_comparison(estimate, truth)
            result["synthetic_truth_note"] = "Known simulated IMU pose; no trajectory alignment applied."
        time_log = args.output/"Log/fast_lio_time_log.csv"
        if time_log.exists():
            a = np.genfromtxt(time_log, delimiter=",", skip_header=1, ndmin=2)
            if a.shape[1] >= 2:
                result["processing_time_s"] = summary(a[:, 1])
        write_json(args.output/"replay_result.json", result)
        print("Result:", args.output/"replay_result.json", flush=True)
    if not succeeded:
        raise SystemExit("Replay failed or emitted no odometry")

if __name__ == "__main__":
    main()
