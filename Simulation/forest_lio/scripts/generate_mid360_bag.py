#!/usr/bin/env python3
"""Generate a timed point-map/IMU LIO benchmark; this is not a full optical model."""
import argparse
import struct
from pathlib import Path
import time
import numpy as np
import rosbag
import genpy
import yaml
from renderer import PointRenderer, read_pcd
from bvh_renderer import BVHRenderer
from scipy.spatial.transform import Rotation
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from common import POINT_DTYPE, custom_points, save_trajectory, write_json

def trajectory(t, duration, static, amplitude):
    """C2-continuous analytic IMU pose; initial orientation and position are identity."""
    if t <= static:
        return np.zeros(3), np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3)
    scale = duration-static
    u = np.clip((t-static)/scale, 0, 1)
    s = 10*u**3-15*u**4+6*u**5
    sd = (30*u**2-60*u**3+30*u**4)/scale
    sdd = (60*u-180*u**2+120*u**3)/scale**2
    a, ad, add = 2*np.pi*s, 2*np.pi*sd, 2*np.pi*sdd
    p = amplitude*np.array([np.sin(a), 0.5*(1-np.cos(a)), 0.15*(1-np.cos(a))])
    pa = amplitude*np.array([np.cos(a), 0.5*np.sin(a), 0.15*np.sin(a)])
    paa = amplitude*np.array([-np.sin(a), 0.5*np.cos(a), 0.15*np.cos(a)])
    vel = pa*ad; acc = paa*ad**2+pa*add
    yaw = 0.4*np.sin(a)
    rot = Rotation.from_euler("z", yaw).as_matrix()
    omega = np.array([0, 0, 0.4*np.cos(a)*ad])
    return p, vel, acc, rot, omega

def trajectory_pose_samples(times,duration,static,amplitude):
    """Vectorized analytic IMU positions and orientations at each ray time."""
    times=np.asarray(times,dtype=float)
    u=np.clip((times-static)/(duration-static),0,1)
    phase=2*np.pi*(10*u**3-15*u**4+6*u**5)
    positions=amplitude*np.column_stack([np.sin(phase),.5*(1-np.cos(phase)),.15*(1-np.cos(phase))])
    rotations=Rotation.from_euler("z",.4*np.sin(phase)).as_matrix()
    return positions,rotations

def templates_from_bag(path, topic, count, period, window=None):
    templates = []
    with rosbag.Bag(str(path)) as bag:
        total = bag.get_message_count(topic_filters=[topic])
        selected = set(np.linspace(0, max(0, total-1), count).astype(int))
        for i, (_, raw, recorded) in enumerate(bag.read_messages(topics=[topic], raw=True)):
            if window is not None and recorded.to_sec()-bag.get_start_time()>=window[1]:
                break
            if window is not None and not window[0] <= recorded.to_sec()-bag.get_start_time() < window[1]:
                continue
            if i not in selected:
                continue
            _, _, _, _, _, _, pts = custom_points(raw[1])
            radius = np.linalg.norm(pts["xyz"], axis=1)
            good = np.isfinite(radius) & (radius >= 0.5) & (pts["line"] < 4)
            good &= ((pts["tag"] & 0x30) <= 0x10) & (pts["offset_time"] < period*1e9)
            if good.mean() < 0.6:
                continue
            p = pts[good].copy()
            p = p[np.argsort(p["offset_time"], kind="stable")]
            rays = p["xyz"].astype(float)
            rays /= np.linalg.norm(rays, axis=1)[:, None]
            templates.append((rays, p["offset_time"].copy(), p["line"].copy(), p["reflectivity"].copy()))
    if not templates:
        raise ValueError("No valid returned-direction templates")
    return templates

def pack_scan(seq, stamp, points):
    # Serialize in one operation, avoiding thousands of Python CustomPoint objects.
    sec = int(stamp); ns = int(round((stamp-sec)*1e9))
    if ns == 1000000000:
        sec += 1; ns = 0
    frame = b"livox_frame"
    data = struct.pack("<IIII", seq, sec, ns, len(frame))+frame
    data += struct.pack("<QIB3sI", sec*1000000000+ns, len(points), 0, b"\0"*3, len(points))
    return CustomMsg().deserialize(data+points.tobytes())

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--template-bag", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--template-topic", default="/livox/lidar_192_168_2_181")
    ap.add_argument("--config", type=Path)
    ap.add_argument("--duration", type=float, default=30)
    ap.add_argument("--static-seconds", type=float, default=3)
    ap.add_argument("--amplitude", type=float, default=0.5)
    ap.add_argument("--frame-rate", type=float, default=10)
    ap.add_argument("--imu-rate", type=float, default=200)
    ap.add_argument("--subscan-ms", type=float, default=10)
    ap.add_argument("--angular-resolution-deg", type=float, default=0.5)
    ap.add_argument("--surface-radius", type=float, default=0.2)
    ap.add_argument("--max-range", type=float, default=30)
    ap.add_argument("--range-noise-m", type=float, default=0)
    ap.add_argument("--gyro-noise", type=float, default=0)
    ap.add_argument("--acc-noise-g", type=float, default=0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--renderer",choices=["bvh","raster"],default="bvh")
    ap.add_argument("--render-threads",type=int,default=4)
    ap.add_argument("--surface-model",choices=["fixed","adaptive"],default="fixed")
    args = ap.parse_args()
    if args.surface_model=="adaptive" and args.renderer!="bvh":ap.error("Adaptive surfaces require BVH")
    if args.render_threads<1:ap.error("render-threads must be positive")
    if args.duration <= args.static_seconds or args.static_seconds < 0:
        ap.error("duration must exceed nonnegative static-seconds")
    for name in ["frame_rate", "imu_rate", "subscan_ms", "angular_resolution_deg", "surface_radius", "max_range"]:
        if getattr(args, name) <= 0:
            ap.error(name+" must be positive")
    if any(getattr(args, k) < 0 for k in ["amplitude", "range_noise_m", "gyro_noise", "acc_noise_g"]):
        ap.error("amplitude/noise values cannot be negative")
    if args.output.exists() and any(args.output.iterdir()):
        ap.error("output directory must be empty")
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    config_path = args.config or root/"Modules/fast_lio2/config/mid360.yaml"
    config = yaml.safe_load(config_path.read_text())
    translation = np.array(config["mapping"]["extrinsic_T"])
    extrinsic = np.array(config["mapping"]["extrinsic_R"]).reshape(3, 3)
    xyz = read_pcd(args.map)
    if len(xyz) < 16:
        raise ValueError("Need at least 16 map points")
    renderer = (BVHRenderer(xyz,args.surface_radius,args.max_range,threads=args.render_threads,surface_model=args.surface_model)
                if args.renderer=="bvh" else PointRenderer(xyz, args.angular_resolution_deg, args.surface_radius, args.max_range))
    period = 1/args.frame_rate
    templates = templates_from_bag(args.template_bag, args.template_topic, 24, period)
    rng = np.random.default_rng(args.seed)
    epoch = 1700000000.0
    truth = []
    ray_counts, hit_counts = [], []
    started = time.monotonic()
    lidar_topic = "/sim/mid360/lidar"; imu_topic = "/sim/mid360/imu"
    from nav_msgs.msg import Odometry
    with rosbag.Bag(str(args.output/"sensors.bag"), "w") as bag:
        # Write IMU through final scan end, including an extra sample for bracketing.
        for i in range(int(np.ceil(args.duration*args.imu_rate))+2):
            t = i/args.imu_rate
            p, v, acc, rot, omega = trajectory(t, args.duration, args.static_seconds, args.amplitude)
            specific = rot.T@(acc-np.array([0, 0, -9.81]))/9.81
            imu = Imu(header=Header(seq=i, stamp=genpy.Time.from_sec(epoch+t), frame_id="livox_frame"))
            gyro = omega+rng.normal(0, args.gyro_noise, 3)
            a = specific+rng.normal(0, args.acc_noise_g, 3)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = a
            imu.orientation_covariance[0] = -1
            bag.write(imu_topic, imu, imu.header.stamp)
            q = Rotation.from_matrix(rot).as_quat()
            truth.append(np.r_[epoch+t, p, q])
            odom = Odometry(header=Header(seq=i, stamp=imu.header.stamp, frame_id="world"), child_frame_id="body")
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            bag.write("/sim/truth/odom", odom, imu.header.stamp)
        for frame in range(int(args.duration*args.frame_rate)):
            start = frame*period
            rays, offsets, lines, reflectivity = templates[frame%len(templates)]
            if args.renderer=="bvh":
                positions,rotations=trajectory_pose_samples(start+offsets*1e-9,args.duration,args.static_seconds,args.amplitude)
                origins=positions+np.einsum("nij,j->ni",rotations,translation)
                directions=np.einsum("nij,nj->ni",rotations,rays@extrinsic.T)
                ranges=renderer.scan_world(directions,origins,config["preprocess"]["blind"])
            else:
                ranges = np.zeros(len(rays))
                groups = np.floor(offsets.astype(float)*1e-6/args.subscan_ms).astype(int)
                for g in np.unique(groups):
                    mask = groups == g
                    midpoint = 0.5*(offsets[mask].min()+float(offsets[mask].max()))*1e-9
                    p, _, _, rot, _ = trajectory(start+midpoint, args.duration, args.static_seconds, args.amplitude)
                    origin = p+rot@translation; sensor_rot = rot@extrinsic
                    ranges[mask] = renderer.scan(rays[mask], origin, sensor_rot, config["preprocess"]["blind"])
            hit = ranges > 0
            ranges[hit] += rng.normal(0, args.range_noise_m, int(hit.sum()))
            ranges = np.clip(ranges, 0, args.max_range)
            pts = np.zeros(len(rays), dtype=POINT_DTYPE)
            pts["offset_time"] = offsets
            pts["xyz"] = rays*ranges[:, None]
            pts["line"] = lines
            # Reflectivity remains a template proxy, not reconstructed material reflectance.
            pts["reflectivity"] = np.where(hit, reflectivity, 0)
            msg = pack_scan(frame, epoch+start, pts)
            # Record at acquisition end, while header remains scan start.
            bag.write(lidar_topic, msg, genpy.Time.from_sec(epoch+start+period))
            ray_counts.append(len(rays)); hit_counts.append(int(hit.sum()))
            if frame%25 == 0:
                print("Generated %d/%d scans" % (frame+1, int(args.duration*args.frame_rate)), flush=True)
    save_trajectory(args.output/"truth.tum", truth)
    write_json(args.output/"simulation.json", {
        "map": str(args.map.resolve()), "template_bag": str(args.template_bag.resolve()),
        "config": str(config_path.resolve()), "parameters": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "surface_model_report":getattr(renderer,"surface_report",None),
        "map_points": len(xyz), "template_frames": len(templates), "scans": len(ray_counts),
        "ray_count_mean": float(np.mean(ray_counts)), "hit_fraction": float(np.sum(hit_counts)/np.sum(ray_counts)),
        "lidar_topic": lidar_topic, "imu_topic": imu_topic, "wall_time_s": time.monotonic()-started,
        "truth": "Analytic IMU pose in fixed PCD world; not ground truth for original forest flight.",
        "limitations": [
            "Templates contain only observed return directions; missing original beam directions are not recovered.",
            "BVH/raster queries approximate forest geometry with fixed-radius PCA surfel disks; unobserved geometry remains absent.",
            "BVH uses each point analytic pose; raster uses subscan midpoint poses controlled by subscan-ms.",
            "Reflectivity is copied from observed templates, not a material/optical model.",
            "Noise values are explicit experimental inputs, not fully calibrated sensor noise.",
            "Analytic pose benchmark has no flight dynamics, control or collision response."]})
    print("Generated:", args.output/"sensors.bag")

if __name__ == "__main__":
    main()
