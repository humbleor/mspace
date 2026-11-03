#!/usr/bin/env python3
"""Configuration-driven PX4 fleet in one forest world."""

import argparse
from collections import deque
from dataclasses import dataclass, field
import json, os, shutil, signal, subprocess, sys, threading, time, traceback
from pathlib import Path
import xml.etree.ElementTree as ET
import xmlrpc.client
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Simulation/forest_lio/scripts"))
from common import POINT_DTYPE, save_trajectory, write_json, summary
from renderer import read_pcd
from bvh_renderer import BVHRenderer
from scene import to_local
from fleet import load_fleet, allocate_ports, gazebo_environment, bounded_call
from fleet_visualization import prepare_rviz, route_markers, vehicle_label
from fleet_evidence import all_ready, all_settled, pair_separation, audit_graph
from runtime_model import prepare_model, compose_world
from module_launch import prepare_modules
from motion import lidar_motion
from forest_geometry import geometry_model
from fleet_startup import StartupStages, cached_geometry, open_windows, contact_message
from collision_evidence import ego_acceptance
from contact_evidence import contact_records, contact_acceptance
from mission_evidence import ordered_arrivals
from world_frame_evidence import (
    fixed_transform,
    command_round_trip,
    initial_anchor,
    cloud_alignment,
)
from process_lifecycle import stop_owned, process_identity


@dataclass
class Aircraft:
    spec: object
    output: Path
    goals: np.ndarray
    truth: list = field(default_factory=list)
    history: deque = field(default_factory=lambda: deque(maxlen=1000))
    lio: list = field(default_factory=list)
    world: list = field(default_factory=list)
    fcu: list = field(default_factory=list)
    imu: list = field(default_factory=list)
    world_targets: list = field(default_factory=list)
    fcu_targets: list = field(default_factory=list)
    clouds: list = field(default_factory=list)
    contacts: list = field(default_factory=list)
    contact_times: list = field(default_factory=list)
    state_history: list = field(default_factory=list)
    frames: list = field(default_factory=list)
    pipeline: list = field(default_factory=list)
    arrivals: list = field(default_factory=list)
    state: object = None
    ready: bool = False
    ready_at: float = None
    takeoff: bool = False
    armed_at: float = None
    trigger_at: float = None
    mission_state: str = "waiting"
    mission_events: list = field(default_factory=list)
    settled_at: float = None
    last_service: float = 0.0
    frame: int = 0
    next_scan: float = None
    command_id: int = 0
    received: dict = field(default_factory=dict)
    module_alignment: dict = field(default_factory=dict)
    accepted: np.ndarray = None
    lock: object = field(default_factory=threading.RLock)

    def __post_init__(self):
        self.accepted = self.goals.copy()


def pose_row(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    return [m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def target_row(m):
    return {
        "stamp": m.header.stamp.to_sec(),
        "mask": int(m.type_mask),
        "position": [m.position.x, m.position.y, m.position.z],
        "velocity": [m.velocity.x, m.velocity.y, m.velocity.z],
        "yaw": float(m.yaw),
    }


def world_evidence(c, map_points, tree=None):
    """Nonzero placements require absolute world-cloud alignment."""
    r = {}
    errors = []
    try:
        r["anchor"] = initial_anchor(
            c.world,
            c.ready_at,
            expected_position=c.spec.initial[:3],
            expected_yaw=c.spec.initial[3],
        )
        r["transform"] = fixed_transform(c.fcu, c.world)
        tr = r["transform"]
        r["commands"] = command_round_trip(
            c.world_targets, c.fcu_targets, tr["angle_rad"], tr["translation"]
        )
        if not c.clouds:
            raise ValueError("No world cloud sample")
        r["cloud"] = cloud_alignment(c.clouds[0], map_points, tree=tree)
        errors = [k for k, v in r.items() if not v["passed"]]
    except Exception as exc:
        errors.append(str(exc))
    r.update(passed=not errors, errors=errors)
    return r


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--fleet",
        type=Path,
        required=True,
        help="JSON vehicles: uav_id, instance, initial [x,y,z,yaw], world goals or grid",
    )
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--stage", choices=["hover", "forest"], default="forest")
    ap.add_argument(
        "--scene", type=Path, default=ROOT / "artifacts/forest_lio/route_125_155.json"
    )
    ap.add_argument(
        "--template-bag",
        type=Path,
        default=Path.home() / "bagfiles/yxb_20260208-104116.bag",
    )
    ap.add_argument(
        "--px4", type=Path, default=ROOT / "artifacts/px4_forest/PX4-Autopilot"
    )
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument(
        "--map-height",
        type=float,
        default=1.5,
        help="Scene map vertical reference, not control takeoff height",
    )
    ap.add_argument("--minimum-separation", type=float, default=1.2)
    ap.add_argument("--max-truth-gap", type=float, default=0.05)
    ap.add_argument("--render-threads", type=int, default=4)
    ap.add_argument("--forest-geometry", type=Path)
    ap.add_argument(
        "--no-tree-collision",
        action="store_true",
        help="Diagnostic only; cannot pass forest acceptance",
    )
    ap.add_argument(
        "--gui", choices=["both", "gazebo", "rviz", "none"], default="both",
        help="Visualization windows (default: both); none for headless runs",
    )
    ap.add_argument("--keep-open", action="store_true")
    ap.add_argument(
        "--prepare-only",
        action="store_true",
        help="Generate resources without launching processes",
    )
    return ap


def main(argv=None):
    ap = parser()
    a = ap.parse_args(argv)
    try:
        specs = load_fleet(a.fleet)
        if (
            not all(
                np.isfinite(x)
                for x in (
                    a.duration,
                    a.speed,
                    a.map_height,
                    a.minimum_separation,
                    a.max_truth_gap,
                )
            )
            or a.duration <= 10
            or not 0 < a.speed <= 1
            or a.minimum_separation < 0.8
            or a.max_truth_gap <= 0
            or a.render_threads < 1
        ):
            raise ValueError(
                "Invalid limits: duration>10, 0<speed<=1, separation>=0.8, positive truth gap/threads"
            )
        for i, v in enumerate(specs):
            for w in specs[i + 1 :]:
                if (
                    np.linalg.norm(np.asarray(v.initial[:3]) - w.initial[:3])
                    < a.minimum_separation
                ):
                    raise ValueError("Initial placement violates minimum separation")
    except (ValueError, KeyError, TypeError) as exc:
        ap.error(str(exc))
    out = a.output.resolve()
    px4 = a.px4.resolve()
    if out.exists() and any(out.iterdir()):
        ap.error("output must be empty")
    binary = px4 / "build/px4_sitl_default/bin/px4"
    plugin = ROOT / "artifacts/px4_forest/plant_build/libmspace_forest_plant.so"
    if not binary.is_file() or not plugin.is_file():
        ap.error("Build isolated PX4/plugin first")
    if not a.prepare_only and a.gui != "none" and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        ap.error("No graphical display")
    if not a.prepare_only:
        programs = []
        if a.gui in ("both", "gazebo"):
            programs.append("gzclient")
        if a.gui in ("both", "rviz"):
            programs.append("rviz")
        for program in programs:
            if not shutil.which(program):
                ap.error(program + " is not installed")
    out.mkdir(parents=True, exist_ok=True)
    startup = StartupStages(out)
    print("Output: %s" % out, flush=True)
    (out / "ros_home").mkdir()
    ports = allocate_ports(specs)
    aircraft = []
    with startup.stage('Prepare aircraft models and navigation configs'):
        for v in specs:
            d = out / v.name
            d.mkdir()
            for name in ("Log", "PCD"):
                (d / name).mkdir()
            goals = np.vstack([[v.initial[0], v.initial[1], 1.2], v.goals])
            write_json(
                d / "route_goals.json",
                {
                    "frame": "world",
                    "generator": "generateWps" if v.grid is not None else "goals",
                    "grid": v.grid,
                    "goals": goals,
                },
            )
            c = Aircraft(v, d, goals)
            aircraft.append(c)
            c.module_alignment = prepare_modules(
                ROOT,
                d,
                goals,
                grid=v.grid,
                world_frame=True,
                uav_id=v.uav_id,
                swarm_num=len(specs),
                initial=v.initial,
                sensor_prefix=v.sensor_prefix,
            )
            write_json(d / "module_alignment.json", c.module_alignment)
            gazebo = prepare_model(
                px4,
                d,
                plugin,
                ports["vehicles"][v.name],
                v.instance,
                gps=a.stage == "hover",
                model_name=v.model_name,
                pose=[v.initial[0], v.initial[1], 0.25, 0, 0, v.initial[3]],
                sensor_prefix=v.sensor_prefix,
                physics_speed=a.speed if v.uav_id == 1 else None,
            )
        compose_world([c.output / "iris.sdf" for c in aircraft], out / "forest.world")
    env = os.environ.copy()
    env["PATH"] = (
        str(binary.parent)
        + ":"
        + ":".join(p for p in env["PATH"].split(":") if not p.startswith("/mnt/"))
    )
    env.update(
        ROS_MASTER_URI="http://127.0.0.1:%d" % ports["ros"],
        ROS_IP="127.0.0.1",
        ROS_HOME=str(out / "ros_home"),
        GAZEBO_MASTER_URI="http://127.0.0.1:%d" % ports["gazebo"],
        GAZEBO_MODEL_PATH=str(gazebo / "models") + ":/usr/share/gazebo-11/models",
        GAZEBO_PLUGIN_PATH=str(px4 / "build/px4_sitl_default/build_gazebo")
        + ":/opt/ros/noetic/lib",
        PX4_SIM_MODEL="iris",
        PX4_ESTIMATOR="ekf2",
        PX4_SIM_SPEED_FACTOR=str(a.speed),
        PX4_SIM_HOST_ADDR="127.0.0.1",
    )
    env.pop("ROS_HOSTNAME", None)
    params = {k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()}
    params["output"] = str(out)
    manifest = {
        "parameters": params,
        "ports": ports,
        "vehicles": [dict(vars(v), system_id=v.system_id) for v in specs],
        "world_frame": True,
        "gps_enabled": a.stage == "hover",
        "gazebo_external_odometry_enabled": False,
        "px4_commit": subprocess.check_output(
            ["git", "-C", str(px4), "rev-parse", "HEAD"], text=True
        ).strip(),
    }
    write_json(out / "manifest.json", manifest)
    rviz_config = prepare_rviz(ROOT, out, specs)
    print("RViz config: %s" % (ROOT / "Simulation/px4_forest/config/fleet.rviz"), flush=True)
    print("RViz run snapshot: %s" % rviz_config, flush=True)
    if a.prepare_only:
        print("Prepared %d aircraft: %s" % (len(specs), out))
        return
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.update(env)
    processes = []
    handles = []
    errors = []
    success = False
    origin = None
    renderer = None
    geometry = None
    local_map = None
    graph = {}
    reports = {}
    start = None
    flight_end = None
    snapshots = {}
    began = time.monotonic()
    rospy = None
    windows = []

    def launch(args, name, cwd=out, critical=True, child_env=None):
        h = (out / name).open("w")
        handles.append(h)
        p = subprocess.Popen(
            list(map(str, args)),
            cwd=cwd,
            env=child_env or env,
            stdout=h,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append((p, name, critical))
        write_json(
            out / "processes.json",
            {
                "controller": process_identity(os.getpid()),
                "output": str(out),
                "roots": [
                    dict(process_identity(q.pid) or {"pid": q.pid}, log=n)
                    for q, n, _ in processes
                ],
            },
        )
        return p

    def check_processes():
        for p, name, critical in processes:
            if critical and p.poll() is not None:
                raise RuntimeError(
                    "Required process exited: " + name + " (exit %s)" % p.returncode
                )

    try:
        if a.stage == "forest":
            from generate_mid360_bag import templates_from_bag
            from scan_message import pack_scan

            with startup.stage('Read and transform forest PCD'):
                scene = json.loads(a.scene.read_text())
                map_origin = np.asarray(scene["origin_world"], float)
                map_origin[2] -= a.map_height
                local_map = to_local(
                    read_pcd(scene["map"]),
                    map_origin,
                    np.asarray(scene["rotation_world_from_local"]),
                )
            with startup.stage('Build lidar renderer (normals and BVH)'):
                renderer = BVHRenderer(local_map, threads=a.render_threads)
            with startup.stage('Read Mid-360 scan templates'):
                templates = templates_from_bag(
                    a.template_bag, "/livox/lidar_192_168_2_181", 24, 0.1, window=(40, 120)
                )
            cfg = yaml.safe_load(
                (ROOT / "Modules/fast_lio2/config/mid360.yaml").read_text()
            )
            extrinsic_T = np.asarray(cfg["mapping"]["extrinsic_T"])
            extrinsic_R = np.asarray(cfg["mapping"]["extrinsic_R"]).reshape(3, 3)
            if not a.no_tree_collision:
                with startup.stage("Load cached or extract forest geometry"):
                    geometry, cache_info = cached_geometry(
                        local_map, ROOT / "artifacts/px4_forest/geometry_cache",
                        previous_root=ROOT / "artifacts/px4_forest",
                        explicit=a.forest_geometry,
                    )
                    print("Forest geometry source: %s (%s)" % (
                        cache_info["source"], cache_info.get("previous_path")
                        or cache_info.get("cache_path") or cache_info.get("path")), flush=True)
                    write_json(out / "forest_geometry.json", geometry)
                    write_json(out / "geometry_cache.json", cache_info)
        with startup.stage('Start private ROS master'):
            launch(["roscore", "-p", ports["ros"]], "roscore.log")
            proxy = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
            deadline = time.monotonic() + 30
            while True:
                try:
                    if bounded_call(lambda: proxy.getPid("/forest_swarm"), 1.0)[0] == 1:
                        break
                except (OSError, TimeoutError):
                    pass
                check_processes()
                if time.monotonic() > deadline:
                    raise RuntimeError("ROS master startup timeout")
                time.sleep(0.1)
        import rospy as ros

        rospy = ros
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from sensor_msgs import point_cloud2
        from geometry_msgs.msg import PoseStamped, Pose
        from std_msgs.msg import Header, Bool, String
        from visualization_msgs.msg import Marker, MarkerArray
        from mavros_msgs.msg import State, PositionTarget
        from mavros_msgs.srv import SetMode, CommandBool, ParamGet
        from livox_ros_driver2.msg import CustomMsg
        from prometheus_msgs.msg import SwarmCommand
        from gazebo_msgs.msg import ContactsState
        from gazebo_msgs.srv import SpawnModel
        from std_srvs.srv import Empty
        import rospkg

        rospy.set_param("/use_sim_time", True)
        rospy.init_node("forest_swarm", disable_signals=True)
        handles_ros = []

        def subscribe(c, topic, kind, callback):
            def receive(m):
                with c.lock:
                    c.received[topic] = (rospy.Time.now().to_sec(), time.monotonic())
                    callback(m)

            handles_ros.append(rospy.Subscriber(topic, kind, receive, queue_size=2000))

        for c in aircraft:
            v = c.spec
            ns = v.namespace
            sp = v.sensor_prefix

            def truth_cb(m, c=c):
                row = pose_row(m)
                c.truth.append(row)
                c.history.append(row)

            subscribe(c, sp + "/plant/odom", Odometry, truth_cb)
            subscribe(
                c,
                sp + "/mid360/imu",
                Imu,
                lambda m, c=c: c.imu.append(
                    [
                        m.header.stamp.to_sec(),
                        m.linear_acceleration.x,
                        m.linear_acceleration.y,
                        m.linear_acceleration.z,
                        m.angular_velocity.x,
                        m.angular_velocity.y,
                        m.angular_velocity.z,
                    ]
                ),
            )

            def contacts_cb(m, c=c):
                c.contact_times.append(m.header.stamp.to_sec())
                c.contacts.extend(contact_records(m))

            subscribe(c, sp + "/plant/tree_contacts", ContactsState, contacts_cb)

            def state_cb(m, c=c):
                c.state = m
                c.state_history.append(
                    [rospy.Time.now().to_sec(), m.connected, m.armed, m.mode]
                )

            subscribe(c, ns + "/mavros/state", State, state_cb)
            subscribe(
                c,
                ns + "/mavros/local_position/odom",
                Odometry,
                lambda m, c=c: c.fcu.append(pose_row(m)),
            )
            if a.stage == "forest":
                subscribe(
                    c,
                    ns + "/drone_Odom_high_freq",
                    Odometry,
                    lambda m, c=c: c.lio.append(pose_row(m)),
                )
                subscribe(
                    c,
                    ns + "/world/odom",
                    Odometry,
                    lambda m, c=c: c.world.append(pose_row(m)),
                )

                def ready_cb(m, c=c):
                    c.ready = bool(m.data)
                    if m.data and c.ready_at is None:
                        c.ready_at = rospy.Time.now().to_sec()

                subscribe(c, ns + "/frame_ready", Bool, ready_cb)
                subscribe(
                    c,
                    ns + "/world/setpoint_raw/local",
                    PositionTarget,
                    lambda m, c=c: c.world_targets.append(target_row(m)),
                )
                subscribe(
                    c,
                    ns + "/mavros/setpoint_raw/local",
                    PositionTarget,
                    lambda m, c=c: c.fcu_targets.append(target_row(m)),
                )

                def cloud_cb(m, c=c):
                    if c.ready and not c.clouds:
                        c.clouds.append(
                            np.asarray(
                                list(
                                    point_cloud2.read_points(
                                        m, field_names=("x", "y", "z"), skip_nans=True
                                    )
                                ),
                                dtype=float,
                            ).reshape(-1, 3)
                        )

                subscribe(c, ns + "/world/cloud", PointCloud2, cloud_cb)

                def goal_cb(m, c=c):
                    index = m.id - 100 + 1
                    if m.action == Marker.ADD and 1 <= index < len(c.accepted):
                        c.accepted[index] = [
                            m.pose.position.x,
                            m.pose.position.y,
                            m.pose.position.z,
                        ]

                subscribe(
                    c, "/uav_%d_ego_planner_node/goal_point" % v.uav_id, Marker, goal_cb
                )
                c.lidar_pub = rospy.Publisher(
                    sp + "/mid360/lidar", CustomMsg, queue_size=10
                )
                c.station_pub = rospy.Publisher(
                    ns + "/prometheus/swarm_command", SwarmCommand, queue_size=10
                )
                def mission_cb(m, c=c):
                    c.mission_state = m.data
                    c.mission_events.append([rospy.Time.now().to_sec(), m.data])
                    if m.data == "planning" and c.trigger_at is None:
                        c.trigger_at = rospy.Time.now().to_sec()
                        c.arrivals.append([0, c.trigger_at])
                        print(c.spec.name + ": shared OFFBOARD gate started EGO", flush=True)
                c.mission_sub = rospy.Subscriber(
                    v.namespace + "/planning/mission_state", String, mission_cb
                )
            else:
                c.setpoint_pub = rospy.Publisher(
                    ns + "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
                )
            c.set_mode = rospy.ServiceProxy(ns + "/mavros/set_mode", SetMode)
            c.arm = rospy.ServiceProxy(ns + "/mavros/cmd/arming", CommandBool)
            c.get_param = rospy.ServiceProxy(ns + "/mavros/param/get", ParamGet)
        with startup.stage('Start PX4 instances and Gazebo backend'):
            for c in aircraft:
                launch(
                    [binary, "-i", c.spec.instance, "-d", "-s", "etc/init.d-posix/rcS"],
                    c.spec.name + "/px4.log",
                    c.output,
                )
            launch(
                [
                    "gzserver",
                    "--verbose",
                    out / "forest.world",
                    "-s",
                    "/opt/ros/noetic/lib/libgazebo_ros_api_plugin.so",
                ],
                "gazebo.log",
                child_env=gazebo_environment(env),
            )
        with startup.stage("Open RViz and Gazebo windows"):
            windows.extend(open_windows(a.gui, rviz_config, launch))
            if a.gui in ("both", "rviz"):
                for c in aircraft:
                    goals_view = rospy.Publisher(
                        "/sim/view/" + c.spec.name + "/goals", MarkerArray,
                        queue_size=1, latch=True,
                    )
                    goals_view.publish(route_markers(c.goals[1:], c.spec.uav_id))
                    handles_ros.append(goals_view)
                    label_view = rospy.Publisher(
                        "/sim/view/" + c.spec.name + "/vehicle", MarkerArray,
                        queue_size=1, latch=True,
                    )

                    def publish_label(event, c=c, publisher=label_view):
                        with c.lock:
                            if not c.world:
                                return
                            row = c.world[-1]
                        publisher.publish(vehicle_label(
                            c.spec.name, row[1:4], c.spec.uav_id,
                            rospy.Time.from_sec(row[0]),
                        ))

                    handles_ros.extend([label_view, rospy.Timer(rospy.Duration(0.2), publish_label)])
            if a.gui in ("both", "rviz") and local_map is not None:
                from visualize_sitl import make_cloud

                forest_view = rospy.Publisher(
                    "/sim/view/forest", PointCloud2, queue_size=1, latch=True
                )
                forest_view.publish(make_cloud(local_map))
        with startup.stage('Start MAVROS and wait for all aircraft'):
            mavros_share = Path(rospkg.RosPack().get_path("mavros"))
            for c in aircraft:
                v = c.spec
                cfg = yaml.safe_load(
                    (ROOT / "Experiment/mavros/launch/px4_config.yaml").read_text()
                )
                cfg["conn"]["timeout"] = max(10.0, 3.0 / a.speed)
                cfg["local_position"] = {
                    "frame_id": v.name + "/fcu_local",
                    "tf": {
                        "send": True,
                        "frame_id": v.name + "/fcu_local",
                        "child_frame_id": v.name + "/base_link",
                        "send_fcu": False,
                    },
                }
                cfg["imu"]["frame_id"] = v.name + "/base_link"
                (c.output / "mavros_config.yaml").write_text(yaml.safe_dump(cfg))
                wrapper = ET.Element("launch")
                group = ET.SubElement(wrapper, "group", ns=v.name)
                inc = ET.SubElement(
                    group, "include", file=str(mavros_share / "launch/node.launch")
                )
                p = ports["vehicles"][v.name]
                for key, value in {
                    "pluginlists_yaml": str(
                        ROOT / "Experiment/mavros/launch/px4_pluginlists.yaml"
                    ),
                    "config_yaml": str(c.output / "mavros_config.yaml"),
                    "gcs_url": "",
                    "tgt_component": "1",
                    "fcu_url": "udp://127.0.0.1:%d@127.0.0.1:%d" % (p["mavros"], p["px4"]),
                    "tgt_system": str(v.system_id),
                }.items():
                    ET.SubElement(inc, "arg", name=key, value=value)
                ET.ElementTree(wrapper).write(
                    c.output / "mavros.launch", encoding="unicode"
                )
                launch(["roslaunch", c.output / "mavros.launch"], v.name + "/mavros.log")
            deadline = time.monotonic() + 100
            while not all_ready(
                c.truth and c.truth[-1][0] > 2 and c.state is not None and c.state.connected
                for c in aircraft
            ):
                check_processes()
                if time.monotonic() > deadline:
                    raise RuntimeError("Fleet physics/PX4/MAVROS startup timeout")
                time.sleep(0.05)
        # One rendering/evaluation origin, never a separate forest origin per aircraft.
        offsets = np.array(
            [np.asarray(c.truth[-1][1:4]) - c.spec.initial[:3] for c in aircraft]
        )
        origin = np.median(offsets, axis=0)
        if np.max(np.linalg.norm(offsets - origin, axis=1)) > 0.05:
            raise RuntimeError(
                "Settled placements disagree with shared world transform"
            )
        manifest["gazebo_origin"] = origin
        write_json(out / "manifest.json", manifest)
        with startup.stage('Spawn physical forest collision geometry'):
            if geometry is not None:
                model = geometry_model(geometry, origin)
                sdf = ET.Element("sdf", version="1.6")
                sdf.append(model)
                xml = ET.tostring(sdf, encoding="unicode")
                (out / "forest_collisions.sdf").write_text(xml)
                rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30)
                pose = Pose()
                pose.orientation.w = 1
                response = bounded_call(
                    lambda: rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
                        "forest_collision_geometry", xml, "", pose, "world"
                    ),
                    10.0,
                )
                if not response.success:
                    raise RuntimeError("Forest spawn failed: " + response.status_message)
                tree = ET.parse(out / "forest.world")
                tree.getroot().find("world").append(model)
                tree.write(out / "forest.world", encoding="unicode")
        with startup.stage('Start per-aircraft navigation'):
            if a.stage == "forest":
                for c in aircraft:
                    launch(
                        ["roslaunch", c.output / "modules.launch"],
                        c.spec.name + "/navigation.log",
                    )
        print("[startup] Initialization complete; entering fleet sensor/flight loop", flush=True)
        start = rospy.Time.now().to_sec()
        for c in aircraft:
            c.next_scan = start + 0.2
        deadline = time.monotonic() + a.duration / a.speed + 180
        last_print = -10
        last_hover = -1
        while rospy.Time.now().to_sec() - start < a.duration:
            check_processes()
            if time.monotonic() > deadline:
                raise RuntimeError("Fleet wall-clock timeout")
            now = rospy.Time.now().to_sec()
            for c in aircraft:
                with c.lock:
                    if c.contacts:
                        raise RuntimeError(contact_message(c.spec.name, c.contacts[0]))
                    if c.takeoff and a.stage == "forest" and not c.ready:
                        raise RuntimeError(c.spec.name + ": frame alignment lost")
                    # frame_ready and goal markers are event topics, not periodic sources.
                    periodic = [
                        c.spec.sensor_prefix + "/plant/odom",
                        c.spec.sensor_prefix + "/mid360/imu",
                        c.spec.sensor_prefix + "/plant/tree_contacts",
                        c.spec.namespace + "/mavros/state",
                        c.spec.namespace + "/mavros/local_position/odom",
                    ]
                    if a.stage == "forest":
                        periodic += [
                            c.spec.namespace + "/drone_Odom_high_freq",
                            c.spec.namespace + "/world/odom",
                        ]
                    if now - start > 8:
                        for topic in periodic:
                            received_sim, received_wall = c.received.get(topic, (-1, 0))
                            if now - received_sim > (
                                3.0 if topic.endswith("/state") else 1.0
                            ) or time.monotonic() - received_wall > max(
                                5.0, 2.0 / a.speed
                            ):
                                raise RuntimeError(
                                    c.spec.name + ": stale source " + topic
                                )
                    history = np.asarray(c.history).copy()
                if (
                    a.stage == "forest"
                    and len(history) > 1
                    and history[-1, 0] >= c.next_scan
                ):
                    if c.next_scan - 0.1 < history[0, 0] or now - c.next_scan > 0.4:
                        raise RuntimeError(
                            c.spec.name + ": sensor backlog/history overflow"
                        )
                    rays, offsets, lines, refl = templates[c.frame % len(templates)]
                    cycle = time.monotonic()
                    origins, directions = lidar_motion(
                        history,
                        c.next_scan - 0.1 + offsets * 1e-9,
                        origin,
                        extrinsic_T,
                        extrinsic_R,
                        rays,
                    )
                    interp = time.monotonic() - cycle
                    render_start = time.monotonic()
                    ranges = renderer.scan_world(directions, origins, 0.5)
                    render_s = time.monotonic() - render_start
                    points = np.zeros(len(rays), dtype=POINT_DTYPE)
                    points["offset_time"] = offsets
                    points["xyz"] = rays * ranges[:, None]
                    points["line"] = lines
                    points["reflectivity"] = np.where(ranges > 0, refl, 0)
                    c.lidar_pub.publish(
                        pack_scan(
                            c.frame,
                            c.next_scan - 0.1,
                            points,
                            frame_id=c.spec.name + "/lidar",
                        )
                    )
                    c.frames.append([c.next_scan, int((ranges > 0).sum()), len(rays)])
                    c.pipeline.append(
                        [
                            c.next_scan,
                            interp,
                            render_s,
                            time.monotonic() - cycle,
                            rospy.Time.now().to_sec() - c.next_scan,
                        ]
                    )
                    c.frame += 1
                    c.next_scan += 0.1
            ready = (
                all_ready(
                    c.ready
                    and len(c.lio) > 25
                    and c.state is not None
                    and c.state.connected
                    for c in aircraft
                )
                if a.stage == "forest"
                else now - start > 8
            )
            for c in aircraft:
                if a.stage == "hover" and now - last_hover >= 0.018:
                    target = PositionTarget(
                        header=Header(
                            stamp=rospy.Time.now(), frame_id=c.spec.name + "/fcu_local"
                        )
                    )
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.type_mask = (
                        PositionTarget.IGNORE_AFX
                        | PositionTarget.IGNORE_AFY
                        | PositionTarget.IGNORE_AFZ
                        | PositionTarget.IGNORE_YAW_RATE
                    )
                    target.position.z = 1.2
                    c.setpoint_pub.publish(target)
                if ready and now - start > 8:
                    if (
                        a.stage == "forest"
                        and not c.takeoff
                        and c.station_pub.get_num_connections()
                    ):
                        c.command_id += 1
                        c.station_pub.publish(
                            SwarmCommand(
                                header=Header(stamp=rospy.Time.now()),
                                Mode=SwarmCommand.Takeoff,
                                Command_ID=c.command_id,
                                source="simulation_station",
                                yaw_ref=c.spec.initial[3],
                            )
                        )
                        c.takeoff = True
                        c.last_service = time.monotonic()
                    elif (
                        a.stage == "hover" or c.takeoff
                    ) and time.monotonic() - c.last_service > 1:
                        if c.state.mode != "OFFBOARD":
                            bounded_call(lambda: c.set_mode(custom_mode="OFFBOARD"))
                        elif not c.state.armed:
                            bounded_call(lambda: c.arm(value=True))
                        c.last_service = time.monotonic()
                if c.state is not None and c.state.armed and c.state.mode == "OFFBOARD":
                    if c.armed_at is None:
                        c.armed_at = now
                if c.armed_at is not None and (
                    not c.state.armed or c.state.mode != "OFFBOARD"
                ):
                    raise RuntimeError(c.spec.name + ": left armed OFFBOARD")
            if a.stage == "hover" and now - last_hover >= 0.018:
                last_hover = now
            if a.stage == "forest":
                for c in aircraft:
                    if c.mission_state == "blocked":
                        raise RuntimeError(c.spec.name + ": shared mission handover blocked")
                for c in aircraft:
                    if c.trigger_at is not None and c.world:
                        index = len(c.arrivals)
                        if (
                            index < len(c.goals)
                            and np.linalg.norm(
                                np.asarray(c.world[-1][1:4]) - c.accepted[index]
                            )
                            < 0.5
                        ):
                            c.arrivals.append([index, now])
                        settled = (
                            len(c.arrivals) == len(c.goals)
                            and np.linalg.norm(
                                np.asarray(c.world[-1][1:4]) - c.accepted[-1]
                            )
                            < 0.2
                        )
                        c.settled_at = (
                            (c.settled_at if c.settled_at is not None else now)
                            if settled
                            else None
                        )
            else:
                for c in aircraft:
                    settled = (
                        c.armed_at is not None
                        and c.fcu
                        and abs(c.fcu[-1][3] - 1.2) < 0.2
                        and now - c.armed_at > 6
                    )
                    c.settled_at = (
                        (c.settled_at if c.settled_at is not None else now)
                        if settled
                        else None
                    )
            if now - last_print > 5:
                print(
                    "Fleet %.1fs " % (now - start)
                    + "; ".join(
                        "%s %s armed=%s lidar=%d goals=%d/%d ready=%s"
                        % (
                            c.spec.name,
                            c.state.mode,
                            c.state.armed,
                            c.frame,
                            len(c.arrivals),
                            len(c.goals),
                            c.ready,
                        )
                        for c in aircraft
                    ),
                    flush=True,
                )
                last_print = now
            if all_settled([c.settled_at for c in aircraft], now):
                success = True
                break
            time.sleep(0.003)
        if not success:
            errors.append("Not every aircraft settled before the simulation deadline")
        bounded_call(lambda: rospy.ServiceProxy("/gazebo/pause_physics", Empty)())
        flight_end = rospy.Time.now().to_sec()
        status = bounded_call(lambda: proxy.getSystemState("/forest_swarm"))[2]
        if a.stage == "forest":
            graph = audit_graph(specs, dict(status[0]), dict(status[1]))
            if not graph["passed"]:
                success = False
                errors.extend(graph["errors"])
        if a.keep_open and success and windows:
            print(
                "Fleet settled; physics paused. Close visualization windows or Ctrl+C to save.",
                flush=True,
            )
            try:
                while any(p.poll() is None for p in windows):
                    time.sleep(0.2)
            except KeyboardInterrupt:
                pass
    except KeyboardInterrupt:
        success = False
        errors.append("Run interrupted")
    except Exception as exc:
        success = False
        errors.append(type(exc).__name__ + ": " + str(exc))
        traceback.print_exc()
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        if rospy is not None:
            try:
                from std_srvs.srv import Empty

                bounded_call(
                    lambda: rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
                )
                flight_end = rospy.Time.now().to_sec()
            except Exception:
                pass
            audit_deadline = time.monotonic() + 10.0
            for c in aircraft:

                def parameters(c=c):
                    snap = (
                        {
                            "grid_map": rospy.get_param(
                                "/uav_%d_ego_planner_node/grid_map" % c.spec.uav_id
                            )
                        }
                        if a.stage == "forest"
                        else {}
                    )
                    snap["ekf"] = {}
                    for name in (
                        "MAV_SYS_ID",
                        "EKF2_AID_MASK",
                        "EKF2_HGT_MODE",
                        "COM_OF_LOSS_T",
                    ):
                        reply = c.get_param(param_id=name)
                        if not reply.success:
                            raise RuntimeError("Unable to read PX4 " + name)
                        snap["ekf"][name] = (
                            reply.value.real
                            if name == "COM_OF_LOSS_T"
                            else reply.value.integer
                        )
                    return snap

                try:
                    remaining = audit_deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("Fleet audit deadline reached")
                    snapshots[c.spec.name] = bounded_call(
                        parameters, min(3.0, remaining)
                    )
                except Exception as exc:
                    success = False
                    errors.append(c.spec.name + ": parameter audit: " + str(exc))
        keep = [
            p.pid for p, name, _ in processes if name in ("roscore.log", "gazebo.log")
        ]
        cleanup = stop_owned(out, keep_alive=keep)
        for p, _, _ in processes:
            if p.poll() is not None:
                p.wait()
        if cleanup["remaining"]:
            success = False
            errors.append("Incomplete process cleanup")
        if rospy is not None:
            rospy.signal_shutdown("Fleet complete")
        for h in handles:
            h.close()
        traces = {}
        for c in aircraft:
            v = c.spec
            vehicle_errors = []
            if c.clouds:
                np.save(c.output / "world_cloud_sample.npy", c.clouds[0])
            raw = np.asarray(c.truth, dtype=float).reshape(-1, 8)
            if flight_end is not None:
                raw = raw[raw[:, 0] <= flight_end]
            local = raw.copy()
            if origin is not None:
                local[:, 1:4] -= origin
            traces[v.name] = local
            for name, data in (
                ("plant_world", raw),
                ("truth", local),
                ("lio", c.lio),
                ("world", c.world),
                ("fcu", c.fcu),
            ):
                save_trajectory(c.output / (name + ".tum"), data)
            for name, data, width, header in (
                ("imu", c.imu, 7, "stamp,ax_g,ay_g,az_g,gx,gy,gz"),
                ("frames", c.frames, 3, "scan_end,hits,rays"),
                (
                    "sensor_timing",
                    c.pipeline,
                    5,
                    "scan_end,interpolation_wall_s,render_wall_s,total_wall_s,lag_sim_s",
                ),
            ):
                np.savetxt(
                    c.output / (name + ".csv"),
                    np.asarray(data).reshape(-1, width),
                    delimiter=",",
                    header=header,
                )
            contact = contact_acceptance(
                c.contacts,
                len(c.contact_times),
                True,
                c.contact_times,
                (float(raw[0, 0]), float(raw[-1, 0])) if len(raw) else None,
                max_gap=a.max_truth_gap,
            )
            if not contact["passed"]:
                vehicle_errors.append("Physical contact evidence: " + contact["status"])
            world_check = (
                world_evidence(c, local_map, renderer.tree)
                if a.stage == "forest" and local_map is not None
                else None
            )
            if world_check is not None and not world_check["passed"]:
                vehicle_errors.append("World alignment evidence failed")
            collision = None
            snap = snapshots.get(v.name, {})
            ekf = snap.get("ekf", {})
            if ekf.get("MAV_SYS_ID") != v.system_id or ekf.get("COM_OF_LOSS_T", 0) <= 0:
                vehicle_errors.append("PX4 identity/watchdog audit failed")
            if a.stage == "forest":
                if ekf.get("EKF2_AID_MASK") != 24 or ekf.get("EKF2_HGT_MODE") != 3:
                    vehicle_errors.append("GNSS-denied EKF parameters failed")
                if geometry is None:
                    vehicle_errors.append(
                        "Physical forest geometry disabled/unavailable"
                    )
                if "grid_map" in snap and local_map is not None:
                    collision = ego_acceptance(
                        local_map,
                        local,
                        snap["grid_map"],
                        a.max_truth_gap,
                        tree=renderer.tree,
                    )
                    if not collision["passed"]:
                        vehicle_errors.append("Forest clearance acceptance failed")
            requested = (
                ordered_arrivals(c.world, c.goals[1:], c.trigger_at)
                if c.trigger_at is not None
                else {"complete": False}
            )
            physics = (
                ordered_arrivals(local, c.goals[1:], c.trigger_at)
                if c.trigger_at is not None
                else {"complete": False}
            )
            evidence = {"world_requested": requested, "physics_requested": physics}
            write_json(c.output / "requested_waypoint_evidence.json", evidence)
            if a.stage == "forest" and (
                not requested["complete"] or not physics["complete"]
            ):
                vehicle_errors.append("Original requested waypoints incomplete")
            final_target = c.accepted[-1] if a.stage == "forest" else c.goals[0]
            goal_error = (
                float(np.linalg.norm(local[-1, 1:4] - final_target))
                if len(local) and origin is not None
                else None
            )
            if a.stage == "forest" and (goal_error is None or goal_error >= 0.3):
                vehicle_errors.append("Final physical goal tolerance failed")
            if c.settled_at is None:
                vehicle_errors.append("Aircraft did not settle")
            r = {
                "status": "failed" if vehicle_errors else "ok",
                "errors": vehicle_errors,
                "uav_id": v.uav_id,
                "instance": v.instance,
                "system_id": v.system_id,
                "module_alignment": c.module_alignment,
                "initial": v.initial,
                "goals": c.goals,
                "accepted_goals": c.accepted,
                "arrivals": c.arrivals,
                "trigger_sim_s": c.trigger_at,
                "mission_handover_events": c.mission_events,
                "goal_error_m": goal_error,
                "frame_ready_sim_s": c.ready_at,
                "lidar_frames": c.frame,
                "imu_samples": len(c.imu),
                "state_history": c.state_history,
                "world_frame_evidence": world_check,
                "collision": collision,
                "contacts": contact,
                "parameter_checks": ekf,
                "mission_evidence": evidence,
                "sensor_pipeline": {
                    key: summary(np.asarray(c.pipeline)[:, i])
                    for i, key in enumerate(
                        [
                            "scan_end",
                            "interpolation_wall_s",
                            "render_wall_s",
                            "total_wall_s",
                            "published_lag_sim_s",
                        ]
                    )
                    if i and c.pipeline
                },
            }
            reports[v.name] = r
            write_json(c.output / "result.json", r)
            write_json(c.output / "contacts.json", c.contacts)
            write_json(c.output / "module_params.json", snap)
            if vehicle_errors:
                success = False
                errors.extend(v.name + ": " + e for e in vehicle_errors)
        pair = pair_separation(traces, a.minimum_separation, a.max_truth_gap)
        if not pair["passed"]:
            success = False
            errors.append("Inter-aircraft clearance evidence failed")
        write_json(out / "pair_separation.json", pair)
        write_json(out / "graph.json", graph)
        result = {
            "status": "ok" if success and not errors else "failed",
            "errors": errors,
            "stage": a.stage,
            "startup_timing": startup.records,
            "vehicles": reports,
            "pair_separation": pair,
            "graph_passed": graph.get("passed"),
            "cleanup": cleanup,
            "world_frame": True,
            "wall_s": time.monotonic() - began,
            "flight_end_sim_s": flight_end,
            "limits": [
                "Known configured initial placements; no online inter-UAV registration.",
                "Lidar returns contain static forest only; other UAVs are exchanged as planner trajectories.",
                "Piecewise-linear physical clearance between recorded samples.",
                "No physical flight controller or real-flight validation.",
            ],
        }
        write_json(out / "result.json", result)
        lines = [
            "# PX4 forest fleet validation",
            "",
            "Status: **" + result["status"] + "**; stage: " + a.stage + ".",
            "",
            "Aircraft | Status | Lidar frames | Physical goal error (m)",
            "--- | --- | --- | ---",
        ]
        lines += [
            "%s | %s | %d | %s"
            % (name, r["status"], r["lidar_frames"], r["goal_error_m"])
            for name, r in reports.items()
        ]
        lines += [
            "",
            "Inter-aircraft clearance: "
            + str(pair["passed"])
            + ". Cleanup: "
            + cleanup["status"]
            + ".",
            "",
        ] + ["- " + e for e in errors]
        lines += ["", "Validation limits:"] + ["- " + x for x in result["limits"]]
        (out / "report.md").write_text("\n".join(lines) + "\n")
        print(out / "result.json", flush=True)
    if not success or errors:
        raise SystemExit("Fleet SITL validation failed")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    main()
