#!/usr/bin/env python3
"""Hardware-free regression tests. Owns an isolated ROS master and scoped child processes."""
import io
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
import unittest
import xmlrpc.client

import roslaunch
import roslib.packages
import rospkg
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from prometheus_msgs.msg import DroneState, SwarmCommand
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty
from traj_utils.msg import Bspline, MultiBsplines

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "artifacts/tfmini_bridge"
OUT.mkdir(parents=True, exist_ok=True)
RP = rospkg.RosPack()

def load(package, relative, *args):
    cfg = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(Path(RP.get_path(package))/relative), cfg,
                                      argv=list(args), verbose=False)
    return cfg

def port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]

def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)

def wait_for(predicate, timeout=6, reason="condition"):
    end = time.monotonic()+timeout
    while time.monotonic() < end and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(.025)
    raise AssertionError("timed out waiting for "+reason)

def tcp_connect(port_number, timeout=6):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            return socket.create_connection(("127.0.0.1",port_number),timeout=.3)
        except OSError:
            time.sleep(.025)
    raise AssertionError("timed out waiting for TCP listener on %d"%port_number)

def spline(uid=1, tid=10):
    m = Bspline(drone_id=uid, order=3, traj_id=tid, start_time=rospy.Time.now())
    m.knots = [float(i) for i in range(8)]
    m.pos_pts = [Point(1, 2, 3), Point(2, 3, 4), Point(3, 4, 5), Point(4, 5, 6)]
    m.yaw_dt = .1
    return m

def packet(kind, sender, msg):
    b = io.BytesIO()
    msg.serialize(b)
    data = b.getvalue()
    return struct.pack("!4sIHH", b"MSB2", len(data), kind, sender)+data

class ConfigTest(unittest.TestCase):
    def test_tfmini_maps_to_unique_uav_frames_and_is_enabled(self):
        for uid in (1, 2):
            cfg = load("mavros_bringup", "launch/px4_mavros_swarm_basic.launch",
                       "uav_id:=%d"%uid, "enable_world_frame:=true")
            base = "/uav%d/mavros/"%uid
            self.assertNotIn("distance_sensor", cfg.params[base+"plugin_blacklist"].value)
            self.assertEqual(cfg.params[base+"distance_sensor/tfmini_down/frame_id"].value,
                             "uav%d/tfmini_link"%uid)
            self.assertFalse(cfg.params[base+"distance_sensor/tfmini_down/subscriber"].value)
            self.assertEqual(cfg.params[base+"distance_sensor/tfmini_down/id"].value, 0)
            self.assertFalse(cfg.params[base+"distance_sensor/tfmini_down/send_tf"].value)

    def test_bridge_mode_has_no_tx_rx_overlap_or_direct_ros_bypass(self):
        for uid in (1, 2):
            cfg = load("ego_planner", "launch_new/real_ego_run.launch",
                       "uav_id:=%d"%uid, "enable_world_frame:=true", "use_bridge:=true")
            node = next(n for n in cfg.nodes if n.type=="ego_planner_node")
            remaps = dict({tuple(p) for p in node.remap_args})
            ns = "/uav%d/bridge/"%uid
            self.assertEqual(remaps["~planning/broadcast_bspline_from_planner"], ns+"tx/bspline")
            self.assertEqual(remaps["~planning/broadcast_bspline_to_planner"], ns+"rx/bspline")
            self.assertEqual(remaps["~planning/bridge_request_current"], ns+"request_current")
            self.assertTrue(cfg.params["/"+node.name+"/fsm/use_bridge"].value)
            self.assertIn(("/uav%d/planning/swarm_trajs"%uid, ns+"tx/multi_traj"), {tuple(p) for p in node.remap_args})
            if uid == 2:
                self.assertIn(("/uav1/planning/swarm_trajs", ns+"rx/prev_multi_traj"), {tuple(p) for p in node.remap_args})

    def test_direct_ros_planner_remains_default(self):
        cfg = load("ego_planner", "launch_new/real_ego_run.launch", "uav_id:=2")
        remaps = dict(next(n for n in cfg.nodes if n.type=="ego_planner_node").remap_args)
        self.assertEqual(remaps["~planning/broadcast_bspline_from_planner"], "/broadcast_bspline")
        self.assertEqual(remaps["~planning/broadcast_bspline_to_planner"], "/broadcast_bspline")

class RuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = port()
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d"%p
        os.environ["ROS_IP"] = "127.0.0.1"
        os.environ.pop("ROS_HOSTNAME", None)
        os.environ["ROS_HOME"] = str(OUT/"ros_home")
        f = open(OUT/"master.log", "w")
        cls.master = subprocess.Popen(["roscore", "-p", str(p)], stdout=f,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        cls.addClassCleanup(stop, cls.master)
        cls.addClassCleanup(f.close)
        end = time.monotonic()+30
        while True:
            try:
                with xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]) as proxy:
                    proxy.getPid("/test")
                break
            except Exception:
                if time.monotonic()>end:
                    raise
                time.sleep(.1)
        rospy.init_node("real_io_test", anonymous=True, disable_signals=True)

    def launch(self, package, binary, name, params, remaps=()):
        for key, val in params.items():
            rospy.set_param("/"+name+"/"+key, val)
        executable = roslib.packages.find_node(package, binary)[0]
        f = open(OUT/(name+".log"), "w")
        self.addCleanup(f.close)
        proc = subprocess.Popen([executable, "__name:="+name]+list(remaps), stdout=f,
                                stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(stop, proc)
        return proc

    def sub(self, topic, typ):
        data = []
        sub = rospy.Subscriber(topic, typ, data.append, queue_size=100)
        self.addCleanup(sub.unregister)
        return data

    def pub(self, topic, typ):
        p = rospy.Publisher(topic, typ, queue_size=10)
        self.addCleanup(p.unregister)
        return p

    def test_takeoff_height_lowercase_controls_synthetic_z_target(self):
        targets = self.sub("/test/takeoff_target", PositionTarget)
        self.launch("prometheus_swarm_control", "swarm_controller", "takeoff_test",
                    {"uav_id":1,"swarm_num":1,"controller_flag":1,
                     "require_frame_ready":False,"takeoff_height":1.7,"flag_printf":False},
                    ["/uav1/mavros/setpoint_raw/local:=/test/takeoff_target"])
        state = self.pub("/uav1/prometheus/drone_state", DroneState)
        commands = self.pub("/uav1/prometheus/swarm_command", SwarmCommand)
        wait_for(lambda: state.get_num_connections() and commands.get_num_connections())
        s = DroneState(connected=True, armed=True, landed=False, mode="OFFBOARD")
        s.position = [1,2,3]
        for _ in range(15):
            state.publish(s)
            time.sleep(.02)
        cmd = SwarmCommand(Mode=SwarmCommand.Takeoff, Command_ID=1)
        for _ in range(30):
            state.publish(s)
            commands.publish(cmd)
            time.sleep(.02)
        motion = [m for m in targets if m.type_mask != 0x4000]
        self.assertTrue(motion)
        self.assertAlmostEqual(motion[-1].position.z, 4.7, places=4)

    def test_mavros_receives_tfmini_for_each_uav(self):
        from pymavlink.dialects.v20 import common as mav
        for uid in (1, 2):
            with self.subTest(uid=uid):
                listen, remote = port(socket.SOCK_DGRAM), port(socket.SOCK_DGRAM)
                name = "fake_mavros_%d"%uid
                cfg = load("mavros_bringup", "launch/px4_mavros_swarm_basic.launch",
                           "uav_id:=%d"%uid, "enable_world_frame:=true", "tfmini_id:=%d"%(uid+4))
                base = "/uav%d/mavros/"%uid
                params = {k[len(base):]:v.value for k,v in cfg.params.items() if k.startswith(base)}
                params.update({"fcu_url":"udp://127.0.0.1:%d@127.0.0.1:%d"%(listen,remote),
                               "gcs_url":"","plugin_blacklist":[],
                               "plugin_whitelist":["distance_sensor"],"target_system_id":uid})
                values = self.sub("/"+name+"/distance_sensor/tfmini_down", Range)
                proc = self.launch("mavros", "mavros_node", name, params)
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                    sock.bind(("127.0.0.1",remote))
                    class Writer:
                        def write(self, data):
                            return sock.sendto(data,("127.0.0.1",listen))
                    link = mav.MAVLink(Writer(),srcSystem=uid,srcComponent=1)
                    for _ in range(5):
                        link.distance_sensor_send(1000,40,1200,999,mav.MAV_DISTANCE_SENSOR_LASER,0,
                                                  mav.MAV_SENSOR_ROTATION_PITCH_270,0)
                        time.sleep(.08)
                    self.assertFalse(values,"unmapped sensor ID must not leak into TFmini stream")
                    end = time.monotonic()+8
                    while not values and time.monotonic()<end:
                        link.heartbeat_send(mav.MAV_TYPE_QUADROTOR,mav.MAV_AUTOPILOT_PX4,0,0,0)
                        link.distance_sensor_send(1000,40,1200,100+uid*25,
                                                  mav.MAV_DISTANCE_SENSOR_LASER,uid+4,
                                                  mav.MAV_SENSOR_ROTATION_PITCH_270,0)
                        time.sleep(.08)
                self.assertTrue(values,"actual MAVROS must emit configured TFmini Range")
                self.assertAlmostEqual(values[-1].range,1+uid*.25)
                self.assertEqual(values[-1].header.frame_id,"uav%d/tfmini_link"%uid)
                self.assertAlmostEqual(values[-1].min_range,.4)
                stop(proc)

    def bridge(self, uid, tcp_listen, tcp_peer, udp_listen, udp_peer, name):
        return self.launch("rosmsg_tcp_bridge","bridge_node",name,
                           {"drone_id":uid,"swarm_num":2,"next_drone_ip":"127.0.0.1",
                            "broadcast_ip":"127.0.0.1","bind_ip":"127.0.0.1",
                            "tcp_port":tcp_listen,"tcp_peer_port":tcp_peer,
                            "udp_port":udp_listen,"udp_peer_port":udp_peer})

    def test_same_master_transport_no_feedback_and_peer_restart(self):
        t1,t2,u1,u2 = port(),port(),port(socket.SOCK_DGRAM),port(socket.SOCK_DGRAM)
        a = self.bridge(1,t1,t2,u1,u2,"bridge_a")
        b = self.bridge(2,t2,t1,u2,u1,"bridge_b")
        states = self.sub("/uav1/bridge_connection_state",Bool)
        rxm = self.sub("/uav2/bridge/rx/prev_multi_traj",MultiBsplines)
        rxu = self.sub("/uav2/bridge/rx/bspline",Bspline)
        txm = self.pub("/uav1/bridge/tx/multi_traj",MultiBsplines)
        txu = self.pub("/uav1/bridge/tx/bspline",Bspline)
        wait_for(lambda: states and states[-1].data,reason="TCP connection")
        wait_for(lambda: txm.get_num_connections() and txu.get_num_connections())
        m = spline()
        multi = MultiBsplines(drone_id_from=1,traj=[Bspline(),m])
        txm.publish(multi)
        txu.publish(m)
        wait_for(lambda: rxm and rxu,reason="TCP and UDP payloads")
        self.assertEqual(rxm[-1].traj[1].pos_pts[2].z,5)
        self.assertEqual(rxu[-1].traj_id,10)
        time.sleep(.4)
        self.assertEqual(len(rxm),1,"RX must not be fed back to TX")
        txu.publish(m)
        time.sleep(.2)
        self.assertEqual(len(rxu),1,"duplicate immutable trajectory must not republish")
        stop(b)
        wait_for(lambda: states and not states[-1].data,reason="disconnect state")
        b = self.bridge(2,t2,t1,u2,u1,"bridge_b_restart")
        wait_for(lambda: states and states[-1].data,reason="reconnect")
        m = spline(tid=11)
        txm.publish(MultiBsplines(drone_id_from=1,traj=[Bspline(),m]))
        wait_for(lambda: len(rxm)==2,reason="payload after restart")
        self.assertIsNone(a.poll())
        self.assertIsNone(b.poll())

    def test_three_uav_periodic_chain_keeps_newer_udp_and_successor(self):
        proc=self.launch("ego_planner","bridge_chain_test","chain_merge_test",{})
        self.assertEqual(proc.wait(timeout=8),0,"actual EGO chain callback regression failed; see chain_merge_test.log")

    def test_incomplete_frame_timeout_receiver_state_and_no_peer_shutdown(self):
        t,u=port(),port(socket.SOCK_DGRAM)
        receiver=self.bridge(2,t,port(),u,port(socket.SOCK_DGRAM),"bridge_timeout_test")
        states=self.sub("/uav2/bridge_connection_state",Bool)
        rx=self.sub("/uav2/bridge/rx/prev_multi_traj",MultiBsplines)
        wait_for(lambda: bool(states),reason="initial receiver state")
        self.assertFalse(states[-1].data)
        with tcp_connect(t) as tcp:
            wait_for(lambda: states[-1].data,reason="receiver TCP connected")
            tcp.sendall(b"MSB")
            wait_for(lambda: not states[-1].data,timeout=4,reason="partial-frame timeout state")
        self.assertFalse(rx,"incomplete frame must not publish")
        with tcp_connect(t) as tcp:
            tcp.sendall(packet(1,1,MultiBsplines(drone_id_from=1,traj=[Bspline(),spline(tid=90)])))
            wait_for(lambda: len(rx)==1,reason="valid frame after timed-out connection")
        wait_for(lambda: not states[-1].data,reason="receiver observes peer close")
        sender=self.bridge(1,port(),port(),port(socket.SOCK_DGRAM),port(socket.SOCK_DGRAM),"bridge_no_peer_test")
        time.sleep(.3)
        started=time.monotonic();stop(sender)
        self.assertLess(time.monotonic()-started,2.,"shutdown with no peer must remain bounded")
        self.assertIsNone(receiver.poll())

    def test_late_bridge_requests_original_current_planner_chain(self):
        name="current_chain_planner"
        cfg=load("ego_planner","launch_new/real_ego_run.launch", "uav_id:=1", "use_bridge:=true")
        node=next(n for n in cfg.nodes if n.type=="ego_planner_node")
        base="/"+node.name+"/"
        params={k[len(base):]:v.value for k,v in cfg.params.items() if k.startswith(base)}
        params.update({"fsm/flight_type":1,"fsm/thresh_replan_time":1000.,
                       "fsm/thresh_no_replan_meter":10.,"grid_map/resolution":.2,
                       "grid_map/map_size_x":10.,"grid_map/map_size_y":10.,
                       "grid_map/map_size_z":4.,"grid_map/odom_depth_timeout":100.})
        remaps=[a+":="+b for a,b in node.remap_args]
        planner=self.launch("ego_planner","ego_planner_node",name,params,remaps)
        own=[]
        initial=[]
        initial_sub=rospy.Subscriber("/uav1/bridge/tx/bspline",Bspline,initial.append,queue_size=100)
        self.addCleanup(initial_sub.unregister)
        own_sub=rospy.Subscriber("/uav1/bridge/tx/multi_traj",MultiBsplines,own.append,queue_size=100)
        self.addCleanup(own_sub.unregister)
        odom=self.pub("/uav1/mavros/local_position/odom",Odometry)
        goal=self.pub("/uav1/prometheus/ego/goal",PoseStamped)
        request=self.pub("/uav1/bridge/request_current",Empty)
        wait_for(lambda: odom.get_num_connections() and goal.get_num_connections() and own_sub.get_num_connections() and initial_sub.get_num_connections(),timeout=15)
        s=Odometry(); s.pose.pose.position.z=1.; s.pose.pose.orientation.w=1.
        for _ in range(10):
            s.header.stamp=rospy.Time.now(); odom.publish(s); time.sleep(.03)
        g=PoseStamped(); g.pose.position=Point(2,0,1); g.pose.orientation.w=1.
        goal.publish(g)
        wait_for(lambda: bool(initial),timeout=15,reason="real planner initial trajectory")
        first=initial[-1]
        # Planning preceded either bridge. Recovery must rebuild current data, not replay bridge cache.
        t1,t2,u1,u2=port(),port(),port(socket.SOCK_DGRAM),port(socket.SOCK_DGRAM)
        a=self.bridge(1,t1,t2,u1,u2,"late_bridge_a")
        rx=self.sub("/uav2/bridge/rx/prev_multi_traj",MultiBsplines)
        b=self.bridge(2,t2,t1,u2,u1,"late_bridge_b")
        wait_for(lambda: bool(rx),timeout=6,reason="current chain after late TCP connection")
        self.assertEqual(rx[-1].traj[1].start_time,first.start_time)
        self.assertEqual(rx[-1].traj[1].traj_id,first.traj_id)
        end=first.start_time.to_sec()+first.knots[len(first.pos_pts)]
        wait_for(lambda: rospy.Time.now().to_sec()>end+.2,timeout=15,reason="trajectory expiration")
        time.sleep(.6)
        count=len(own)
        for _ in range(4): request.publish(Empty()); time.sleep(.15)
        self.assertEqual(len(own),count,"expired trajectory must not be refreshed")
        self.assertIsNone(planner.poll()); self.assertIsNone(a.poll()); self.assertIsNone(b.poll())

    def test_rejects_non_cubic_and_degenerate_ego_trajectories(self):
        t,u = port(),port(socket.SOCK_DGRAM)
        proc = self.bridge(2,t,port(),u,port(socket.SOCK_DGRAM),"bridge_contract_test")
        rxm = self.sub("/uav2/bridge/rx/prev_multi_traj",MultiBsplines)
        rxu = self.sub("/uav2/bridge/rx/bspline",Bspline)
        states = self.sub("/uav2/bridge_connection_state",Bool)
        wait_for(lambda: bool(states),reason="bridge initialization")
        invalid=[]
        for order in (1,2,4,5):
            m=spline(tid=40+order)
            m.order=order
            m.pos_pts=m.pos_pts[:order+1] if order<3 else m.pos_pts+[Point()]*(order-3)
            m.knots=[float(i) for i in range(len(m.pos_pts)+order+1)]
            invalid.append(m)
        for knots in ([0.0]*8, [0.,1.,2.,3.,3.,5.,6.,7.],
                      [float(i)-10 for i in range(8)]):
            m=spline(tid=50+len(invalid)); m.knots=knots; invalid.append(m)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            for m in invalid:
                sock.sendto(packet(2,1,m),("127.0.0.1",u))
                with tcp_connect(t) as tcp:
                    tcp.sendall(packet(1,1,MultiBsplines(drone_id_from=1,traj=[Bspline(),m])))
                time.sleep(.05)
            time.sleep(.2)
            self.assertFalse(rxu,"invalid spline must never reach EGO UDP callback")
            self.assertFalse(rxm,"invalid spline must never reach EGO TCP callback")
            sock.sendto(packet(2,1,spline(tid=70)),("127.0.0.1",u))
        wait_for(lambda: len(rxu)==1,reason="valid spline after invalid inputs")
        self.assertIsNone(proc.poll())

    def test_tcp_split_combined_and_udp_malformed_self_packets(self):
        t,u = port(),port(socket.SOCK_DGRAM)
        proc = self.bridge(2,t,port(),u,port(socket.SOCK_DGRAM),"bridge_packet_test")
        rxm = self.sub("/uav2/bridge/rx/prev_multi_traj",MultiBsplines)
        rxu = self.sub("/uav2/bridge/rx/bspline",Bspline)
        def connect():
            s=socket.socket()
            try:
                s.connect(("127.0.0.1",t))
                return s
            except OSError:
                s.close()
                return None
        holder=[]
        wait_for(lambda: (holder.append(s) or True) if (s:=connect()) else False,
                 reason="test bridge TCP listener")
        with holder[0] as s:
            p=packet(1,1,MultiBsplines(drone_id_from=1,traj=[Bspline(),spline(tid=21)]))
            s.sendall(p[:3]); time.sleep(.08); s.sendall(p[3:17]);time.sleep(.08);s.sendall(p[17:])
            wait_for(lambda: len(rxm)==1,reason="fragmented TCP frame")
            p2=packet(1,1,MultiBsplines(drone_id_from=1,traj=[Bspline(),spline(tid=22)]))
            p3=packet(1,1,MultiBsplines(drone_id_from=1,traj=[Bspline(),spline(tid=23)]))
            s.sendall(p2+p3)
            wait_for(lambda: len(rxm)==3,reason="coalesced TCP frames")
        time.sleep(.1)
        with socket.socket() as s:
            s.connect(("127.0.0.1",t))
            s.sendall(struct.pack("!4sIHH",b"MSB2",0xffffffff,1,1))
        time.sleep(.15)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            dst=("127.0.0.1",u)
            good=packet(2,1,spline(tid=30))
            for data in [b"x",good[:-3],struct.pack("!4sIHH",b"MSB2",16,2,1)+b"\xff"*16,
                         packet(2,2,spline(uid=2)),packet(2,1,spline(uid=2))]:
                s.sendto(data,dst)
            time.sleep(.15)
            self.assertEqual(rxu,[])
            s.sendto(good,dst)
        wait_for(lambda: len(rxu)==1,reason="valid UDP after rejected packets")
        self.assertIsNone(proc.poll())

if __name__=="__main__":
    unittest.main(verbosity=2)
