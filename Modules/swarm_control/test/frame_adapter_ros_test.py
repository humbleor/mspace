#!/usr/bin/env python3
"""Synthetic ROS transport tests. Owns one isolated roscore; never connects to hardware."""
import math
import os
import signal
import socket
import struct
import subprocess
import time
import unittest
import xmlrpc.client
import rospy
import roslib.packages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from prometheus_msgs.msg import DroneState
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from prometheus_msgs.msg import SwarmCommand

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
OUT = os.path.join(ROOT, "artifacts/swarm_world_frame")
os.makedirs(OUT, exist_ok=True)


def stop(proc):
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


class AdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
        os.environ["ROS_IP"] = "127.0.0.1"
        os.environ.pop("ROS_HOSTNAME", None)
        os.environ["ROS_HOME"] = os.path.join(OUT, "ros_home")
        cls.master_log = open(os.path.join(OUT, "master.log"), "w")
        cls.master = subprocess.Popen(["roscore", "-p", str(port)], stdout=cls.master_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        cls.addClassCleanup(stop, cls.master)
        deadline = time.monotonic()+40  # transient load delays roscore binding
        while True:
            try:
                with xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]) as master:
                    master.getPid("/test")
                break
            except Exception:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.1)
        rospy.init_node("frame_adapter_test", anonymous=True, disable_signals=True)

    def setUp(self):
        self.messages = {key: [] for key in ("ready", "odom", "state", "cloud", "cmd")}
        self.subs = []
        for key, topic, typ in [
            ("ready", "/uav2/frame_ready", Bool), ("odom", "/uav2/world/odom", Odometry),
            ("state", "/uav2/prometheus/drone_state", DroneState),
            ("cloud", "/uav2/world/cloud", PointCloud2),
            ("cmd", "/uav2/mavros/setpoint_raw/local", PositionTarget)]:
            self.subs.append(rospy.Subscriber(topic, typ, lambda m, k=key: self.messages[k].append(m), queue_size=100))
        self.fcu_pub = rospy.Publisher("/uav2/mavros/local_position/odom", Odometry, queue_size=10)
        self.lio_pub = rospy.Publisher("/uav2/drone_Odom_high_freq", Odometry, queue_size=10)
        self.raw_pub = rospy.Publisher("/uav2/local/drone_state", DroneState, queue_size=10)
        self.cloud_pub = rospy.Publisher("/uav2/drone_cloud_registered", PointCloud2, queue_size=10)
        self.cmd_pub = rospy.Publisher("/uav2/world/setpoint_raw/local", PositionTarget, queue_size=10)
        self.fcu = Odometry()
        self.fcu.header.frame_id = "uav2/fcu_local"
        self.fcu.child_frame_id = "uav2/base_link"
        self.fcu.pose.pose.position.x, self.fcu.pose.pose.position.y, self.fcu.pose.pose.position.z = 2, -3, 1
        self.fcu.pose.pose.orientation.z = math.sin(math.pi/4)
        self.fcu.pose.pose.orientation.w = math.cos(math.pi/4)
        self.lio = Odometry()
        self.lio.header.frame_id = "uav2/lio_init"
        self.lio.child_frame_id = "uav2/lio_body"
        self.lio.pose.pose.position.x, self.lio.pose.pose.position.y = 7, 8
        self.lio.pose.pose.orientation.w = 1
        self.raw = DroneState(connected=True)
        self.raw.header.frame_id = "uav2/fcu_local"
        self.node = None
        self.log = open(os.path.join(OUT, self._testMethodName+".log"), "w")
        self.addCleanup(self.log.close)
        for name, value in {"uav_id": 2, "init_y": 5., "stable_duration": .3,
                            "source_timeout": .3, "receive_timeout": .3, "max_source_skew": .15,
                            "body_in_imu/translation": [0., 0., 0.],
                            "body_in_imu/rpy": [0., 0., 0.]}.items():
            rospy.set_param("/test_adapter/"+name, value)
        nodes = roslib.packages.find_node("prometheus_swarm_control", "swarm_frame_adapter")
        self.assertTrue(nodes, "adapter executable not built")
        self.node = subprocess.Popen([nodes[0], "__name:=test_adapter"], stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(stop, self.node)
        deadline = time.monotonic()+4
        while self.fcu_pub.get_num_connections() == 0:
            if time.monotonic() > deadline:
                self.fail("adapter did not subscribe")
            time.sleep(.02)
        self.addCleanup(self.cleanup_topics)

    def cleanup_topics(self):
        for pub in (self.fcu_pub, self.lio_pub, self.raw_pub, self.cloud_pub, self.cmd_pub):
            pub.unregister()
        for sub in self.subs:
            sub.unregister()

    def pump(self, duration=.65, stamp_offset=0):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            stamp = rospy.Time.now()+rospy.Duration(stamp_offset)
            self.fcu.header.stamp = stamp
            self.lio.header.stamp = stamp
            self.raw.header.stamp = stamp
            self.fcu_pub.publish(self.fcu)
            self.lio_pub.publish(self.lio)
            self.raw_pub.publish(self.raw)
            time.sleep(.02)

    def ready(self):
        return bool(self.messages["ready"] and self.messages["ready"][-1].data)

    def restart_adapter(self):
        """Re-spawn the adapter so params written after setUp take effect."""
        stop(self.node)
        deadline = time.monotonic()+4
        while self.fcu_pub.get_num_connections():
            if time.monotonic() > deadline:
                self.fail("adapter did not release its subscriptions")
            time.sleep(.02)
        nodes = roslib.packages.find_node("prometheus_swarm_control", "swarm_frame_adapter")
        self.node = subprocess.Popen([nodes[0], "__name:=test_adapter"], stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(stop, self.node)
        deadline = time.monotonic()+4
        while self.fcu_pub.get_num_connections() == 0:
            if time.monotonic() > deadline:
                self.fail("adapter did not resubscribe")
            time.sleep(.02)

    def publish_cloud(self, points):
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = "uav2/lio_init", rospy.Time.now()
        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, len(points), 16, 16*len(points)
        cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(("x", "y", "z", "intensity"))]
        cloud.data = b"".join(struct.pack("<ffff", p[0], p[1], p[2], p[3]) for p in points)
        self.cloud_pub.publish(cloud)
        self.pump(.1)
        return self.messages["cloud"][-1]

    def test_transforms_cloud_and_commands(self):
        self.pump()
        self.assertTrue(self.ready())
        odom = self.messages["odom"][-1]
        self.assertAlmostEqual(odom.pose.pose.position.x, 0, places=5)
        self.assertAlmostEqual(odom.pose.pose.position.y, 5, places=5)
        self.assertEqual(odom.child_frame_id, "uav2/base_link")
        self.fcu.twist.twist.linear.x = 1
        self.pump(.12)
        self.assertAlmostEqual(self.messages["state"][-1].velocity[0], 1, places=5)
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = "uav2/lio_init", rospy.Time.now()
        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 2, 1, 16, 20
        cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(("x", "y", "z", "intensity"))]
        cloud.data = struct.pack("<ffff", 7, 9, 0, 42)+b"PAD!"+struct.pack("<ffff", 8, 8, 0, 43)+b"PAD!"
        self.cloud_pub.publish(cloud)
        self.pump(.1)
        out = self.messages["cloud"][-1]
        self.assertEqual(out.header.stamp, cloud.header.stamp)
        self.assertEqual(out.header.frame_id, "world")
        x, y, z, intensity = struct.unpack_from("<ffff", out.data)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 6)
        self.assertEqual(intensity, 42)
        self.assertEqual(out.data[16:20], b"PAD!")
        cmd = PositionTarget()
        cmd.header.frame_id, cmd.header.stamp = "world", rospy.Time.now()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = 0b100111000000
        cmd.position.x, cmd.position.y, cmd.position.z = 1, 5, 2
        cmd.velocity.x = 1
        self.cmd_pub.publish(cmd)
        self.pump(.1)
        out = self.messages["cmd"][-1]
        self.assertAlmostEqual(out.position.x, 2)
        self.assertAlmostEqual(out.position.y, -2)
        self.assertAlmostEqual(out.position.z, 3)
        self.assertAlmostEqual(out.velocity.x, 0, places=5)
        self.assertAlmostEqual(out.velocity.y, 1)
        self.assertAlmostEqual(out.yaw, math.pi/2, places=5)
        count = len(self.messages["cmd"])
        cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.cmd_pub.publish(cmd)
        self.pump(.1)
        self.assertEqual(len(self.messages["cmd"]), count)
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask |= PositionTarget.IGNORE_PX  # partial XY position cannot be yaw-rotated
        self.cmd_pub.publish(cmd)
        self.pump(.1)
        self.assertEqual(len(self.messages["cmd"]), count)

    def test_body_imu_offset_shifts_lio_anchor(self):
        # A measured body origin 1 m along the LIO X axis must move the LIO anchor with it.
        rospy.set_param("/test_adapter/body_in_imu/translation", [1.0, 0.0, 0.0])
        self.restart_adapter()
        self.pump()
        self.assertTrue(self.ready())
        out = self.publish_cloud([(7, 9, 0, 1)])
        x, y, _z, _i = struct.unpack_from("<ffff", out.data)
        self.assertAlmostEqual(x, -1, places=5)
        self.assertAlmostEqual(y, 6, places=5)

    def test_stale_data_latches_invalid(self):
        self.pump()
        self.assertTrue(self.ready())
        time.sleep(.5)
        self.assertFalse(self.ready())
        count = len(self.messages["odom"])
        self.pump()
        self.assertFalse(self.ready())
        self.assertEqual(len(self.messages["odom"]), count)

    def test_old_stamps_never_initialize(self):
        self.pump(stamp_offset=-2)
        self.assertFalse(self.ready())
        self.assertFalse(self.messages["odom"])

    def test_invalid_quaternion_and_tilt_block_initialization(self):
        self.fcu.pose.pose.orientation.w = 0
        self.fcu.pose.pose.orientation.z = 0
        self.pump()
        self.assertFalse(self.ready())
        self.fcu.pose.pose.orientation.x = math.sin(.15)
        self.fcu.pose.pose.orientation.w = math.cos(.15)
        self.pump()
        self.assertFalse(self.ready())

    def test_controller_requires_own_and_peer_readiness(self):
        calls = []
        arm = rospy.Service("/uav2/mavros/cmd/arming", CommandBool,
                            lambda req: (calls.append("arm"), CommandBoolResponse(success=True, result=0))[1])
        mode = rospy.Service("/uav2/mavros/set_mode", SetMode,
                             lambda req: (calls.append("mode"), SetModeResponse(mode_sent=True))[1])
        self.addCleanup(arm.shutdown)
        self.addCleanup(mode.shutdown)
        for name, value in {"uav_id": 2, "swarm_num": 2, "controller_flag": 1,
                            "require_frame_ready": True}.items():
            rospy.set_param("/test_controller/"+name, value)
        nodes = roslib.packages.find_node("prometheus_swarm_control", "swarm_controller")
        proc = subprocess.Popen([nodes[0], "__name:=test_controller",
                                 "/uav1/frame_ready:=/test/frame1", "/uav2/frame_ready:=/test/frame2",
                                 "/uav2/mavros/setpoint_raw/local:=/test/controller_setpoint"],
                                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(stop, proc)
        pub = rospy.Publisher("/uav2/prometheus/swarm_command", SwarmCommand, queue_size=10)
        own = rospy.Publisher("/test/frame2", Bool, queue_size=10)
        peer = rospy.Publisher("/test/frame1", Bool, queue_size=10)
        self.addCleanup(pub.unregister)
        self.addCleanup(own.unregister)
        self.addCleanup(peer.unregister)
        time.sleep(.4)
        command = SwarmCommand(Mode=SwarmCommand.Idle, yaw_ref=999)
        for i in range(20):
            own.publish(Bool(data=True))
            peer.publish(Bool(data=False))
            pub.publish(command)
            time.sleep(.02)
        self.assertEqual(calls, [], "must not arm or enter OFFBOARD before both frames are ready")
        for i in range(20):
            own.publish(Bool(data=True))
            peer.publish(Bool(data=True))
            pub.publish(command)
            time.sleep(.02)
        self.assertIn("arm", calls)
        self.assertIn("mode", calls)
        targets = []
        sub = rospy.Subscriber("/test/controller_setpoint", PositionTarget, targets.append, queue_size=100)
        self.addCleanup(sub.unregister)
        command = SwarmCommand(Mode=SwarmCommand.Move, Move_mode=SwarmCommand.XYZ_POS)
        command.position_ref = [1, 2, 3]
        for i in range(20):
            own.publish(Bool(data=True))
            peer.publish(Bool(data=True))
            pub.publish(command)
            time.sleep(.02)
        motion = [m for m in targets if m.type_mask != 0x4000]
        self.assertTrue(motion)
        self.assertEqual(motion[-1].header.frame_id, "world")
        self.assertGreater(motion[-1].header.stamp.to_sec(), 0)
        time.sleep(.7)
        count = len([m for m in targets if m.type_mask != 0x4000])
        time.sleep(.15)
        # Armed/invalid readiness may suppress motion setpoints only; the pre-arm idle heartbeat is allowed.
        self.assertEqual(len([m for m in targets if m.type_mask != 0x4000]), count,
                         "stale readiness must stop motion setpoints")

    def test_reference_jump_latches_invalid(self):
        self.pump()
        self.assertTrue(self.ready())
        self.fcu.pose.pose.position.x += 3
        self.pump(.1)
        self.assertFalse(self.ready())


if __name__ == "__main__":
    result = unittest.main(verbosity=2, exit=False).result
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if result.wasSuccessful() else 1)
