#!/usr/bin/env python3
"""Exercise the real bridge with an isolated ROS master; no FCU or motors."""
import os, signal, socket, subprocess, time, unittest, xmlrpc.client
from pathlib import Path
import rospy, roslib.packages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header, String
from mavros_msgs.msg import State, ExtendedState
from prometheus_msgs.msg import DroneState, SwarmCommand
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "artifacts/mission_handover_tests"
OUT.mkdir(parents=True, exist_ok=True)

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try: p.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL); p.wait()

class HandoverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as s: s.bind(("127.0.0.1",0)); port=s.getsockname()[1]
        os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d"%port, ROS_IP="127.0.0.1", ROS_HOME=str(OUT/"ros_home"))
        os.environ.pop("ROS_HOSTNAME",None)
        f=open(OUT/"master.log","w"); cls.addClassCleanup(f.close)
        cls.master=subprocess.Popen(["roscore","-p",str(port)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        cls.addClassCleanup(stop,cls.master)
        end=time.monotonic()+35
        while True:
            try:
                with xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]) as m: m.getPid("/handover_test")
                break
            except Exception:
                if time.monotonic()>end: raise
                time.sleep(.1)
        rospy.init_node("handover_test",anonymous=True,disable_signals=True)

    def setUp(self):
        self.prefix="/uav2"; self.starts=[]; self.stops=[]; self.commands=[]; self.status=[]
        self.subs=[]; self.pubs=[]
        for topic,typ,store in [("/planning/mission_start",PoseStamped,self.starts),("/planning/mission_stop",Header,self.stops),("/prometheus/swarm_command",SwarmCommand,self.commands),("/planning/mission_state",String,self.status)]:
            self.subs.append(rospy.Subscriber(self.prefix+topic,typ,store.append,queue_size=100))
        # A second ROS node models the independently connected trajectory server.
        consumer_code = "import rospy; from geometry_msgs.msg import PoseStamped; rospy.init_node('test_traj_consumer', anonymous=True); s=rospy.Subscriber('/uav2/planning/mission_start',PoseStamped,lambda m:None); rospy.spin()"
        consumer=subprocess.Popen(["python3","-c",consumer_code],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        self.addCleanup(stop,consumer)
        def pub(topic,typ,latch=False):
            p=rospy.Publisher(self.prefix+topic,typ,queue_size=10,latch=latch); self.pubs.append(p); return p
        self.pose_pub=pub("/world/odom",Odometry)
        self.aggregate_pub=pub("/prometheus/drone_state",DroneState)
        self.state_pub=pub("/mavros/state",State)
        self.land_pub=pub("/mavros/extended_state",ExtendedState)
        self.frame_pub=pub("/frame_ready",Bool)
        self.ready_pub=pub("/planning/mission_ready",Bool,True)
        self.traj_pub=pub("/planning/ego/traj_cmd",PositionCommand)
        self.spline_pub=pub("/planning/bspline",Bspline)
        self.station_pub=pub("/prometheus/swarm_command",SwarmCommand)
        self.pose=DroneState(connected=True,armed=True,mode="POSCTL")
        self.pose.header.frame_id="world"; self.pose.position=[10.,3.,1.2];self.pose.attitude=[0.,0.,.7]
        self.pose.attitude_q.w=1.
        self.mode="MANUAL" if self._testMethodName in ("test_automatic_takeoff_is_not_overwritten_on_ground","test_cancelled_takeoff_permission_is_not_reused") else "POSCTL";self.armed=True;self.air=True;self.ready=True;self.connected=True
        name="test_mission_bridge"
        for k,v in {"uav_id":2,"enable_mission_gate":True,"require_frame_ready":True,"handover/settle_time":.2,"handover/feedback_timeout":.25,"handover/trajectory_timeout":.25}.items():rospy.set_param("/"+name+"/"+k,v)
        exe=roslib.packages.find_node("prometheus_swarm_control","ego_traj_to_cmd")[0]
        f=open(OUT/(self._testMethodName+".log"),"w");self.addCleanup(f.close)
        node=subprocess.Popen([exe,"__name:="+name],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);self.addCleanup(stop,node)
        self.addCleanup(self.cleanup)
        deadline=time.monotonic()+5
        while self.pose_pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.02)
        self.assertGreater(self.pose_pub.get_num_connections(),0,"bridge must consume feedback before forwarding EGO")
        self.pump(.35)

    def cleanup(self):
        for x in self.subs+self.pubs:x.unregister()
        time.sleep(.1)

    def pump(self,seconds,pose=True,cmd=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.state_pub.publish(State(connected=self.connected,armed=self.armed,mode=self.mode))
            self.land_pub.publish(ExtendedState(landed_state=2 if self.air else 1))
            self.frame_pub.publish(Bool(self.ready));self.ready_pub.publish(Bool(True))
            if pose:
                import math
                o=Odometry();o.header.stamp=rospy.Time.now();o.header.frame_id="world"
                o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=self.pose.position
                o.pose.pose.orientation.z=math.sin(self.pose.attitude[2]/2);o.pose.pose.orientation.w=math.cos(self.pose.attitude[2]/2)
                o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=self.pose.velocity
                self.pose_pub.publish(o)
            # Keep the aggregate live even when the actual odometry source is frozen.
            self.pose.header.stamp=rospy.Time.now();self.aggregate_pub.publish(self.pose)
            if cmd is not None:cmd.header.stamp=rospy.Time.now();self.traj_pub.publish(cmd)
            time.sleep(.02)

    def fresh_command(self,traj_id=12):
        b=Bspline(traj_id=traj_id,start_time=rospy.Time.now(),order=3)
        self.spline_pub.publish(b);time.sleep(.04)
        c=PositionCommand(trajectory_id=traj_id,trajectory_flag=1,yaw=1.1)
        c.header.frame_id="world";c.position.x=self.pose.position[0]+.02;c.position.y=3.;c.position.z=1.2
        return c

    def test_hold_one_start_and_reentry_rejects_old_trajectory(self):
        self.assertTrue(self.commands,"valid hold setpoints must be prestreamed before OFFBOARD")
        self.assertEqual(len(self.starts),0)
        self.assertAlmostEqual(self.commands[-1].yaw_ref,.7,places=3)
        self.mode="OFFBOARD";self.pump(.45)
        self.assertEqual(len(self.starts),1)
        old=PositionCommand(trajectory_id=9,yaw=-2.);old.position.x=99.
        self.pump(.15,cmd=old)
        self.assertLess(self.commands[-1].position_ref[0],11.)
        c=self.fresh_command();self.pump(.15,cmd=c)
        self.assertAlmostEqual(self.commands[-1].yaw_ref,1.1,places=3,msg="mission yaw must follow EGO")
        self.assertIn("running",[m.data for m in self.status])
        delayed=Bspline(traj_id=99,start_time=self.starts[-1].header.stamp+rospy.Duration(.001),order=3)
        self.spline_pub.publish(delayed);self.pump(.35,cmd=c)
        self.assertEqual(self.status[-1].data,"running","delayed older spline must not replace the active trajectory ID")
        self.pose.position[0]=10.5;self.mode="POSCTL";self.pump(.15,cmd=c)
        self.assertGreater(len(self.stops),0)
        self.mode="OFFBOARD";self.pump(.4,cmd=c)
        self.assertEqual(len(self.starts),2)
        self.assertAlmostEqual(self.commands[-1].position_ref[0],10.5,places=2)
        self.pump(.2,cmd=c);self.assertEqual(len(self.starts),2)

    def test_automatic_takeoff_is_not_overwritten_on_ground(self):
        self.mode="MANUAL";self.pose.position[2]=0.;self.air=False;self.pump(.1)
        self.station_pub.publish(SwarmCommand(Mode=SwarmCommand.Takeoff,source="simulation_station"));self.pump(.1)
        self.commands.clear();self.mode="OFFBOARD";self.pump(.35)
        self.assertEqual(len(self.starts),0)
        self.assertFalse([m for m in self.commands if m.source!="simulation_station"])
        self.air=True;self.pose.position[2]=1.2;self.pump(.65)
        self.assertEqual(len(self.starts),1)
        self.pump(.4)
        self.assertEqual(self.status[-1].data,"planning", "native takeoff must continue through the holding state")

    def test_cancelled_takeoff_permission_is_not_reused(self):
        self.pose.position[2]=0.;self.air=False;self.pump(.1)
        self.station_pub.publish(SwarmCommand(Mode=SwarmCommand.Takeoff,source="simulation_station"));self.pump(.1)
        self.mode="OFFBOARD";self.pump(.15)
        self.mode="MANUAL";self.pump(.1)
        self.mode="OFFBOARD";self.air=True;self.pose.position[2]=1.2;self.pump(.5)
        self.assertEqual(len(self.starts),0,"cancelled takeoff must not authorize a later OFFBOARD entry")
        self.mode="POSCTL";self.pump(.1);self.mode="OFFBOARD";self.pump(.4)
        self.assertEqual(len(self.starts),1)

    def test_disconnect_is_not_a_mode_exit(self):
        self.mode="OFFBOARD";self.pump(.4);self.assertEqual(len(self.starts),1)
        self.connected=False;self.pump(.15)
        self.connected=True;self.pump(.5)
        self.assertEqual(len(self.starts),1,"reconnection in OFFBOARD must not restart a mission")
        self.assertIn("blocked",[m.data for m in self.status])
        self.mode="POSCTL";self.pump(.1);self.mode="OFFBOARD";self.pump(.4)
        self.assertEqual(len(self.starts),2)

    def test_trajectory_server_clears_sessions_and_retains_path_facing_yaw(self):
        import math
        messages=[]
        sub=rospy.Subscriber("/test_position_cmd",PositionCommand,messages.append,queue_size=100);self.subs.append(sub)
        start=rospy.Publisher(self.prefix+"/planning/mission_start",PoseStamped,queue_size=1)
        stop_pub=rospy.Publisher(self.prefix+"/planning/mission_stop",Header,queue_size=1)
        self.pubs.extend([start,stop_pub])
        rospy.set_param("/test_traj_server/mission_gate",True);rospy.set_param("/test_traj_server/uav_id",2)
        rospy.set_param("/test_traj_server/traj_server/time_forward",1.)
        exe=roslib.packages.find_node("ego_planner","traj_server")[0]
        f=open(OUT/"trajectory_server.log","w");self.addCleanup(f.close)
        node=subprocess.Popen([exe,"__name:=test_traj_server","/test_traj_server/planning/bspline:="+self.prefix+"/planning/bspline","/position_cmd:=/test_position_cmd"],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);self.addCleanup(stop,node)
        time.sleep(1.3)
        def spline(ident):
            from geometry_msgs.msg import Point
            b=Bspline(traj_id=ident,order=3,start_time=rospy.Time.now(),knots=[-.3,-.2,-.1,0.,.1,.2,.3,.4])
            b.pos_pts=[Point(10.+(i-1)*.3,3.,1.2) for i in range(4)]
            return b
        old=spline(2);self.spline_pub.publish(old);time.sleep(.15)
        self.assertFalse(messages,"trajectory server must ignore splines before permission")
        begin=PoseStamped();begin.header.stamp=rospy.Time.now();begin.header.frame_id="world"
        begin.pose.orientation.z=math.sin(.7/2);begin.pose.orientation.w=math.cos(.7/2)
        start.publish(begin);time.sleep(.05)
        self.spline_pub.publish(old);time.sleep(.1);self.assertFalse(messages)
        # Vertical motion and tiny backward XY corrections must not define a new heading.
        for dx,dy in [(0.,0.),(-.01,.005)]:
            from geometry_msgs.msg import Point
            vertical=spline(3)
            vertical.pos_pts=[Point(10.+(i-1)*dx,3.+(i-1)*dy,1.2+(i-1)*.3) for i in range(4)]
            messages.clear();self.spline_pub.publish(vertical);time.sleep(.25)
            self.assertTrue(messages)
            self.assertLess(max(abs(m.yaw-.7) for m in messages),.01,"vertical/tiny XY motion caused a heading change")
        messages.clear()
        fresh=spline(4);self.spline_pub.publish(fresh);time.sleep(.25)
        self.assertTrue(messages);self.assertLess(abs(messages[0].yaw-.7),.15)
        self.assertLess(messages[-1].yaw,.69,"EGO must still turn toward the forward path")
        cancel=Header(stamp=rospy.Time.now(),frame_id="world");stop_pub.publish(cancel);time.sleep(.1)
        n=len(messages);self.spline_pub.publish(spline(4));time.sleep(.15);self.assertEqual(len(messages),n)
        # Delayed starts from the cancelled session must not revive its spline.
        start.publish(begin);self.spline_pub.publish(spline(5));time.sleep(.15);self.assertEqual(len(messages),n)
        stop_pub.publish(Header(stamp=cancel.stamp-rospy.Duration(.04),frame_id="world"));time.sleep(.05)
        begin.header.stamp=cancel.stamp-rospy.Duration(.02)
        start.publish(begin);time.sleep(.05);self.spline_pub.publish(spline(6));time.sleep(.15)
        self.assertEqual(len(messages),n,"delayed stop must not move the cancellation watermark backwards")

    def test_external_land_remains_owner_when_feedback_stops(self):
        self.mode="OFFBOARD";self.pump(.4)
        self.station_pub.publish(SwarmCommand(Mode=SwarmCommand.Land,source="operator"));self.pump(.1)
        self.commands.clear();self.pump(.6,pose=False)
        self.assertFalse([m for m in self.commands if m.source!="operator"],"stale feedback must not override operator Land")

    def test_stale_feedback_blocks_restart_until_mode_exit(self):
        self.mode="OFFBOARD";self.pump(.4);self.assertEqual(len(self.starts),1)
        c=self.fresh_command();c.velocity.x=.4;self.pump(.1,cmd=c)
        self.pump(.5,pose=False)
        self.assertEqual(self.commands[-1].Move_mode,SwarmCommand.XYZ_POS)
        self.assertEqual(list(self.commands[-1].velocity_ref),[0.,0.,0.])
        self.pump(.4)
        self.assertEqual(len(self.starts),1,"fresh feedback alone must not resume an aborted session")
        self.assertIn("blocked",[m.data for m in self.status])
        self.mode="POSCTL";self.pump(.1);self.mode="OFFBOARD";self.pump(.4)
        self.assertEqual(len(self.starts),2)

if __name__=="__main__":unittest.main()
