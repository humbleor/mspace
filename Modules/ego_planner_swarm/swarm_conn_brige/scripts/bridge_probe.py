#!/usr/bin/env python3
"""Finite communication fixtures only. Run on a ground-test master with no controller/planner."""
import argparse
import json
import math
import time
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from traj_utils.msg import Bspline, MultiBsplines

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uav-id",type=int,choices=(1,2),required=True)
    p.add_argument("--duration",type=float,default=15)
    p.add_argument("--rate",type=float,default=5)
    args=p.parse_args()
    if not (0<args.duration<=300 and 0<args.rate<=20):
        p.error("duration must be 0..300 seconds; rate must be 0..20 Hz")
    rospy.init_node("bridge_probe_uav%d"%args.uav_id,anonymous=True)
    uid=args.uav_id; ns="/uav%d/bridge/"%uid
    stats={"uav_id":uid,"tx_udp":0,"tx_tcp":0,"rx_udp":0,"rx_tcp":0,"self_rx":0,"tcp_connected":False}
    ages=[]
    def receive(m):
        stats["rx_udp"]+=1
        stats["self_rx"]+=int(m.drone_id==uid)
        ages.append((rospy.Time.now()-m.start_time).to_sec()*1000)
    def receive_multi(m): stats["rx_tcp"]+=1
    def state(m): stats["tcp_connected"]=m.data
    subs=[rospy.Subscriber(ns+"rx/bspline",Bspline,receive,queue_size=100),
          rospy.Subscriber(ns+"rx/prev_multi_traj",MultiBsplines,receive_multi,queue_size=100),
          rospy.Subscriber("/uav%d/bridge_connection_state"%uid,Bool,state)]
    tx=rospy.Publisher(ns+"tx/bspline",Bspline,queue_size=10)
    mt=rospy.Publisher(ns+"tx/multi_traj",MultiBsplines,queue_size=10)
    # Bounded wall-clock run, including if /clock accidentally stops.
    start=time.monotonic(); sequence=0
    while time.monotonic()-start<args.duration and not rospy.is_shutdown():
        sequence+=1
        m=Bspline(drone_id=uid,order=3,traj_id=sequence,start_time=rospy.Time.now())
        m.knots=[float(i) for i in range(8)]
        m.pos_pts=[Point(float(i),float(uid),2.0) for i in range(4)]
        m.yaw_dt=.1
        tx.publish(m); stats["tx_udp"]+=1
        if uid==1:
            mt.publish(MultiBsplines(drone_id_from=1,traj=[Bspline(),m]))
            stats["tx_tcp"]+=1
        time.sleep(1/args.rate)
    time.sleep(.3)
    if ages:
        ages.sort()
        stats["observed_age_ms_p95"]=ages[min(len(ages)-1,math.ceil(.95*len(ages))-1)]
    stats["note"]="Fixture counts, not flight commands. Cross-host ages require synchronized clocks."
    print(json.dumps(stats,indent=2))
    return 0 if stats["rx_udp"] and not stats["self_rx"] and (uid==1 or stats["rx_tcp"]) else 1

if __name__=="__main__":
    raise SystemExit(main())
