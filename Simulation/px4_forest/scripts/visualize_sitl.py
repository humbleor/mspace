#!/usr/bin/env python3
"""Read-only ROS visualization feeds, isolated from sensor and flight loops."""
import argparse,json,time,copy,threading
from pathlib import Path
import numpy as np
import rospy,tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TransformStamped,Point
from visualization_msgs.msg import Marker,MarkerArray
from mavros_msgs.msg import State

def make_cloud(points):
    xyz=np.asarray(points,dtype="<f4")
    return PointCloud2(header=Header(frame_id="world"),height=1,width=len(xyz),
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(("x","y","z"))],
        is_bigendian=False,point_step=12,row_step=12*len(xyz),data=xyz.tobytes(),is_dense=True)

def local_pose(odom,initial):
    p=PoseStamped(header=copy.deepcopy(odom.header),pose=copy.deepcopy(odom.pose.pose))
    p.header.frame_id="world"
    p.pose.position.x-=initial[0];p.pose.position.y-=initial[1];p.pose.position.z-=initial[2]
    return p

def main():
    ap=argparse.ArgumentParser();ap.add_argument("output",type=Path);a=ap.parse_args()
    meta=json.loads((a.output/"visual_scene.json").read_text())
    rospy.init_node("forest_visualization",disable_signals=False)
    origin=np.asarray(meta["initial"]);goals=np.asarray(meta["goals"])
    tf=TransformStamped();tf.header.frame_id="world";tf.child_frame_id="gazebo_world"
    tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=-origin
    tf.transform.rotation.w=1
    broadcaster=tf2_ros.StaticTransformBroadcaster();broadcaster.sendTransform(tf)
    forest_pub=rospy.Publisher("/sim/view/forest",PointCloud2,queue_size=1,latch=True)
    if (a.output/"visual_map.npy").exists():
        forest_pub.publish(make_cloud(np.load(a.output/"visual_map.npy")))
    goals_pub=rospy.Publisher("/sim/view/goals",MarkerArray,queue_size=1,latch=True)
    markers=MarkerArray()
    for i,pos in enumerate(goals):
        m=Marker(header=Header(frame_id="world"),ns="route_goals",id=i,type=Marker.SPHERE,action=Marker.ADD)
        m.pose.position.x,m.pose.position.y,m.pose.position.z=pos;m.pose.orientation.w=1
        m.scale.x=m.scale.y=m.scale.z=.22;m.color.r=1;m.color.g=.65;m.color.a=1
        markers.markers.append(m)
        t=copy.deepcopy(m);t.ns="goal_labels";t.type=Marker.TEXT_VIEW_FACING;t.text="Takeoff" if i==0 else str(i)
        t.scale.z=.28;t.pose.position.z+=.35;markers.markers.append(t)
    goals_pub.publish(markers)
    latest={};lock=threading.Lock();state=None
    def odom_cb(m,key):
        with lock:latest[key]=local_pose(m,origin if key=="truth" else np.zeros(3))
    def state_cb(m):
        nonlocal state
        state=m
    rospy.Subscriber("/sim/plant/odom",Odometry,odom_cb,callback_args="truth",queue_size=1)
    rospy.Subscriber(meta.get("lio_topic","/uav1/drone_Odom_high_freq"),Odometry,odom_cb,callback_args="lio",queue_size=1)
    rospy.Subscriber(meta.get("fcu_topic","/uav1/mavros/local_position/odom"),Odometry,odom_cb,callback_args="fcu",queue_size=1)
    rospy.Subscriber(meta.get("state_topic","/uav1/mavros/state"),State,state_cb,queue_size=1)
    paths={k:RosPath(header=Header(frame_id="world")) for k in ("truth","lio","fcu")}
    pubs={k:rospy.Publisher("/sim/view/"+k+"_path",RosPath,queue_size=1,latch=True) for k in paths}
    vehicle=rospy.Publisher("/sim/view/vehicle",MarkerArray,queue_size=1,latch=True)
    last={k:-1 for k in paths}
    rospy.loginfo("Visualization ready: world frame, %d goals, observer-only feeds",len(goals))
    while not rospy.is_shutdown():
        with lock:poses=latest.copy()
        for k,p in poses.items():
            stamp=p.header.stamp.to_sec()
            if stamp-last[k]<.1:continue
            last[k]=stamp;paths[k].header.stamp=p.header.stamp;paths[k].poses.append(p)
            paths[k].poses=paths[k].poses[-4000:];pubs[k].publish(paths[k])
        if "truth" in poses:
            pose=poses["truth"]
            m=Marker(header=pose.header,ns="physical_drone",id=0,type=Marker.ARROW,action=Marker.ADD,pose=pose.pose)
            m.scale.x=.65;m.scale.y=.16;m.scale.z=.16;m.color.b=1;m.color.g=.65;m.color.a=1
            t=Marker(header=pose.header,ns="flight_status",id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
            t.pose=copy.deepcopy(pose.pose);t.pose.position.z+=.6;t.pose.orientation.x=t.pose.orientation.y=t.pose.orientation.z=0;t.pose.orientation.w=1
            t.scale.z=.25;t.color.a=1;t.color.g=1
            t.text=(state.mode+" / "+("ARMED" if state.armed else "DISARMED")) if state else "Connecting"
            vehicle.publish(MarkerArray(markers=[m,t]))
        time.sleep(.2)
if __name__=="__main__":main()
