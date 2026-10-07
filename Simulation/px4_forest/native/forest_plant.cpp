#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace gazebo {
class ForestPlant: public ModelPlugin {
  physics::LinkPtr link;
  physics::WorldPtr world;
  event::ConnectionPtr update;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Publisher imu,truth;
  double last=-1;
public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    if(!ros::isInitialized()){int argc=0;char **argv=nullptr;ros::init(argc,argv,"forest_plant",ros::init_options::NoSigintHandler);}
    nh.reset(new ros::NodeHandle());
    link=model->GetLink("base_link");world=model->GetWorld();
    if(!link)gzthrow("Iris base_link missing");
    imu=nh->advertise<sensor_msgs::Imu>("/sim/mid360/imu",1000);
    truth=nh->advertise<nav_msgs::Odometry>("/sim/plant/odom",1000);
    update=event::Events::ConnectWorldUpdateBegin(std::bind(&ForestPlant::Tick,this));
  }
  void Tick() {
    double t=world->SimTime().Double();
    if(t<=0 || (last>=0 && t-last<.0039))return;
    last=t;
    auto pose=link->WorldPose();
    auto v=link->WorldLinearVel();
    auto a=pose.Rot().RotateVectorReverse(link->WorldLinearAccel()-world->Gravity())/9.81;
    auto w=link->RelativeAngularVel();
    sensor_msgs::Imu m;m.header.stamp.fromSec(t);m.header.frame_id="livox_frame";m.orientation_covariance[0]=-1;
    m.angular_velocity.x=w.X();m.angular_velocity.y=w.Y();m.angular_velocity.z=w.Z();
    m.linear_acceleration.x=a.X();m.linear_acceleration.y=a.Y();m.linear_acceleration.z=a.Z();
    imu.publish(m);
    nav_msgs::Odometry o;o.header=m.header;o.header.frame_id="gazebo_world";o.child_frame_id="base_link";
    o.pose.pose.position.x=pose.Pos().X();o.pose.pose.position.y=pose.Pos().Y();o.pose.pose.position.z=pose.Pos().Z();
    o.pose.pose.orientation.x=pose.Rot().X();o.pose.pose.orientation.y=pose.Rot().Y();o.pose.pose.orientation.z=pose.Rot().Z();o.pose.pose.orientation.w=pose.Rot().W();
    o.twist.twist.linear.x=v.X();o.twist.twist.linear.y=v.Y();o.twist.twist.linear.z=v.Z();
    o.twist.twist.angular=m.angular_velocity;
    truth.publish(o);
  }
};
GZ_REGISTER_MODEL_PLUGIN(ForestPlant)
}
