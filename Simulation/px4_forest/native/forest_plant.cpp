#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_msgs/ContactsState.h>

namespace gazebo {
class ForestPlant: public ModelPlugin {
  physics::LinkPtr link;
  physics::WorldPtr world;
  event::ConnectionPtr update;
  event::ConnectionPtr contact_update;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Publisher imu,truth,contacts;
  double last=-1;
public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    if(!ros::isInitialized()){int argc=0;char **argv=nullptr;ros::init(argc,argv,"forest_plant",ros::init_options::NoSigintHandler);}
    nh.reset(new ros::NodeHandle());
    link=model->GetLink("base_link");world=model->GetWorld();
    if(!link)gzthrow("Iris base_link missing");
    imu=nh->advertise<sensor_msgs::Imu>("/sim/mid360/imu",1000);
    truth=nh->advertise<nav_msgs::Odometry>("/sim/plant/odom",1000);
    contacts=nh->advertise<gazebo_msgs::ContactsState>("/sim/plant/tree_contacts",1000);
    world->Physics()->GetContactManager()->SetNeverDropContacts(true);
    update=event::Events::ConnectWorldUpdateBegin(std::bind(&ForestPlant::Tick,this));
    contact_update=event::Events::ConnectWorldUpdateEnd(std::bind(&ForestPlant::Contacts,this));
  }
  void Contacts() {
    gazebo_msgs::ContactsState message;
    message.header.stamp.fromSec(world->SimTime().Double());
    message.header.frame_id="gazebo_world";
    auto manager=world->Physics()->GetContactManager();
    for (unsigned i=0;i<manager->GetContactCount();++i) {
      auto c=manager->GetContact(i);
      if (!c || !c->collision1 || !c->collision2) continue;
      auto n1=c->collision1->GetScopedName(), n2=c->collision2->GetScopedName();
      if (n1.find("forest_collision_geometry::")==std::string::npos &&
          n2.find("forest_collision_geometry::")==std::string::npos) continue;
      gazebo_msgs::ContactState state;
      state.collision1_name=n1; state.collision2_name=n2;
      for (int j=0;j<c->count;++j) {
        geometry_msgs::Vector3 position,normal;
        position.x=c->positions[j].X();position.y=c->positions[j].Y();position.z=c->positions[j].Z();
        normal.x=c->normals[j].X();normal.y=c->normals[j].Y();normal.z=c->normals[j].Z();
        state.contact_positions.push_back(position);state.contact_normals.push_back(normal);
        state.depths.push_back(c->depths[j]);
        geometry_msgs::Wrench wrench;
        bool first=n1.find("forest_collision_geometry::")==std::string::npos;
        auto force=first?c->wrench[j].body1Force:c->wrench[j].body2Force;
        auto torque=first?c->wrench[j].body1Torque:c->wrench[j].body2Torque;
        wrench.force.x=force.X();wrench.force.y=force.Y();wrench.force.z=force.Z();
        wrench.torque.x=torque.X();wrench.torque.y=torque.Y();wrench.torque.z=torque.Z();
        state.wrenches.push_back(wrench);
        state.total_wrench.force.x+=force.X();state.total_wrench.force.y+=force.Y();state.total_wrench.force.z+=force.Z();
        state.total_wrench.torque.x+=torque.X();state.total_wrench.torque.y+=torque.Y();state.total_wrench.torque.z+=torque.Z();
      }
      if (c->count>0)message.states.push_back(state);
    }
    // Empty frames prove that the rebuilt observer is active; silence is not
    // treated as a successful no-contact result.
    contacts.publish(message);
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
