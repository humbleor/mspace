#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <traj_utils/Bspline.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <cmath>
#include <string>

// One publisher owns both handover hold and trajectory forwarding. No mode/arm services.
class EgoTrajBridge
{
  ros::NodeHandle nh_{"~"};
  ros::Publisher command_pub_, visual_pub_, start_pub_, stop_pub_, status_pub_;
  ros::Subscriber trajectory_sub_, pose_sub_, state_sub_, land_sub_, frame_sub_, ready_sub_, spline_sub_, station_sub_;
  ros::Timer timer_;
  std::string prefix_, source_, phase_ = "waiting";
  int control_flag_ = 0;
  bool gated_ = false, require_frame_ = false, frame_ = false, planner_ready_ = false;
  bool seen_position_ = false, offboard_ = false, external_takeoff_ = false;
  bool have_pose_ = false, have_hold_ = false, have_command_ = false, have_last_good_ = false;
  double settle_time_ = .75, feedback_timeout_ = .5, command_timeout_ = .5;
  double speed_limit_ = .2, hold_tolerance_ = .15, initial_error_ = .35, planning_timeout_ = 10.;
  double takeoff_height_ = 1.2, takeoff_z_ = 0.;
  ros::WallTime pose_recv_, frame_recv_;
  ros::Time settle_since_, session_, spline_stamp_, command_stamp_, state_time_, land_time_;
  int64_t spline_id_ = -1;
  prometheus_msgs::DroneState pose_, hold_, last_good_;
  mavros_msgs::State state_;
  mavros_msgs::ExtendedState land_;
  quadrotor_msgs::PositionCommand command_;
  uint32_t command_id_ = 0;

  static double norm3(const float *a) { return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }
  bool recent(const ros::WallTime &t, double timeout) const
  { return !t.isZero() && (ros::WallTime::now()-t).toSec() <= timeout; }
  bool freshStamp(const ros::Time &t, double timeout) const
  { const double age=(ros::Time::now()-t).toSec(); return !t.isZero() && age>=-.1 && age<=timeout; }
  bool feedbackValid() const
  {
    if (!have_pose_ || !recent(pose_recv_,feedback_timeout_) || !freshStamp(pose_.header.stamp,feedback_timeout_) ||
        !freshStamp(state_time_,2.5) || !state_.connected || !freshStamp(land_time_,1.5) ||
        (require_frame_ && (!frame_ || !recent(frame_recv_,feedback_timeout_)))) return false;
    if (pose_.header.frame_id!="world") return false;
    for (int i=0;i<3;++i)
      if (!std::isfinite(pose_.position[i]) || !std::isfinite(pose_.velocity[i]) ||
          !std::isfinite(pose_.attitude[i]) || !std::isfinite(pose_.attitude_rate[i])) return false;
    return true;
  }
  bool airborne() const
  { return land_.landed_state==mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR && pose_.position[2]>.5; }
  bool stable() const
  {
    double d=0.; for(int i=0;i<3;++i) d+=std::pow(pose_.position[i]-hold_.position[i],2);
    return norm3(pose_.velocity.data())<speed_limit_ && std::sqrt(d)<hold_tolerance_ &&
      std::abs(pose_.attitude_rate[2])<.15;
  }
  void setPhase(const std::string &phase)
  {
    if (phase_==phase) return;
    phase_=phase; std_msgs::String m; m.data=phase; status_pub_.publish(m);
    ROS_INFO_STREAM("Mission handover: " << phase);
  }
  void stopMission()
  {
    std_msgs::Header stop; stop.stamp=ros::Time::now(); stop.frame_id="world"; stop_pub_.publish(stop);
    have_command_=false; spline_id_=-1; session_=ros::Time(); spline_stamp_=ros::Time();
  }
  void captureHold() { hold_=pose_; have_hold_=true; }
  void abortMission(bool neutralize=true)
  {
    stopMission();
    have_hold_=false;
    if (neutralize) {
      if (feedbackValid()) captureHold();
      else if (have_last_good_) { hold_=last_good_; have_hold_=true; }
      // Replace the downstream controller's cached trajectory, including its feedforward.
      publishHold();
    }
    settle_since_=ros::Time(); setPhase("blocked");
  }
  void publishCommand(const quadrotor_msgs::PositionCommand &c, bool hold=false)
  {
    prometheus_msgs::SwarmCommand m;
    m.header.stamp=ros::Time::now(); m.header.frame_id="world";
    m.Command_ID=++command_id_; m.source=source_; m.Mode=prometheus_msgs::SwarmCommand::Move;
    m.Move_mode=hold || control_flag_==1 ? prometheus_msgs::SwarmCommand::XYZ_POS :
      control_flag_==2 ? prometheus_msgs::SwarmCommand::XY_VEL_Z_POS : prometheus_msgs::SwarmCommand::TRAJECTORY;
    const double p[3]={c.position.x,c.position.y,c.position.z}, v[3]={c.velocity.x,c.velocity.y,c.velocity.z},
      a[3]={c.acceleration.x,c.acceleration.y,c.acceleration.z};
    for(int i=0;i<3;++i) { m.position_ref[i]=p[i]; m.velocity_ref[i]=v[i]; m.acceleration_ref[i]=a[i]; }
    m.yaw_ref=std::atan2(std::sin(c.yaw),std::cos(c.yaw));
    command_pub_.publish(m);
    geometry_msgs::PoseStamped visual; visual.header=m.header; visual.pose.position=c.position;
    visual.pose.orientation.z=std::sin(c.yaw/2); visual.pose.orientation.w=std::cos(c.yaw/2);
    visual_pub_.publish(visual);
  }
  void publishHold()
  {
    if (!have_hold_) return;
    quadrotor_msgs::PositionCommand c;
    c.position.x=hold_.position[0]; c.position.y=hold_.position[1]; c.position.z=hold_.position[2];
    c.yaw=hold_.attitude[2]; publishCommand(c,true);
  }
  void trajectory(const quadrotor_msgs::PositionCommand::ConstPtr &m)
  {
    const double values[]={m->position.x,m->position.y,m->position.z,m->velocity.x,m->velocity.y,m->velocity.z,
      m->acceleration.x,m->acceleration.y,m->acceleration.z,m->yaw};
    for(double x:values) if(!std::isfinite(x)) return;
    if (!gated_) { publishCommand(*m); return; }
    if ((phase_!="planning" && phase_!="running") || !feedbackValid() || !state_.armed ||
        state_.mode!="OFFBOARD" || m->trajectory_id!=spline_id_ || spline_id_<0 ||
        m->trajectory_flag!=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY ||
        m->header.frame_id!="world" || m->header.stamp<session_ || !freshStamp(m->header.stamp,command_timeout_)) return;
    if (phase_=="planning") {
      const double dx=m->position.x-pose_.position[0],dy=m->position.y-pose_.position[1],dz=m->position.z-pose_.position[2];
      if(std::sqrt(dx*dx+dy*dy+dz*dz)>initial_error_) {
        ROS_ERROR("First EGO reference is too far from current feedback; holding until a new handover");
        abortMission(); return;
      }
      setPhase("running");
    }
    command_=*m; command_stamp_=m->header.stamp; have_command_=true;
  }
  void spline(const traj_utils::Bspline::ConstPtr &m)
  {
    if ((phase_=="planning" || phase_=="running") && !session_.isZero() && m->start_time>=session_ &&
        m->start_time>=spline_stamp_ && (m->start_time-ros::Time::now()).toSec()<=.1) {
      spline_id_=m->traj_id; spline_stamp_=m->start_time;
    }
  }
  void station(const prometheus_msgs::SwarmCommand::ConstPtr &m)
  {
    if(m->source==source_) return;
    if(m->Mode==prometheus_msgs::SwarmCommand::Takeoff && phase_=="waiting") {
      external_takeoff_=true; takeoff_z_=have_pose_?pose_.position[2]:0.;
    }
    if(m->Mode==prometheus_msgs::SwarmCommand::Land || m->Mode==prometheus_msgs::SwarmCommand::Disarm ||
       m->Mode==prometheus_msgs::SwarmCommand::Hold) {
      abortMission(false); // The explicit station command keeps control priority.
    }
  }
  void tick(const ros::TimerEvent &)
  {
    const bool state_valid=freshStamp(state_time_,2.5) && state_.connected;
    if (!state_valid) {
      if (offboard_ && phase_!="blocked") abortMission();
      return; // Uncertain mode is not a confirmed operator exit/re-entry.
    }
    const bool mode=state_.armed && state_.mode=="OFFBOARD";
    if(!mode) {
      if(offboard_) { stopMission(); external_takeoff_=false; have_hold_=false; settle_since_=ros::Time(); seen_position_=false; setPhase("waiting"); }
      offboard_=false;
      if(state_.mode=="POSCTL" && freshStamp(state_time_,2.5)) seen_position_=true;
      if(phase_=="blocked") return;
      if(feedbackValid() && state_.armed && state_.mode=="POSCTL" && airborne() && !external_takeoff_) {
        captureHold(); publishHold();
      }
      return;
    }
    offboard_=true;
    if (feedbackValid()) { last_good_=pose_; have_last_good_=true; }
    if(phase_=="waiting" && !seen_position_ && !external_takeoff_) return; // Do not auto-start on an in-air node restart.
    if(!feedbackValid()) {
      if(phase_!="waiting" && phase_!="blocked") abortMission();
      return;
    }
    if(phase_=="blocked") { if(have_hold_) publishHold(); return; }
    if(phase_=="waiting") {
      if(!airborne()) return;
      // The same bridge also serves SITL's existing native OFFBOARD takeoff.
      if(external_takeoff_ && (pose_.position[2]<takeoff_z_+takeoff_height_-.15 || norm3(pose_.velocity.data())>=speed_limit_)) return;
      external_takeoff_=false; captureHold(); settle_since_=ros::Time(); setPhase("holding");
    }
    if(phase_=="holding") {
      publishHold();
      if(!stable() || !planner_ready_ || start_pub_.getNumSubscribers()<2) {
        double error=0.; for(int i=0;i<3;++i) error+=std::pow(pose_.position[i]-hold_.position[i],2);
        ROS_INFO_THROTTLE(3., "Handover waiting: speed=%.3f hold_error=%.3f yaw_rate=%.3f planner_ready=%d start_receivers=%u",
          norm3(pose_.velocity.data()),std::sqrt(error),pose_.attitude_rate[2],int(planner_ready_),start_pub_.getNumSubscribers());
        settle_since_=ros::Time(); return;
      }
      if(settle_since_.isZero()) settle_since_=ros::Time::now();
      if((ros::Time::now()-settle_since_).toSec()<settle_time_) return;
      geometry_msgs::PoseStamped start; start.header.stamp=ros::Time::now(); start.header.frame_id="world";
      start.pose.position.x=pose_.position[0]; start.pose.position.y=pose_.position[1]; start.pose.position.z=pose_.position[2];
      start.pose.orientation.z=std::sin(pose_.attitude[2]/2); start.pose.orientation.w=std::cos(pose_.attitude[2]/2);
      session_=start.header.stamp; spline_id_=-1; have_command_=false;
      setPhase("planning"); start_pub_.publish(start);
    } else if(phase_=="planning") {
      publishHold();
      if((ros::Time::now()-session_).toSec()>planning_timeout_) abortMission();
    } else if(phase_=="running") {
      if(!have_command_ || !freshStamp(command_stamp_,command_timeout_)) { abortMission(); publishHold(); }
      else publishCommand(command_);
    }
  }
public:
  EgoTrajBridge()
  {
    int id; nh_.param("uav_id",id,1); nh_.param("control_flag",control_flag_,0);
    nh_.param("enable_mission_gate",gated_,false); nh_.param("require_frame_ready",require_frame_,false);
    prefix_="/uav"+std::to_string(id); source_=prefix_+"/ego_traj_to_cmd";
    command_pub_=nh_.advertise<prometheus_msgs::SwarmCommand>(prefix_+"/prometheus/swarm_command",10);
    visual_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/prometheus/ego_pos_ref",1);
    trajectory_sub_=nh_.subscribe(prefix_+"/planning/ego/traj_cmd",1,&EgoTrajBridge::trajectory,this);
    if(!gated_) return;
    nh_.param("handover/settle_time",settle_time_,.75); nh_.param("handover/feedback_timeout",feedback_timeout_,.5);
    nh_.param("handover/trajectory_timeout",command_timeout_,.5); nh_.param("handover/max_speed",speed_limit_,.2);
    nh_.param("handover/hold_tolerance",hold_tolerance_,.15); nh_.param("handover/max_initial_error",initial_error_,.35);
    nh_.param("handover/planning_timeout",planning_timeout_,10.); nh_.param("takeoff_height",takeoff_height_,1.2);
    for(double value:{settle_time_,feedback_timeout_,command_timeout_,speed_limit_,hold_tolerance_,initial_error_,planning_timeout_,takeoff_height_})
      if(!std::isfinite(value) || value<=0) throw std::runtime_error("Handover limits must be finite and positive");
    start_pub_=nh_.advertise<geometry_msgs::PoseStamped>(prefix_+"/planning/mission_start",1);
    stop_pub_=nh_.advertise<std_msgs::Header>(prefix_+"/planning/mission_stop",1);
    status_pub_=nh_.advertise<std_msgs::String>(prefix_+"/planning/mission_state",1,true);
    std_msgs::String status; status.data=phase_; status_pub_.publish(status);
    // Aggregate DroneState can re-stamp cached sensor values. Validate the source odometry instead.
    const std::string odom_topic=prefix_+(require_frame_?"/world/odom":"/mavros/local_position/odom");
    pose_sub_=nh_.subscribe<nav_msgs::Odometry>(odom_topic,1,[this](const nav_msgs::Odometry::ConstPtr &m){
      const auto &q=m->pose.pose.orientation;
      const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      have_pose_=std::isfinite(n) && std::abs(n-1.)<.02;
      if(!have_pose_) return;
      pose_.header=m->header; pose_recv_=ros::WallTime::now();
      pose_.position={{float(m->pose.pose.position.x),float(m->pose.pose.position.y),float(m->pose.pose.position.z)}};
      pose_.velocity={{float(m->twist.twist.linear.x),float(m->twist.twist.linear.y),float(m->twist.twist.linear.z)}};
      pose_.attitude={{float(std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))),
        float(std::asin(std::max(-1.,std::min(1.,2*(q.w*q.y-q.z*q.x))))),
        float(std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))}};
      pose_.attitude_rate={{float(m->twist.twist.angular.x),float(m->twist.twist.angular.y),float(m->twist.twist.angular.z)}};
    });
    state_sub_=nh_.subscribe<mavros_msgs::State>(prefix_+"/mavros/state",1,[this](const mavros_msgs::State::ConstPtr &m){state_=*m;state_time_=ros::Time::now();});
    land_sub_=nh_.subscribe<mavros_msgs::ExtendedState>(prefix_+"/mavros/extended_state",1,[this](const mavros_msgs::ExtendedState::ConstPtr &m){land_=*m;land_time_=ros::Time::now();});
    frame_sub_=nh_.subscribe<std_msgs::Bool>(prefix_+"/frame_ready",1,[this](const std_msgs::Bool::ConstPtr &m){frame_=m->data;frame_recv_=ros::WallTime::now();});
    ready_sub_=nh_.subscribe<std_msgs::Bool>(prefix_+"/planning/mission_ready",1,[this](const std_msgs::Bool::ConstPtr &m){planner_ready_=m->data;});
    spline_sub_=nh_.subscribe(prefix_+"/planning/bspline",10,&EgoTrajBridge::spline,this);
    station_sub_=nh_.subscribe(prefix_+"/prometheus/swarm_command",10,&EgoTrajBridge::station,this);
    timer_=nh_.createTimer(ros::Duration(.02),&EgoTrajBridge::tick,this);
  }
};
int main(int argc,char **argv)
{
  ros::init(argc,argv,"ego_traj_to_cmd");
  try { EgoTrajBridge bridge; ros::spin(); }
  catch(const std::exception &e) { ROS_FATAL("%s",e.what()); return 1; }
  return 0;
}
