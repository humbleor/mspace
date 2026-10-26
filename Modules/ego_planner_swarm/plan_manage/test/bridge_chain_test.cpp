#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <boost/thread/mutex.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
// Load dependencies before exposing private FSM callbacks to this in-process test.
#define private public
#include <plan_manage/ego_replan_fsm.h>
#undef private
#include <stdexcept>

void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
traj_utils::Bspline spline(int id, ros::Time time) {
  traj_utils::Bspline m;
  m.drone_id=id; m.order=3; m.start_time=time; m.traj_id=id;
  m.knots={-3,-2,-1,0,1,2,3,4};
  for (int i=0;i<4;++i) { geometry_msgs::Point p; p.x=i; p.y=id; p.z=1; m.pos_pts.push_back(p); }
  return m;
}
int main(int argc, char **argv) {
  ros::init(argc,argv,"bridge_chain_regression",ros::init_options::NoSigintHandler);
  ros::Time::init();
  try {
    ego_planner::EGOReplanFSM fsm;
    fsm.planner_manager_.reset(new ego_planner::EGOPlannerManager);
    fsm.planner_manager_->pp_.drone_id=2;
    fsm.planner_manager_->local_data_.start_time_=ros::Time(0);
    fsm.have_odom_=true; fsm.odom_pos_.setZero(); fsm.planning_horizen_=10;
    fsm.bridge_mode_=true;
    const auto now=ros::Time::now();
    auto successor=boost::make_shared<traj_utils::Bspline>(spline(3,now));
    auto predecessor=boost::make_shared<traj_utils::Bspline>(spline(1,now));
    fsm.BroadcastBsplineCallback(successor);
    fsm.BroadcastBsplineCallback(predecessor);
    auto chain=boost::make_shared<traj_utils::MultiBsplines>();
    chain->drone_id_from=1; chain->traj={traj_utils::Bspline(),spline(1,now-ros::Duration(.05))};
    fsm.swarmTrajsCallback(chain);
    const auto &cache=fsm.planner_manager_->swarm_trajs_buf_;
    require(cache.size()>=4 && cache[3].drone_id==3,"periodic predecessor chain deleted successor UDP trajectory");
    require(cache[1].start_time_==now,"older TCP snapshot replaced newer UDP trajectory");
    require(fsm.multi_bspline_msgs_buf_.traj[1].start_time==now,"forwarded chain regressed to older TCP snapshot");
    auto late_udp=boost::make_shared<traj_utils::Bspline>(spline(1,now-ros::Duration(.02)));
    fsm.BroadcastBsplineCallback(late_udp);
    require(cache[1].start_time_==now,"late UDP snapshot replaced newer trajectory");
    chain->traj[1].start_time=now+ros::Duration(.01);
    fsm.swarmTrajsCallback(chain);
    require(cache[1].start_time_==chain->traj[1].start_time,"newer predecessor update did not merge");
    fsm.bridge_mode_=false;
    fsm.swarmTrajsCallback(chain);
    require(cache.size()==2,"direct-ROS mode changed its existing replacement behavior");
    std::cout << "Bridge chain merge regression passed" << std::endl;
  } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return 1; }
  return 0;
}
