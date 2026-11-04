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
    fsm.have_recv_pre_agent_=false;
    auto wrong=boost::make_shared<traj_utils::MultiBsplines>();
    wrong->drone_id_from=2;
    wrong->traj={traj_utils::Bspline(),spline(1,now),spline(2,now)};
    fsm.swarmTrajsCallback(wrong);
    require(!fsm.have_recv_pre_agent_,"chain containing receiver itself opened startup gate");
    require(fsm.multi_bspline_msgs_buf_.traj.empty(),"invalid chain polluted forwarding cache");
    wrong->drone_id_from=1; wrong->traj={traj_utils::Bspline(),traj_utils::Bspline()};
    fsm.swarmTrajsCallback(wrong);
    require(!fsm.have_recv_pre_agent_,"empty predecessor opened startup gate");
    auto first=boost::make_shared<traj_utils::MultiBsplines>();
    first->drone_id_from=1; first->traj={traj_utils::Bspline(),spline(1,now)};
    fsm.swarmTrajsCallback(first);
    require(fsm.have_recv_pre_agent_,"valid predecessor did not open startup gate without own trajectory");
    require(fsm.planner_manager_->local_data_.start_time_.isZero(),"startup required a local trajectory");
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
    fsm.mission_gate_=true; fsm.mission_initialized_=true;
    fsm.mission_active_=true; fsm.have_trigger_=true; fsm.have_target_=true; fsm.have_new_target_=true;
    fsm.planner_manager_->local_data_.start_time_=now;
    auto stop_msg=boost::make_shared<std_msgs::Header>(); stop_msg->stamp=ros::Time::now();
    fsm.missionStopCallback(stop_msg);
    require(!fsm.mission_active_ && !fsm.have_target_ && !fsm.have_trigger_, "stop did not clear mission permission");
    require(fsm.planner_manager_->local_data_.start_time_.isZero(), "stop retained a replayable old local spline");
    fsm.target_type_=1; fsm.have_odom_=true; fsm.odom_stamp_=ros::Time::now();
    auto start_msg=boost::make_shared<geometry_msgs::PoseStamped>();
    start_msg->header.frame_id="world"; start_msg->header.stamp=stop_msg->stamp-ros::Duration(.01);
    fsm.missionStartCallback(start_msg);
    require(!fsm.mission_active_, "cancelled start event was replayed");
    start_msg->header.stamp=ros::Time::now();
    fsm.missionStartCallback(start_msg);
    require(fsm.mission_active_ && fsm.have_trigger_, "fresh mission start did not grant permission");
    auto delayed_stop=boost::make_shared<std_msgs::Header>(); delayed_stop->stamp=stop_msg->stamp;
    fsm.missionStopCallback(delayed_stop);
    require(fsm.mission_active_, "delayed previous stop cancelled the new mission");
    fsm.planner_manager_->pp_.max_vel_=.6; fsm.planner_manager_->pp_.max_acc_=1.;
    fsm.wps_={Eigen::Vector3d(0,0,2),Eigen::Vector3d(8,0,2),Eigen::Vector3d(16,0,2)};
    fsm.odom_pos_=Eigen::Vector3d(0,0,1.); fsm.start_pt_=fsm.odom_pos_;
    fsm.end_pt_=fsm.wps_.back(); fsm.planning_horizen_=3.5;
    fsm.target_type_=3; fsm.wp_id_=0; fsm.level_grid_approach_=true;
    require(fsm.planner_manager_->planGlobalTrajWaypoints(fsm.odom_pos_,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),fsm.wps_,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),true),"level grid setup failed");
    fsm.getLocalTarget();
    require((fsm.local_target_pt_-fsm.wps_.front()).norm()<1e-9,"local lookahead skipped the grid approach endpoint");
    require(fsm.local_target_vel_.norm()<1e-9,"grid approach endpoint did not request zero velocity");
    fsm.level_grid_approach_=false; fsm.getLocalTarget();
    require((fsm.local_target_pt_-fsm.wps_.front()).norm()>1.,"normal cruise lookahead was not restored");
    // Arrival must release the approach even if planning never entered EXEC_TRAJ.
    fsm.level_grid_approach_=true; fsm.exec_state_=ego_planner::EGOReplanFSM::GEN_NEW_TRAJ;
    fsm.odom_pos_=fsm.wps_.front(); fsm.odom_vel_.setZero();
    fsm.updateGridApproach();
    require(!fsm.level_grid_approach_ && fsm.wp_id_==1,"already-at-first-point approach did not release");
    fsm.level_grid_approach_=true; fsm.exec_state_=ego_planner::EGOReplanFSM::REPLAN_TRAJ;
    fsm.odom_pos_=fsm.wps_.front()+Eigen::Vector3d(.3,0,0); fsm.odom_vel_.setZero();
    fsm.updateGridApproach(); require(fsm.level_grid_approach_,"reference arrival was mistaken for physical arrival");
    fsm.odom_pos_=fsm.wps_.front(); fsm.updateGridApproach();
    require(!fsm.level_grid_approach_ && fsm.exec_state_==ego_planner::EGOReplanFSM::GEN_NEW_TRAJ,"delayed physical arrival was stuck in replanning");
    fsm.level_grid_approach_=true; fsm.exec_state_=ego_planner::EGOReplanFSM::EMERGENCY_STOP;
    fsm.updateGridApproach();
    require(fsm.level_grid_approach_ && fsm.exec_state_==ego_planner::EGOReplanFSM::EMERGENCY_STOP,"grid arrival overrode emergency stop");
    std::cout << "Bridge chain merge and mission reset regression passed" << std::endl;
  } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return 1; }
  return 0;
}
