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
#include <traj_utils/GetCurrentTrajectory.h>
#include <plan_manage/bridge_snapshot.h>
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
    auto snapshot=spline(1,now-ros::Duration(.8));
    require(ego_planner::usableBridgeSnapshot(snapshot,1,now,ros::Time(),.02),
            "executing snapshot older than UDP gate was rejected");
    require(!ego_planner::usableBridgeSnapshot(snapshot,1,now,snapshot.start_time,.02),
            "duplicate snapshot was accepted");
    require(!ego_planner::usableBridgeSnapshot(snapshot,1,now,snapshot.start_time+ros::Duration(.01),.02),
            "older snapshot could overwrite a newer one");
    require(!ego_planner::usableBridgeSnapshot(snapshot,1,now+ros::Duration(.2),ros::Time(),.02),
            "expired snapshot was accepted");
    require(!ego_planner::usableBridgeSnapshot(snapshot,2,now,ros::Time(),.02),
            "wrong peer identity was accepted");
    require(!ego_planner::usableBridgeSnapshot(snapshot,1,now,ros::Time(),.4),
            "untrusted clock bound was accepted");

    auto &local = fsm.planner_manager_->local_data_;
    Eigen::MatrixXd points(3,4);
    points << 0,1,2,3, 0,0,0,0, 1,1,1,1;
    local.position_traj_=ego_planner::UniformBspline(points,3,1.0);
    local.duration_=local.position_traj_.getTimeSum();
    local.start_time_=snapshot.start_time;
    local.traj_id_=42;
    fsm.exec_state_=ego_planner::EGOReplanFSM::EXEC_TRAJ;
    traj_utils::GetCurrentTrajectory query;
    require(fsm.getCurrentTrajectory(query.request,query.response) && query.response.valid,
            "local snapshot depended on a predecessor chain");
    require(query.response.trajectory.start_time==snapshot.start_time &&
            query.response.trajectory.traj_id==42 && query.response.trajectory.drone_id==2,
            "local snapshot changed source identity");
    fsm.exec_state_=ego_planner::EGOReplanFSM::EMERGENCY_STOP;
    require(fsm.getCurrentTrajectory(query.request,query.response) && !query.response.valid,
            "emergency stop exported a trajectory");
    fsm.exec_state_=ego_planner::EGOReplanFSM::EXEC_TRAJ;
    local.start_time_=now-ros::Duration(3.0);
    require(fsm.getCurrentTrajectory(query.request,query.response) && !query.response.valid,
            "expired local trajectory was exported");
    // A late follower needs a newly dated stationary prediction, not a replay
    // of the finished path. Exercise the same selector used by the UDP timer.
    fsm.exec_state_=ego_planner::EGOReplanFSM::WAIT_TARGET;
    fsm.bridge_goal_completed_=true;
    fsm.odom_pos_=fsm.end_pt_=Eigen::Vector3d(3,0,1);
    fsm.odom_vel_.setZero();
    fsm.odom_stamp_=ros::Time::now();
    traj_utils::Bspline hold;
    require(fsm.bridgeBroadcastTrajectory(hold), "settled predecessor did not offer a fresh stationary spline");
    require(hold.start_time==fsm.odom_stamp_ && hold.start_time>local.start_time_ &&
            hold.pos_pts.size()==4, "hold reused the finished path's timestamp");
    for (const auto &pt : hold.pos_pts)
      require(pt.x==3 && pt.y==0 && pt.z==1, "stationary prediction moved away from measured pose");
    require(fsm.getCurrentTrajectory(query.request,query.response) && !query.response.valid,
            "local executing-trajectory service returned a hold");
    fsm.odom_vel_.x()=.5;
    require(!fsm.bridgeBroadcastTrajectory(hold), "moving predecessor reported a hold");
    fsm.odom_vel_.setZero();
    fsm.odom_stamp_-=ros::Duration(2);
    require(!fsm.bridgeBroadcastTrajectory(hold), "stale odometry refreshed a hold");
    fsm.bridge_goal_completed_=false;

    local.start_time_=ros::Time();
    fsm.have_recv_pre_agent_=false;
    // Direct ROS keeps its original sequential startup path.
    fsm.bridge_mode_ = false;
    auto direct = boost::make_shared<traj_utils::MultiBsplines>();
    direct->drone_id_from = 1;
    direct->traj = {traj_utils::Bspline(), spline(1, now)};
    fsm.swarmTrajsCallback(direct);
    require(fsm.predecessorsReady(now), "direct ROS startup no longer accepts a predecessor chain");
    require(fsm.planner_manager_->swarm_trajs_buf_.size() == 2, "direct ROS did not replace its cache");
    require(fsm.planner_manager_->local_data_.start_time_.isZero(), "startup required a local trajectory");
    {
      ego_planner::EGOReplanFSM udp;
      udp.planner_manager_.reset(new ego_planner::EGOPlannerManager);
      udp.planner_manager_->pp_.drone_id = 3;
      udp.planner_manager_->local_data_.start_time_ = ros::Time();
      udp.have_odom_ = false;
      udp.bridge_mode_ = true;
      udp.exec_state_ = ego_planner::EGOReplanFSM::SEQUENTIAL_START;
      udp.have_recv_pre_agent_ = false;
      const auto stamp = ros::Time::now() - ros::Duration(.35);
      auto first = boost::make_shared<traj_utils::Bspline>(spline(1, stamp));
      auto second = boost::make_shared<traj_utils::Bspline>(spline(2, stamp));
      auto tcp = boost::make_shared<traj_utils::MultiBsplines>();
      tcp->drone_id_from = 2;
      tcp->traj = {traj_utils::Bspline(), *first, *second};
      udp.swarmTrajsCallback(tcp);
      require(!udp.predecessorsReady(ros::Time::now()), "TCP opened the UDP startup gate");
      const auto future = boost::make_shared<traj_utils::Bspline>(spline(1, ros::Time::now()+ros::Duration(.05)));
      udp.BroadcastBsplineCallback(future);
      require(udp.planner_manager_->swarm_trajs_buf_.size()>1 &&
              udp.planner_manager_->swarm_trajs_buf_[1].drone_id==1,
              "slight sender clock lead dropped the UDP packet");
      require(!udp.predecessorsReady(ros::Time::now()), "future timestamp opened startup");
      udp.planner_manager_->swarm_trajs_buf_.clear();
      auto too_early = boost::make_shared<traj_utils::Bspline>(spline(1, ros::Time::now()+ros::Duration(.4)));
      udp.BroadcastBsplineCallback(too_early);
      require(udp.planner_manager_->swarm_trajs_buf_.empty(), "excessive sender clock lead was accepted");
      auto almost_expired = boost::make_shared<traj_utils::Bspline>(spline(1, ros::Time::now()-ros::Duration(.95)));
      udp.BroadcastBsplineCallback(almost_expired);
      require(udp.planner_manager_->swarm_trajs_buf_.empty(), "clock budget was not reserved at expiry");
      udp.BroadcastBsplineCallback(first);
      require(!udp.predecessorsReady(ros::Time::now()), "incomplete UDP set opened startup");
      udp.BroadcastBsplineCallback(second);
      require(udp.predecessorsReady(ros::Time::now()), "complete executing UDP set did not open startup");
      auto older = boost::make_shared<traj_utils::Bspline>(spline(1, stamp - ros::Duration(.05)));
      udp.BroadcastBsplineCallback(older);
      require(udp.planner_manager_->swarm_trajs_buf_[1].start_time_ == stamp,
              "older UDP replaced the first predecessor");
      require(!udp.predecessorsReady(stamp + ros::Duration(1.01)), "UDP startup readiness stayed latched");
    }
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
    // Flight 20261106-170749: the only local spline ended at z=1.620986,
    // outside the 0.2 m arrival gate for (0,0,2). Exercise the real FSM
    // decision with a controlled clock, including a translated follower.
    for (int id : {1, 2}) {
      const Eigen::Vector3d offset = id == 1 ? Eigen::Vector3d::Zero().eval() : Eigen::Vector3d(10,5,3);
      fsm.planner_manager_->pp_.drone_id=id;
      fsm.wps_={offset+Eigen::Vector3d(0,0,2),offset+Eigen::Vector3d(5,0,2)};
      fsm.replan_thresh_=1.; fsm.have_odom_=true;
      auto reset_approach = [&](double time) {
        ros::Time::setNow(ros::Time(time));
        fsm.odom_stamp_=ros::Time::now();
        fsm.level_grid_approach_=true; fsm.wp_id_=0;
        fsm.exec_state_=ego_planner::EGOReplanFSM::EXEC_TRAJ;
        fsm.odom_pos_=offset+Eigen::Vector3d(0,0,1.620986);
        fsm.odom_vel_.setZero();
        fsm.planner_manager_->local_data_.start_time_=ros::Time(100.);
        fsm.planner_manager_->local_data_.duration_=1.15;
      };
      reset_approach(100.8); fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"unfinished grid approach was interrupted");
      reset_approach(101.7); fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"grid approach did not allow settling time");
      reset_approach(102.3); fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::GEN_NEW_TRAJ,"expired spline stranded UAV at 1.62 m instead of retrying from odometry");
      require(fsm.level_grid_approach_ && fsm.wp_id_==0,"retry skipped the first grid waypoint");
      fsm.start_pt_=fsm.odom_pos_; fsm.getLocalTarget();
      require((fsm.local_target_pt_-fsm.wps_.front()).norm()<1e-9 && fsm.local_target_vel_.norm()<1e-9,"retry changed its stopped first-waypoint target");
      // A successful retry gets its own execution/settling interval.
      fsm.exec_state_=ego_planner::EGOReplanFSM::EXEC_TRAJ;
      fsm.planner_manager_->local_data_.start_time_=ros::Time::now();
      fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"retry immediately restarted a fresh trajectory");
      // Flight 20261106-171143 at bag t=48.899: measured arrival must
      // release cruise immediately, even before the spline expires.
      reset_approach(100.8);
      fsm.odom_pos_=offset+Eigen::Vector3d(-.0101,.0304,1.9935);
      fsm.odom_vel_=Eigen::Vector3d(-.0494,.0314,.165);
      fsm.updateGridApproach();
      require(!fsm.level_grid_approach_ && fsm.wp_id_==1 && fsm.exec_state_==ego_planner::EGOReplanFSM::GEN_NEW_TRAJ,"normal second-flight arrival did not release cruise");
      reset_approach(102.3); fsm.odom_pos_=fsm.wps_.front(); fsm.odom_vel_=Eigen::Vector3d(0,0,.3);
      fsm.updateGridApproach();
      require(fsm.level_grid_approach_ && fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"fast passage was mistaken for settled arrival");
      fsm.odom_vel_.setZero(); fsm.updateGridApproach();
      require(!fsm.level_grid_approach_,"settled arrival remained stuck");
      reset_approach(102.3); fsm.odom_stamp_=ros::Time(101.);
      fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"retry used stale odometry");
      reset_approach(102.3); fsm.odom_stamp_=ros::Time(103.);
      fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"retry used future odometry");
      reset_approach(102.3); fsm.level_grid_approach_=false;
      fsm.updateGridApproach();
      require(fsm.exec_state_==ego_planner::EGOReplanFSM::EXEC_TRAJ,"normal cruise acquired grid-approach retry behavior");
      for (auto state : {ego_planner::EGOReplanFSM::INIT, ego_planner::EGOReplanFSM::WAIT_TARGET,
                         ego_planner::EGOReplanFSM::SEQUENTIAL_START, ego_planner::EGOReplanFSM::EMERGENCY_STOP}) {
        reset_approach(102.3); fsm.exec_state_=state;
        fsm.updateGridApproach();
        require(fsm.exec_state_==state && fsm.level_grid_approach_,"approach retry bypassed a startup, stop or emergency gate");
      }
    }
    std::cout << "UDP startup, direct ROS, mission reset and grid approach regressions passed" << std::endl;
  } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return 1; }
  return 0;
}
