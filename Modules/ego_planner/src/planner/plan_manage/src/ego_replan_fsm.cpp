
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    wait_time_ = true;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/step_X", stepX_, -1.0);
    nh.param("fsm/step_Y", stepY_, -1.0);
    nh.param("fsm/step_Z", stepZ_, -1.0);
    nh.param("fsm/coordinate_X", coordinate_X_, -1.0);
    nh.param("fsm/coordinate_Y", coordinate_Y_, -1.0);
    nh.param("fsm/radius_R", radius_R_, -1.0);
    nh.param("fsm/numSpiralSegments", numSpiralSegments_, 4);
    nh.param("fsm/waypointDistriFlag", waypointDistriFlag_, 1);
    nh.param("fsm/minZ_rotate", minZ_rotate_, -1.0);
    nh.param("fsm/maxZ_rotate", maxZ_rotate_, -1.0);
    nh.param("fsm/stepZ_rotate", stepZ_rotate_, -1.0);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("grid_map/box_min_x", box_min_x, -1.0);
    nh.param("grid_map/box_min_y", box_min_y, -1.0);
    nh.param("grid_map/box_min_z", box_min_z, -1.0);
    nh.param("grid_map/box_max_x", box_max_x, -1.0);
    nh.param("grid_map/box_max_y", box_max_y, -1.0);
    nh.param("grid_map/box_max_z", box_max_z, -1.0);

    // nh.param("fsm/waypoint_num", waypoint_num_, -1);
    // for (int i = 0; i < waypoint_num_; i++)
    // {
    //   nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    //   nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    //   nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    // }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      if (wait_time_)
      {
        sleep(45);
        if (waypointDistriFlag_ == 0)
        {
          // 水平方向遍历
          waypoints = generateWaypoints(box_min_x, box_max_x, box_min_y, box_max_y, box_min_z, box_max_z, stepX_, stepY_, stepZ_);
        }
        else if (waypointDistriFlag_ == 1)
        {
          // 垂直方向遍历
          waypoints = generateSinWaypoints(box_min_x, box_max_x, box_min_y, box_max_y, box_min_z, box_max_z, stepXY_);
        }
        else if (waypointDistriFlag_ == 2)
        {
          // 螺旋式上升
          // X,Y是单木的水平坐标，R是环绕飞行的半径，numSpiralSegments_是一个水平圆周上分布几个点，stepZ是绕飞的垂直步长
          waypoints = spiralrotationWaypoints(coordinate_X_, coordinate_Y_, radius_R_, numSpiralSegments_,
                                              minZ_rotate_, maxZ_rotate_, stepZ_rotate_);
        }
        else
          cout << "Wrong waypointDistriFlag_ value! waypointDistriFlag_=" << waypointDistriFlag_ << endl;

        waypoint_num_ = waypoints.size();
        wait_time_ = false;
        planGlobalTrajbyGivenWps();
      }
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    trigger_ = true;
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      // wps[i](0) = waypoints_[i][0];
      // wps[i](1) = waypoints_[i][1];
      // wps[i](2) = waypoints_[i][2];

      wps[i] = Eigen::Matrix<double, 3, 1>(std::get<0>(waypoints[i]), std::get<1>(waypoints[i]), std::get<2>(waypoints[i]));
      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    // {
    //   visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    //   ros::Duration(0.001).sleep();
    // }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(1, 0, 0, 1), 0.5, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      // have_odom_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      // changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.2, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /// @brief Heuristic Waypoint Generation
  /// @param minX X minimum
  /// @param maxX X maximum
  /// @param minY Y minimum
  /// @param maxY Y maximum
  /// @param minZ Z minimum
  /// @param maxZ Z maximum
  /// @param stepXY X and Y direction step size
  /// @param stepZ Z direction step size
  /// @return waypoint
  Points3D EGOReplanFSM::generateWaypoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double stepX, double stepY, double stepZ)
  {
    Points3D waypoints;

    for (double z = minZ; z < maxZ; z += 2 * stepZ)
    {

      for (double y = minY ; y < maxY; y += 2 * stepY)
      {
        for (double x = minX + stepX/2; x < maxX; x += stepX)
        {
          waypoints.push_back(std::make_tuple(x, y, z));
        }
        y += stepY;
        for (double x = maxX - stepX/2; x > minX; x -= stepX)
        {
          waypoints.push_back(std::make_tuple(x, y, z));
        }
        y -= stepY;
      }
      z += stepZ;
      if (z > maxZ) // Exit if the current Z value exceeds the defined Z maximum
        break;

      for (double y = maxY ; y > minY; y -= 2 * stepY)
      {
        for (double x = minX + stepX/2; x < maxX ; x += stepX)
        {
          waypoints.push_back(std::make_tuple(x, y, z));
        }
        y -= stepY;
        for (double x = maxX ; x > minX; x -= stepX)
        {
          waypoints.push_back(std::make_tuple(x, y, z));
        }
        y += stepY;
      }

      z -= stepZ;
    }
    return waypoints;
  }
  // Points3D EGOReplanFSM::generateWaypoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double stepXY, double stepZ)
  // {
  //   Points3D waypoints;

  //   for (double z = minZ + stepZ; z < maxZ; z += 2 * stepZ)
  //   {

  //     for (double y = minY ; y < maxY; y += 2 * stepXY)
  //     {
  //       for (double x = minX + stepXY/2; x < maxX; x += stepXY)
  //       {
  //         waypoints.push_back(std::make_tuple(x, y, z));
  //       }
  //       y += stepXY;
  //       for (double x = maxX - stepXY/2; x > minX; x -= stepXY)
  //       {
  //         waypoints.push_back(std::make_tuple(x, y, z));
  //       }
  //       y -= stepXY;
  //     }
  //     z += stepZ;
  //     if (z > maxZ) // Exit if the current Z value exceeds the defined Z maximum
  //       break;

  //     for (double y = maxY ; y > minY; y -= 2 * stepXY)
  //     {
  //       for (double x = minX + stepXY/2; x < maxX ; x += stepXY)
  //       {
  //         waypoints.push_back(std::make_tuple(x, y, z));
  //       }
  //       y -= stepXY;
  //       for (double x = maxX ; x > minX; x -= stepXY)
  //       {
  //         waypoints.push_back(std::make_tuple(x, y, z));
  //       }
  //       y += stepXY;
  //     }

  //     z -= stepZ;
  //   }
  //   return waypoints;
  // }
  Points3D EGOReplanFSM::generateSinWaypoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double stepXY)
  {
    Points3D waypoints;
    bool isTop = false;
    double z = minZ;
    for (double y = minY; y < maxY; y += 2 * stepXY)
    {
      for (double x = minX + stepXY/2; x < maxX; x += stepXY)
      {

        if (isTop)
          // z = maxZ;
          z = minZ;
        else
          // z = minZ;
          z = maxZ;
        isTop = !isTop;
        waypoints.push_back(std::make_tuple(x, y, z));
      }
      for (double x = maxX - stepXY/2; x > minX; x -= stepXY)
      {
        if (isTop)
          z = maxZ;
        else
          z = minZ;
        isTop = !isTop;
        waypoints.push_back(std::make_tuple(x, y + stepXY, z));
      }
    }
    return waypoints;
  }

  // Points3D EGOReplanFSM::generateSinWaypoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double stepXY)
  // {
  //   Points3D waypoints;
  //   bool isTop = false;
  //   double z = minZ;
  //   for (double y = minY + stepXY; y < maxY; y += 2 * stepXY)
  //   {
  //     for (double x = minX + stepXY; x < maxX; x += stepXY)
  //     {

  //       if (isTop)
  //         // z = maxZ;
  //         z = minZ;
  //       else
  //         // z = minZ;
  //         z = maxZ;
  //       isTop = !isTop;
  //       waypoints.push_back(std::make_tuple(x, y, z));
  //     }
  //     for (double x = maxX - stepXY; x > minX; x -= stepXY)
  //     {
  //       if (isTop)
  //         z = maxZ;
  //       else
  //         z = minZ;
  //       isTop = !isTop;
  //       waypoints.push_back(std::make_tuple(x, y + stepXY, z));
  //     }
  //   }
  //   return waypoints;
  // }

  /// @brief numSpiralSegments和stepZ共同控制螺旋的圈数,double totalSpiralTurns = (maxZ - minZ) / (stepZ * numSpiralSegments);
  /// @param X X,Y是单木的水平坐标，
  /// @param Y
  /// @param R R是环绕飞行的半径，
  /// @param numSpiralSegments 代表在螺旋路径上生成的点的数量,螺旋路径分段数
  /// @param minZ
  /// @param maxZ
  /// @param stepZ stepZ是绕飞的垂直步长
  /// @return waypoints
  Points3D EGOReplanFSM::spiralrotationWaypoints(double X, double Y, double R, int numSpiralSegments, double minZ, double maxZ, double stepZ)
  {
    Points3D waypoints;

    // 创建一个向量来存储构成圆形路径的 2D 点（X，Y）
    std::vector<std::tuple<double, double>> Point2D;

    // 计算每个连续点之间的角度步长
    double deltaAngle = 2 * M_PI / numSpiralSegments;

    // 计算圆形路径上的 2D 点并将它们存储在 Point2D 向量中
    for (size_t i = 0; i < numSpiralSegments; i++)
    {
      Point2D.push_back(std::make_tuple(X + R * cos(deltaAngle * i), Y + R * sin(deltaAngle * i)));
    }

    int index = 0;
    // 沿螺旋路径生成具有不同 Z 坐标的航点
    for (double z = minZ + stepZ; z < maxZ; z += stepZ)
    {
      // 通过循环 Point2D，确保索引不会超出范围
      if (index == Point2D.size())
        index = 0;

      // 使用 Point2D 中的相应 2D 点的 X 和 Y 坐标以及当前 Z 坐标创建 3D 航点
      waypoints.push_back(std::make_tuple(std::get<0>(Point2D[index]), std::get<1>(Point2D[index]), z));
      index++;
    }

    return waypoints;
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.2;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init); // 调用反弹规划
      if (success)                                                   // 如果规划成功才设置状态
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else // 不成功就再重新规划一次
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      // changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            // ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }


  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
