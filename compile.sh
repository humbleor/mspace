#!/bin/bash

catkin_make --source Experiment/mavros --build build/mavros_bringup
catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/fast_lio2 --build build/fast_lio2
# catkin_make --source Modules/ego_planner --build build/ego_planner
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
catkin_make --source Modules/swarm_control --build build/swarm_control
catkin_make --source Modules/realsense_ros --build build/realsense_ros

# # Compile Gazebo Simulator Plugins
# catkin_make --source Simulator/gazebo_simulator --build build/mspace_gazebo
# catkin_make --source Simulator/realsense_gazebo_plugin --build build/realsense_gazebo_plugin
# catkin_make --source Simulator/velodyne_gazebo_plugins --build build/velodyne_gazebo_plugins
# catkin_make --source Simulator/livox_laser_gazebo_plugins --build build/livox_laser_gazebo_plugins