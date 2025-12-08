#ifndef SWARM_ESTIMATOR_H
#define SWARM_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>

#include <prometheus_msgs/DroneState.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "math_utils.h"
#include "message_utils.h"

// 宏定义
#define NODE_NAME "swarm_estimator"         // 节点名称
#define TRA_WINDOW 10                     // 发布轨迹长度
#define TIMEOUT_MAX 0.1                     // MOCAP超时阈值
// 变量
int uav_id;
string uav_name;                            // 无人机名字(话题前缀)
string object_name;                         // 动作捕捉软件中设定的刚体名字
string msg_name;
int input_source;                           // 0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
Eigen::Vector3f pos_offset;                 // 定位设备偏移量
float yaw_offset;                           // 定位设备偏移量
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3d pos_drone_mocap;            // 无人机当前位置 (mocap)
Eigen::Quaterniond q_mocap;                 // 无人机当前姿态 - 四元数 (mocap)
Eigen::Vector3d Euler_mocap;                // 无人机当前姿态 - 欧拉角 (mocap)
ros::Time mocap_timestamp;                  // mocap时间戳
//---------------------------------------里程计位姿------------------------------------------
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d Euler_t265;
Eigen::Vector3d pos_drone_lidar;
Eigen::Vector3d pos_drone_lidar_fastlio2;
Eigen::Quaterniond q_lidar;
Eigen::Quaterniond q_lidar_fastlio2;
Eigen::Vector3d Euler_lidar;
Eigen::Vector3d pos_drone_gazebo;           // 无人机当前位置 (gazebo)
Eigen::Quaterniond q_gazebo;                // 无人机当前姿态 - 四元数 (gazebo)
Eigen::Vector3d Euler_gazebo;               // 无人机当前姿态 - 欧拉角 (gazebo)
prometheus_msgs::Message message;           // 待打印的文字消息
nav_msgs::Odometry Drone_odom;              // 无人机里程计,用于rviz显示
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人机轨迹容器,用于rviz显示
// 订阅话题
ros::Subscriber state_sub;
ros::Subscriber extended_state_sub;
ros::Subscriber position_sub;
ros::Subscriber velocity_sub;
ros::Subscriber attitude_sub;
ros::Subscriber alt_sub;
ros::Subscriber t265_sub;
ros::Subscriber lidar_sub;
ros::Subscriber lidar_fastlio2_sub;
ros::Subscriber mocap_sub;
ros::Subscriber gazebo_sub;
// 发布话题
ros::Publisher drone_state_pub;
ros::Publisher vision_pub;
ros::Publisher message_pub;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;

void init();

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

void att_cb(const sensor_msgs::Imu::ConstPtr& msg);

void alt_cb(const std_msgs::Float64::ConstPtr &msg);

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(const ros::Time& begin_time);

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg);

void t265_cb(const nav_msgs::Odometry::ConstPtr &msg);

void lidar_cb(const nav_msgs::Odometry::ConstPtr &msg);

void lidar_fastlio2_cb(const nav_msgs::Odometry::ConstPtr &msg);

void timercb_vision(const ros::TimerEvent &e);

void timercb_drone_state(const ros::TimerEvent &e);

void timercb_rviz(const ros::TimerEvent &e);

#endif