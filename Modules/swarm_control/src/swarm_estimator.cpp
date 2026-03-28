#include "swarm_estimator.h"

using namespace std;

void init()
{
    _DroneState.connected = false;
    _DroneState.armed = false;
    _DroneState.mode = "";
    _DroneState.position[0] = 0.0;
    _DroneState.position[1] = 0.0;
    _DroneState.position[2] = 0.0;
    _DroneState.velocity[0] = 0.0;
    _DroneState.velocity[1] = 0.0;
    _DroneState.velocity[2] = 0.0;
    _DroneState.attitude_q.w = 0.0;
    _DroneState.attitude_q.x = 0.0;
    _DroneState.attitude_q.y = 0.0;
    _DroneState.attitude_q.z = 0.0;
    _DroneState.attitude[0] = 0.0;
    _DroneState.attitude[1] = 0.0;
    _DroneState.attitude[2] = 0.0;
    _DroneState.attitude_rate[0] = 0.0;
    _DroneState.attitude_rate[1] = 0.0;
    _DroneState.attitude_rate[2] = 0.0;
    _DroneState.rel_alt = 0.0;
    //
    pos_drone_mocap = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_mocap         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_mocap     = Eigen::Vector3d(0.0, 0.0, 0.0);

    pos_drone_gazebo = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_gazebo         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_gazebo     = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    _DroneState.connected = msg->connected;
    _DroneState.armed = msg->armed;
    _DroneState.mode = msg->mode;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _DroneState.position[0] = msg->pose.position.x;
    _DroneState.position[1] = msg->pose.position.y;
    _DroneState.position[2] = msg->pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    _DroneState.velocity[0] = msg->twist.linear.x;
    _DroneState.velocity[1] = msg->twist.linear.y;
    _DroneState.velocity[2] = msg->twist.linear.z;
}

void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    
    _DroneState.attitude_q.w = q_fcu.w();
    _DroneState.attitude_q.x = q_fcu.x();
    _DroneState.attitude_q.y = q_fcu.y();
    _DroneState.attitude_q.z = q_fcu.z();

    _DroneState.attitude[0] = euler_fcu[0];
    _DroneState.attitude[1] = euler_fcu[1];
    _DroneState.attitude[2] = euler_fcu[2];

    _DroneState.attitude_rate[0] = msg->angular_velocity.x;
    _DroneState.attitude_rate[1] = msg->angular_velocity.x;
    _DroneState.attitude_rate[2] = msg->angular_velocity.x;
}

void alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
    _DroneState.rel_alt = msg->data;
}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Euler_mocap = quaternion_to_euler(q_mocap);
    // 记录收到mocap的时间戳
    mocap_timestamp = ros::Time::now();
}
void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_drone_gazebo = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_gazebo = quaternion_to_euler(q_gazebo);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "wrong gazebo ground truth frame id.");
    }
}
void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_t265 = quaternion_to_euler(q_t265);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong t265 frame id.");
    }
}
void lidar_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "camera_init")
    {
        pos_drone_lidar = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        q_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_lidar = quaternion_to_euler(q_lidar);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong LIDAR frame id.");
    }
}

void lidar_fastlio2_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_drone_lidar_fastlio2 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        q_lidar_fastlio2 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_lidar = quaternion_to_euler(q_lidar_fastlio2);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong LIDAR frame id.");
    }
}

void timercb_vision(const ros::TimerEvent &e)
{
    geometry_msgs::PoseStamped vision;

    //mocap
    if (input_source == 0)
    {
        vision.pose.position.x = pos_drone_mocap[0] + pos_offset[0];
        vision.pose.position.y = pos_drone_mocap[1] + pos_offset[1];
        vision.pose.position.z = pos_drone_mocap[2] + pos_offset[2];

        vision.pose.orientation.x = q_mocap.x();
        vision.pose.orientation.y = q_mocap.y();
        vision.pose.orientation.z = q_mocap.z();
        vision.pose.orientation.w = q_mocap.w();
        // 如果长时间未收到mocap数据，则一直给飞控发送旧数据，此处显示timeout
        if( get_time_in_sec(mocap_timestamp) > TIMEOUT_MAX)
        {
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Mocap Timeout.");
        }

    }
    //faster-lio
    else if (input_source == 1)
    {
        vision.pose.position.x = pos_drone_lidar[0] + pos_offset[0];
        vision.pose.position.y = pos_drone_lidar[1] + pos_offset[1];
        vision.pose.position.z = pos_drone_lidar[2] + pos_offset[2];

        vision.pose.orientation.x = q_lidar.x();
        vision.pose.orientation.y = q_lidar.y();
        vision.pose.orientation.z = q_lidar.z();
        vision.pose.orientation.w = q_lidar.w();
    }
    // gazebo
    else if (input_source == 2)
    {
        vision.pose.position.x = pos_drone_gazebo[0] + pos_offset[0];
        vision.pose.position.y = pos_drone_gazebo[1] + pos_offset[1];
        vision.pose.position.z = pos_drone_gazebo[2] + pos_offset[2];

        vision.pose.orientation.x = q_gazebo.x();
        vision.pose.orientation.y = q_gazebo.y();
        vision.pose.orientation.z = q_gazebo.z();
        vision.pose.orientation.w = q_gazebo.w();
    }
    // t265
    else if (input_source == 3)
    {
        vision.pose.position.x = pos_drone_t265[0] + pos_offset[0];
        vision.pose.position.y = pos_drone_t265[1] + pos_offset[1];
        vision.pose.position.z = pos_drone_t265[2] + pos_offset[2];

        vision.pose.orientation.x = q_t265.x();
        vision.pose.orientation.y = q_t265.y();
        vision.pose.orientation.z = q_t265.z();
        vision.pose.orientation.w = q_t265.w();
    }
    //fast-lio2
    else if (input_source == 4)
    {
        vision.pose.position.x = pos_drone_lidar_fastlio2[0] + pos_offset[0];
        vision.pose.position.y = pos_drone_lidar_fastlio2[1] + pos_offset[1];
        vision.pose.position.z = pos_drone_lidar_fastlio2[2] + pos_offset[2];

        vision.pose.orientation.x = q_lidar_fastlio2.x();
        vision.pose.orientation.y = q_lidar_fastlio2.y();
        vision.pose.orientation.z = q_lidar_fastlio2.z();
        vision.pose.orientation.w = q_lidar_fastlio2.w();
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Wrong input_source.");
    }

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void timercb_drone_state(const ros::TimerEvent &e)
{
    _DroneState.header.stamp = ros::Time::now();
    drone_state_pub.publish(_DroneState);
}

void timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人机当前odometry,用于导航及rviz显示
    nav_msgs::Odometry Drone_odom;
    Drone_odom.header.stamp = ros::Time::now();
    Drone_odom.header.frame_id = "world";
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    // 导航算法规定 高度不能小于0
    if (Drone_odom.pose.pose.position.z <= 0)
    {
        Drone_odom.pose.pose.position.z = 0.01;
    }

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
    odom_pub.publish(Drone_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "world";
    drone_pos.pose.position.x = _DroneState.position[0];
    drone_pos.pose.position.y = _DroneState.position[1];
    drone_pos.pose.position.z = _DroneState.position[2];

    drone_pos.pose.orientation = _DroneState.attitude_q;

    //发布无人机的位姿和轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), drone_pos);
    if (posehistory_vector_.size() > TRA_WINDOW)
    {
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "world";
    drone_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(drone_trajectory);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 读取参数
    nh.param("uav_id", uav_id, 0);
    // 定位数据输入源
    nh.param<int>("input_source", input_source, 0);
    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    uav_name = "/uav" + std::to_string(uav_id);
    msg_name = uav_name + "/control";

    // 变量初始化
    init();

    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
    state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);

    // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
    attitude_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, att_cb); 

    // 【订阅】无人机相对高度 此订阅仅针对户外实验
    alt_sub = nh.subscribe<std_msgs::Float64>(uav_name + "/mavros/global_position/rel_alt", 10, alt_cb);

    if (input_source == 0) {
        // 【订阅】mocap估计位置
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ uav_name + "/pose", 10, mocap_cb);
    }
    else if (input_source == 1)
    {
        // 【订阅】faster-lio估计位置
        lidar_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, lidar_cb);
    }
    else if (input_source == 2)
    {
        // 【订阅】gazebo仿真真值
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth", 10, gazebo_cb);
    }
    else if (input_source == 3)
    {
        // 【订阅】t265估计位置
        t265_sub = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 100, t265_cb);
    }
    //fast-lio2
    else if (input_source == 4)
    {
        // 【订阅】fast-lio2估计位置
        lidar_fastlio2_sub = nh.subscribe<nav_msgs::Odometry>("/drone_Odometry", 100, lidar_fastlio2_cb);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Wrong input_source.");
    }

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10);
    
    // 【发布】提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 【发布】无人机里程计,用于RVIZ显示
    odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/drone_odom", 10);

    // 【发布】无人机运动轨迹
    trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/drone_trajectory", 10);

    // 定时器,定时发送vision信息至飞控,保证50Hz以上
    ros::Timer timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);

    // 定时器,发布 drone_state_pub,保证20Hz以上
    ros::Timer timer_drone_state_pub = nh.createTimer(ros::Duration(0.05), timercb_drone_state);

    // 定时器,发布 rviz显示,保证1Hz以上
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.5), timercb_rviz);

    // 频率
    ros::Rate rate(100.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}