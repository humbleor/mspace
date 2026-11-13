#include "swarm_controller.h"
#include <std_msgs/Int32MultiArray.h>

using namespace std;
void mainloop_cb(const ros::TimerEvent &e);
void control_cb(const ros::TimerEvent &e);

namespace {
bool use_bridge = false;
ros::SteadyTime neighbor_trajectory_last_update, all_trajectories_since;
constexpr double neighbor_trajectory_timeout = 2.0;
constexpr double trajectory_recovery_delay = 0.5;

void setTrajectorySpeed(bool available)
{
    const double target = available ? 1.0 : 0.5;
    if (vel_scale_target_ == target) return;
    vel_scale_target_ = target;
    if (available)
        ROS_INFO("Neighbor trajectories available, ramping velocity setpoint to 1.0x");
    else
        ROS_WARN("Neighbor trajectories unavailable, ramping velocity setpoint to 0.5x");
}

void tcpStateCb(const std_msgs::Bool::ConstPtr &msg)
{
    bridgeStateCb(msg, !use_bridge);
}

void neighborTrajectoriesCb(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    const auto now = ros::SteadyTime::now();
    if (!neighbor_trajectory_last_update.isZero() &&
        (now - neighbor_trajectory_last_update).toSec() >= neighbor_trajectory_timeout) {
        all_trajectories_since = ros::SteadyTime();
        setTrajectorySpeed(false);
    }
    neighbor_trajectory_last_update = now;
    for (int id = 1; id <= swarm_num; ++id) {
        if (id != uav_id && std::find(msg->data.begin(), msg->data.end(), id) == msg->data.end()) {
            all_trajectories_since = ros::SteadyTime();
            setTrajectorySpeed(false);
            return;
        }
    }
    if (all_trajectories_since.isZero()) all_trajectories_since = now;
    if ((now - all_trajectories_since).toSec() >= trajectory_recovery_delay)
        setTrajectorySpeed(true);
}
} // namespace

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 读取参数，变量初始化
    init(nh);
    nh.param("require_frame_ready", require_frame_ready, false);
    nh.param("use_bridge", use_bridge, false);
    if (require_frame_ready) {
        if (controller_flag != 1 || swarm_num < 1 || swarm_num > MAX_UAV_NUM) {
            ROS_FATAL("world-frame mode requires PX4 position control and swarm_num in 1..50");
            return 1;
        }
        for (int id = 1; id <= swarm_num; ++id) {
            frame_readiness[id] = {false, ros::WallTime()};
            frame_ready_subscribers.push_back(nh.subscribe<std_msgs::Bool>(
                "/uav" + std::to_string(id) + "/frame_ready", 10,
                [id](const std_msgs::Bool::ConstPtr& msg) {
                    frame_readiness[id] = {msg->data, ros::WallTime::now()};
                }));
        }
    }

    // 建议控制频率 ： 10 - 50Hz, 控制频率取决于控制形式，若控制方式为速度或加速度应适当提高频率
    ros::Rate rate(controller_hz);

    if(flag_printf)
    {
        printf_param();
    }

    //【订阅】集群控制指令
    command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10, swarm_command_cb);

    //【订阅】本机状态信息
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb);

    bridge_state_sub = nh.subscribe(uav_name + "/bridge_connection_state", 10, tcpStateCb);
    ros::Subscriber neighbor_trajectory_sub;
    if (use_bridge && swarm_num > 1) {
        setTrajectorySpeed(false);
        neighbor_trajectory_sub = nh.subscribe(uav_name + "/planning/usable_neighbor_trajectories",
                                              1, neighborTrajectoriesCb);
    }

    //【订阅】邻居飞机的状态信息
    for(int i = 1; i <= swarm_num; i++) 
    {
        if(i == uav_id)
        {
            continue;
        }

        nei_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav" + std::to_string(i) + "/prometheus/drone_state", 10, boost::bind(nei_state_cb, _1, i));

        //【订阅】邻居的漂移补偿信息
        tree_drift_sub[i] = nh.subscribe<geometry_msgs::Vector3>("/uav" + std::to_string(i) + "/tree_pose_error", 10, boost::bind(tree_drift_cb, _1, i));

        //【发布】漂移修正诊断信息
        drift_correction_pub[i] = nh.advertise<drone_detect_lidar::DriftCorrection>(
            uav_name + "/drift_correction_diag_" + std::to_string(i), 10);
    }

    // 【发布】位置/速度/加速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
    
    // 【发布】姿态期望值 
    setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 【服务】解锁/上锁
    //  本服务通过Mavros功能包 /plugins/command.cpp 实现
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");

    // 【服务】修改系统模式
    //  本服务通过Mavros功能包 /plugins/command.cpp 实现
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");

    ros::Timer debug_timer = nh.createTimer(ros::Duration(10.0), debug_cb);
    ros::Timer mainloop_timer = nh.createTimer(ros::Duration(0.1), mainloop_cb);
    // Bridge reports every 0.5 wall seconds. Direct ROS mode has no heartbeat.
    ros::WallTimer bridge_watchdog = nh.createWallTimer(ros::WallDuration(0.5),
        [](const ros::WallTimerEvent &) {
            if (use_bridge && swarm_num > 1 && !neighbor_trajectory_last_update.isZero() &&
                (ros::SteadyTime::now() - neighbor_trajectory_last_update).toSec() >= neighbor_trajectory_timeout) {
                all_trajectories_since = ros::SteadyTime();
                setTrajectorySpeed(false);
            }
            if (!bridge_last_heartbeat_.isZero() && !bridge_heartbeat_missing_ &&
                (ros::SteadyTime::now() - bridge_last_heartbeat_).toSec() > 2.0) {
                bridge_heartbeat_missing_ = true;
                ROS_WARN("[bridge] Link status heartbeat missing for over 2 seconds");
            }
        });
    ros::Timer control_timer = nh.createTimer(ros::Duration(1.0/controller_hz), control_cb);

    ros::spin();

    return 0;
}

void mainloop_cb(const ros::TimerEvent &e)
{
    if (!framesReady() && Command_Now.Mode != prometheus_msgs::SwarmCommand::Disarm) {
        // Drop commands queued before initialization or after readiness loss.
        Command_Now.Mode = prometheus_msgs::SwarmCommand::Idle;
        Command_Now.yaw_ref = 0;
        Command_Last = Command_Now;
        if (!_DroneState.armed) idle();
        return;
    }
    // Check for geo fence: If drone is out of the geo fence, it will land now.
    if(check_failsafe() == 1)
    {
        Command_Now.Mode = prometheus_msgs::SwarmCommand::Land;
    }

    switch (Command_Now.Mode)
    {
    // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
    case prometheus_msgs::SwarmCommand::Idle:
        
        idle();

        // 设定yaw_ref=999时，切换offboard模式，并解锁
        if(Command_Now.yaw_ref == 999)
        {
            if(_DroneState.mode != "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "OFFBOARD";
                set_mode_client.call(mode_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Setting to OFFBOARD Mode...");
            }

            if(!_DroneState.armed)
            {
                arm_cmd.request.value = true;
                arming_client.call(arm_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Arming...");
            }
        }
        break;

    // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度    
    case prometheus_msgs::SwarmCommand::Takeoff:
        
        // 设定起飞点
        if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Takeoff)
        {
            // 设定起飞位置 
            Takeoff_position = pos_drone;
            pos_des[0] = pos_drone[0];
            pos_des[1] = pos_drone[1];
            pos_des[2] = pos_drone[2] + takeoff_height;
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des    = yaw_drone;
        }
        break;

    // 【Hold】 悬停。当前位置悬停
    case prometheus_msgs::SwarmCommand::Hold:

        if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Hold)
        {
            pos_des = pos_drone;
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = yaw_drone;
        }
        break;

    // 【Land】 降落。两种降落方式： 只有加载了参数Land_mode为1时，启用第二种降落方式；默认启用第一种降落方式。
    //  第一种：当前位置原地降落，降落后会自动上锁，且切换为mannual模式
    //  第二种：当前位置原地降落，降落中到达Disarm_height后，切换为飞控中land模式
    case prometheus_msgs::SwarmCommand::Land:
        if(Land_mode == 1)
        {
            mode_cmd.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(mode_cmd);
        }
        else
        {
            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Land)
            {
                // 设定起飞位置
                pos_des[0] = pos_drone[0];
                pos_des[1] = pos_drone[1];
                pos_des[2] = 0.0;
                vel_des << 0.0, 0.0, -Land_speed;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = yaw_drone;
            }

            if(_DroneState.position[2] < Disarm_height)
            {
                //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);

                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                ROS_INFO_STREAM_ONCE ("---->Landed....");
            }
        }
        
        break;

    case prometheus_msgs::SwarmCommand::Disarm:
        ROS_INFO_STREAM_ONCE ("---->Disarm....");
        if(_DroneState.mode == "OFFBOARD")
        {
            mode_cmd.request.custom_mode = "MANUAL";
            set_mode_client.call(mode_cmd);
        }

        if(_DroneState.armed)
        {
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }
        break;

    case prometheus_msgs::SwarmCommand::Position_Control:

        //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点
        pos_des[0] = Command_Now.position_ref[0] + formation_separation(uav_id-1,0) - gazebo_offset[0];
        pos_des[1] = Command_Now.position_ref[1] + formation_separation(uav_id-1,1) - gazebo_offset[1];
        pos_des[2] = Command_Now.position_ref[2] + formation_separation(uav_id-1,2) - gazebo_offset[2];
        yaw_des = Command_Now.yaw_ref;
        break;

    case prometheus_msgs::SwarmCommand::Velocity_Control:

        //　平面阵型，xy控制速度，z轴高度定高
        //　一阶积分器分布式编队控制算法，可追踪时变轨迹，此处采用双向环的拓扑结构，且仅部分无人机可收到地面站发来的虚拟领队消息
        //　参考文献：Multi-Vehicle consensus with a time-varying reference state 公式(11) - (12)
        //　目前该算法有一定控制偏差，可能是由于参数选取不是最佳导致的
        
        yita = 1/ ((float)swarm_num * k_aij + k_p);

        pos_des[0] = 0.0;
        pos_des[1] = 0.0;
        pos_des[2] = Command_Now.position_ref[2] + formation_separation(uav_id-1,2);

        vel_des[0] = yita * k_p * ( Command_Now.velocity_ref[0] - k_gamma * (pos_drone[0] - Command_Now.position_ref[0] - formation_separation(uav_id-1,0)));
        vel_des[1] = yita * k_p * ( Command_Now.velocity_ref[1] - k_gamma * (pos_drone[1] - Command_Now.position_ref[1] - formation_separation(uav_id-1,1)));
        vel_des[2] = 0.0;

        for(int i = 1; i <= swarm_num; i++) 
        {
            if(i == uav_id)
            {
                continue;
            }

            vel_des[0] += - yita * k_aij * ( vel_nei[i][0] - k_gamma *((pos_drone[0] - pos_nei[i][0]) - ( formation_separation(uav_id-1,0) -  formation_separation(i-1,0))));
            vel_des[1] += - yita * k_aij * ( vel_nei[i][1] - k_gamma *((pos_drone[1] - pos_nei[i][1]) - ( formation_separation(uav_id-1,1) -  formation_separation(i-1,1))));
        }
        
        if(collision_flag == 1)
        {
            // 计算APF项
            for(int i = 1; i <= swarm_num; i++) 
            {
                if(i == uav_id)
                {
                    continue;
                }

                float distance = (pos_drone - pos_nei[i]).norm();
                
                if(distance > R)
                {
                    dv.setZero();
                }else if(distance > r)
                {
                    dv = 4*(R*R-r*r)*(distance*distance - R*R)/pow(distance*distance - r*r,3) * (pos_drone - pos_nei[i]);
                }else
                {
                    dv.setZero();
                }

                // 降低避障的影响，调小这个参数
                vel_des[0] -= 0.5*dv[0];
                vel_des[1] -= 0.5*dv[1];
            }  
        }

        acc_des << 0.0, 0.0, 0.0;
        yaw_des = Command_Now.yaw_ref + formation_separation(uav_id-1,3);
        break;

    case prometheus_msgs::SwarmCommand::Accel_Control:

        //To be continued; 此处加速度控制实则控制的是３轴油门，限制幅度为[-1, 1] , [-1, 1], [0,1]

        //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点,并计算期望加速度（期望油门）
        //　此处也可以根据自己的算法改为　分布式控制
        //　需要增加积分项　否则会有静差
        
        acc_des[0] =  2.5 * (Command_Now.position_ref[0] + formation_separation(uav_id-1,0) - pos_drone[0]) + 3.0 * (Command_Now.velocity_ref[0] - vel_drone[0]);
        acc_des[1] =  2.5 * (Command_Now.position_ref[1] + formation_separation(uav_id-1,1) - pos_drone[1]) + 3.0 * (Command_Now.velocity_ref[1] - vel_drone[1]);
        acc_des[2] =  2.0 * (Command_Now.position_ref[2] + formation_separation(uav_id-1,2) - pos_drone[2]) + 3.0 * (Command_Now.velocity_ref[2] - vel_drone[2]) + 9.8;
        yaw_des = Command_Now.yaw_ref + formation_separation(uav_id-1,3);
        break;

    // 单个飞机情况
    case prometheus_msgs::SwarmCommand::Move:

        //　此控制方式即为　期望位置点控制, 仅针对单个飞机
        if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
        {
            pos_des[0] = Command_Now.position_ref[0];
            pos_des[1] = Command_Now.position_ref[1];
            pos_des[2] = Command_Now.position_ref[2];
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = Command_Now.yaw_ref;
        }else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
        {
            pos_des[0] = 0.0;
            pos_des[1] = 0.0;
            pos_des[2] = Command_Now.position_ref[2];
            vel_des[0] = Command_Now.velocity_ref[0];
            vel_des[1] = Command_Now.velocity_ref[1];
            vel_des[2] = 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = Command_Now.yaw_ref;
        }else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
        {
            for(int i=0; i<3; i++)
            {
                pos_des(i) = Command_Now.position_ref[i];
                vel_des(i) = Command_Now.velocity_ref[i];
                acc_des(i) = Command_Now.acceleration_ref[i];
            }
            yaw_des = Command_Now.yaw_ref;
        }
        else
        {
            pos_des[0] = Command_Now.position_ref[0];
            pos_des[1] = Command_Now.position_ref[1];
            pos_des[2] = Command_Now.position_ref[2];
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = Command_Now.yaw_ref;
            cout << RED  << "Wrong swarm command: switch to XYZ_POS "  << TAIL << endl;
        }

        break;

    case prometheus_msgs::SwarmCommand::User_Mode1:

        break;
    }
    Command_Last = Command_Now;
}

void control_cb(const ros::TimerEvent &e)
{
    if (!framesReady()) return;
    if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Idle  ||
         Command_Now.Mode == prometheus_msgs::SwarmCommand::Disarm  )
    {
        return;
    }

    // 平滑斜坡：每帧向 vel_scale_target_ 逼近 VEL_SCALE_RAMP_PER_SEC / controller_hz
    //   例：1.0→0.5、controller_hz=50、RAMP=0.5/s → 每帧减 0.01，共 50 帧 = 1 秒匀速过渡
    double ramp_delta = VEL_SCALE_RAMP_PER_SEC / controller_hz;
    if (vel_scale < vel_scale_target_)      vel_scale = std::min(vel_scale + ramp_delta, vel_scale_target_);
    else if (vel_scale > vel_scale_target_) vel_scale = std::max(vel_scale - ramp_delta, vel_scale_target_);

    if(controller_flag == 0)
    {
        // 计算控制量
        pos_controller();

        // 发送角度期望值
        send_attitude_setpoint(u_att);
    }else
    {
        if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Takeoff )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Hold )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Land )
        {
            send_pos_vel_xyz_setpoint(pos_des, vel_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Position_Control )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Velocity_Control )
        {
            send_vel_xy_pos_z_setpoint(pos_des, vel_des * vel_scale, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Accel_Control )
        {
            // 暂不可用
            // throttle_sp =  accelToThrottle(acc_des, 1.0, 20.0);
            // send_acc_xyz_setpoint(throttle_sp, yaw_des);   
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Move )
        {
            if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
            {
                send_pos_setpoint(pos_des, yaw_des);
            }
            // else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            // {
            //     send_vel_xy_pos_z_setpoint(pos_des, vel_des, yaw_des);
            // }
            else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                send_pos_vel_acc_setpoint(pos_des, vel_des * vel_scale, acc_des, yaw_des);
            }
            else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
            {
                send_pos_vel_xyz_setpoint(pos_des, vel_des * vel_scale, yaw_des);
            }
            else
            {
                send_pos_setpoint(pos_des, yaw_des);
            }
        }else
        {
            cout << RED  << "Wrong swarm command!"  << TAIL << endl;
        }      
    }

}
