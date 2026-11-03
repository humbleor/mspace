# OFFBOARD 与 EGO 任务交接

单机、双机实机及 PX4 森林仿真共用 `ego_traj_to_cmd` 门控。实机流程：

1. 启动完整链路，确认定位、地图、飞控状态与 `frame_ready`。脚本 sleep 只用于错开节点启动，不负责安排人工起飞。
2. 遥控器 Position 起飞并稳定悬停。程序只预发送当前位置保持参考，不自动解锁或切换模式。
3. 操作者切换 OFFBOARD。门控捕获当前位置与航向，先发布 XYZ 位置保持。
4. 定位和飞控反馈有效、已离地、速度小于 0.2 m/s、距保持点小于 0.15 m、yaw 角速度小于 0.15 rad/s，且规划器就绪与两个启动订阅端连接后，连续稳定 0.75 s 才发送一次任务启动。
5. EGO 从当前里程计重新规划原配置目标，规划期间继续保持。收到本次会话的新 B 样条及有效轨迹参考，且首个参考距当前反馈不超过 0.35 m 后，开始轨迹执行。

保持阶段只暂时使用接管航向。任务航向由 EGO 的水平前瞻轨迹方向与现有转向限速计算，首帧从接管 yaw 衔接；水平前瞻位移不超过0.1米时保持上一有效航向，避免竖直爬升中的微小水平修正触发掉头。水平运动明确后继续朝轨迹前方，不锁定全程机头。此修改不修复 Position 手动起飞时转向，不能据此认定该实机问题已消除。

## 中断与重新开始

退出 OFFBOARD 或解除解锁会结束本次任务。再次启动需要实际观测到 Position，再由操作者切回 OFFBOARD；丢失连接后直接恢复 OFFBOARD、或节点在 OFFBOARD 中重启，不会自动启动新任务。

定位、坐标就绪或轨迹失效、首参考偏差过大、规划超时会进入 `blocked`，不自动恢复旧轨迹。门控发送带时间戳的停止消息，并用有效当前位置或最近有效位置的 XYZ 保持参考清除控制器缓存中的轨迹速度/加速度前馈。坐标转换已失效时，这不保证物理悬停；下游坐标保护和 PX4 失联保护仍负责相应处置。操作者发出的 Hold/Land/Disarm 优先，门控不再覆盖该指令。

## 接口与参数

所有任务话题均为 `/uavN/planning/` 下的独立话题：

| 后缀 | 类型 | 用途 |
| --- | --- | --- |
| `mission_ready` | `std_msgs/Bool`，锁存 | FSM 初始化完成 |
| `mission_start` | `geometry_msgs/PoseStamped` | 会话时间戳及接管位姿，非锁存 |
| `mission_stop` | `std_msgs/Header` | 带停止时间戳的取消，拒绝旧消息倒退会话 |
| `mission_state` | `std_msgs/String`，锁存 | waiting / holding / planning / running / blocked |

门控直接检查 `/uavN/world/odom` 的源时间戳；关闭 world adapter 时使用 MAVROS 本地 Odometry，不使用会重复刷新时间戳的聚合 DroneState 作为新鲜度依据。反馈须位于 `world`，并检查实际 MAVROS mode、armed、connected、ExtendedState 与 frame_ready。

控制入口和 `real_ego_run.launch` 的 `enable_mission_gate` 默认 true；两侧必须一致。旧仿真入口未传递门控参数时保持原行为。不要单独关闭一侧。

门控节点私有参数：

| 参数 | 默认值 |
| --- | --- |
| `handover/settle_time` | 0.75 s |
| `handover/feedback_timeout` | 0.5 s |
| `handover/trajectory_timeout` | 0.5 s |
| `handover/max_speed` | 0.2 m/s |
| `handover/hold_tolerance` | 0.15 m |
| `handover/max_initial_error` | 0.35 m |
| `handover/planning_timeout` | 10 s |

## 验证与边界

`test/mission_handover_ros_test.py` 启动独立 ROS master 和真实桥接/轨迹节点，覆盖接管、旧轨迹拒绝、反馈冻结、连接恢复、操作员降落优先、原生仿真起飞和轨迹航向。`test/run_frame_tests.sh` 检查坐标变换及 launch 接线。

PX4 森林运行器不再独立延时触发轨迹，监听相同 `mission_state` 并记录事件。其已有自动 OFFBOARD 起飞由门控等待爬升结束后交接，不能替代人工 Position 起飞切换的实机验收。先拆桨核对链路，再进行保持接管的小范围验证，最后验收 EGO 运动。真实动力断电及手动起飞转向需另行排查。

## 等高网格的起始轨迹

生成网格（`flight_type=3`）且全部任务航点等高时，全局参考分两部分拼接：当前接管状态到首个网格点采用单段轨迹，首点速度与加速度为零；之后从该静止边界生成其余网格的平滑参考。拼接处位置、速度、加速度连续。局部目标先锁定首点，里程计距首点小于0.2米且速度小于0.2米/秒后释放正常前瞻；后续网格点仍连续巡航。实机和森林 Simulation 使用相同实现，无需新参数。

该处理消除起始爬升对长水平参考的高度过冲影响，不是全程限高；局部避障与跟踪误差仍可能改变实际高度。不同高度的三维任务和普通预设航线保留原有生成方式。
