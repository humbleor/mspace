# rosmsg_tcp_bridge（MSB2）

当前桥传输 UDP 单机 Bspline 和 TCP 逐跳 MultiBsplines，保留原线格式、话题与实机启动方式。此处为已实现的第一轮改动；docs/swarm_conn_msb_design.md 中的新 MSB/全互联方案尚未实现。

## 桥模式仿真

Ubuntu/WSL Bash 中加载 ROS、Livox 工作区及本仓库 devel 环境后：

```bash
roslaunch ego_planner 2uav_mid360_sim.launch use_bridge:=true
```

无 GUI 时追加 `rviz:=false`。该独立双机入口当前默认 `use_bridge:=true`，显式传入 `use_bridge:=false` 使用 ROS 直连。入口依赖 `~/bagfiles/resource/plot2/Largeforest-2_38_32.pcd`，运行前核对文件。

桥模式复用本包 ground_test.launch：UAV1 使用本地 18080/18081，UAV2 使用 28080/28081，TCP peer 和 UDP peer 端口交叉配置。该入口固定为两机；不要同时启动占用相同端口的另一份 ground_test。

EGO 的 TX/RX remap 必须同时切换：单条轨迹从 `/uavN/bridge/tx/bspline` 到对端 `/uavM/bridge/rx/bspline`；前驱链从 `/uav1/bridge/tx/multi_traj` 到 `/uav2/bridge/rx/prev_multi_traj`。桥模式没有 `/broadcast_bspline` 直连旁路。

`Simulation/start.sh swarm` 的 PX4/Gazebo 多机入口可追加 `--use-bridge`，该入口默认仍为直连。它按机数动态生成本地 TCP/UDP 端口，具体见 Simulation/px4_forest/docs/fleet.md。

同机多端口模拟广播使用可选 `udp_peer_ports` 整数数组，同一个 broadcast_ip 下向每个端口发送一份原始帧。数组为空时沿用原 udp_peer_port 单目标，实机默认广播方式不变。

## 实机

继续使用 ego_fastlio2_swarm_uav1.sh / ego_fastlio2_swarm_uav2.sh 的独立 bridge.launch 启动方式，两端编号分别为 1/2，当前 swarm_num 默认 2。地址以部署网络为准，同机仿真的 localhost 端口不用于替代实机配置；不另从规划 launch 重复启动桥。

EGO 接收方 N 的启动链必须来自 N-1，长度 N，仅含空占位与 1..N-1 的轨迹。向下一跳发送时追加本机轨迹。桥模式入口在缓存合并前拒绝错误来源、包含接收方自身或缺少前驱的链；原直连模式行为保留。

## 诊断

```bash
rostopic echo /uav1/bridge/diagnostics
rostopic echo /uav2/bridge/diagnostics
```

采用 diagnostic_msgs/DiagnosticArray，随原 0.5 s 状态定时器发布：

- `uavN/bridge/transport`：TCP 上下游状态、重连、非法帧、发送失败、待发/在发字节、待发覆盖次数、最近发送耗时以及 UDP 包/坏包/自身回灌/发送失败计数。
- `uavN/bridge/peerM`：最近结构合法数据的 TCP/UDP 接收年龄、最近轨迹 ID/原始时间/源年龄/剩余时间、UDP 合法包和重复包计数。
- 尚未观察到数据时年龄为 -1；重复合法包刷新接收观测，但不修改轨迹原始时间或延长其寿命。
- 诊断只是桥的结构校验与时间观测，不表示 EGO 接受、坐标一致或任务可继续。ROS 时间跳变、重新定位后的业务失效尚未在本轮实现。

`bridge_connection_state` 表示 TCP 拓扑。桥模式控制器使用 EGO 的 `/uavN/planning/usable_neighbor_trajectories`（`std_msgs/Int32MultiArray`）。EGO 每 0.2 秒墙钟发布一次，数组只列出已接纳且剩余时域大于 0.1 ROS 秒的邻机编号。控制器按 `swarm_num` 检查所有其他成员，缺少任一成员或 2 秒墙钟内无状态更新时，将速度设定值平滑降至 0.5 倍。持续收到完整状态至少 0.5 秒墙钟后恢复 1.0 倍。PX4 的位置设定值仍参与轨迹跟踪，实际飞行速度由飞控闭环决定。

## UDP 启动与补发

`use_bridge` 模式下，EGO 首次规划依据从 UDP 收到的有效前驱轨迹。每架机广播自己的轨迹，后机在本地按编号等待前驱。EGO 每 0.5 个 ROS 秒检查一次，只补发本机正在执行且仍有效的轨迹，保留原始轨迹 ID 和起始时间。桥统计重复包，同一轨迹键的重复转发间隔至少为 200 毫秒墙钟。规划器按原始起始时间和样条时域判断有效性。

前机正常完成任务且仍有有效里程计时，如果当前位置距终点不超过 0.5 m、速度不超过 0.2 m/s，且里程计时间戳距现在不超过 0.5 ROS 秒，EGO 每 0.5 ROS 秒广播一次基于实测位置的 2 秒静止预测。后机可据此启动，旧运动轨迹仍按时过期。退出任务、里程计过期或飞机开始移动后停止刷新；该预测不发送给本机轨迹执行器，也不作为查询服务的执行中轨迹返回。

实机双主机须在进入 OFFBOARD 前完成系统时钟同步，并确认机间相对偏差低于 `~fsm/bridge_clock_bound`（默认 0.10 秒；范围 0–0.25 秒）。这是部署时核实的误差预算，节点不会测量或补偿实际偏差。桥模式允许缓存预算内略微超前的包，到其起始时刻才计入启动和邻机健康状态；有效截止时间预留同样的误差预算。补发仍按轨迹剩余时域判断，不要求所有 UDP 包都在起始后 0.25 秒内抵达。

## TCP 发送

ROS 回调仅校验、编码和交接，已有 sender 线程独占 TCP socket。最多一帧在发、一帧待发；仅替换未开始发送的待发帧，单帧最大沿用原 1 MiB payload 上限。部分写保持帧完整性，沿用约 200 ms 发送截止时间；失败断开并清空队列。

本地连接代次检查防止编码期间跨重连交接旧帧。重连不重放队列，仍通过原 request_current 请求 EGO 重建当前有效链。当前桥模式的 EGO 通过 UDP 接收路径判断前驱轨迹是否可用于首次规划。

## 验证

```bash
./compile.sh
PYTHONWARNINGS=ignore::ResourceWarning python3 Modules/swarm_control/test/real_io_test.py
bash Modules/swarm_control/test/run_frame_tests.sh
python3 Simulation/run.py test
```

测试脚本使用自己的隔离 Master；运行前加载环境，检查 compile.sh 最终汇总。新增测试覆盖UDP 启动门控、双机仿真 remap、两台 EGO 进程在首个 UDP 包漏收后的顺序启动、诊断时效与重复包、慢 TCP 对端、发送截止时间、断线清空和重连不重放。

2026-11-03：完整六模块构建、通信/接口 15 项、坐标 launch 4 项与 ROS adapter 7 项通过；Simulation/run.py test 的三组 33/31/80 项通过。完整双机 Mid-360 桥模式无 GUI 冒烟中，两机均产生/接收轨迹并运动，未发现 /broadcast_bspline 发布旁路，两台桥退出码均为 0。

冒烟只覆盖启动和初始运动；运行日志出现原 EGO 0.25 s 轨迹时间门限拒收，退出时还出现 boost mutex 异常，原直连模式对照也复现，具体仿真组件尚未定位。不能据此声称完整任务、弱网避碰或实机飞行已验证。双主机真实网络地面测试仍需部署环境执行。


## Consumer diagnostics and local trajectory query

The controller warns if a previously observed bridge status heartbeat stops for over
two wall seconds. This diagnostic does not alter the bridge-mode speed policy.
EGO excludes expired predictions from the usable-neighbor report.

The bridge-mode EGO node offers a read-only local ROS service, for example:

```bash
rosservice call /uav_1_ego_planner_node/planning/get_current_trajectory "{}"
```

A valid response contains the executing spline with its original ID and start time.
The query does not depend on the predecessor chain. The local snapshot validation
contract requires the caller to verify peer identity, coordinate epoch and clock
uncertainty before calling it. Network snapshot exchange and cache invalidation
are separate follow-up work. The non-bridge direct ROS path keeps the 0.25 s window.
