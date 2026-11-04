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

`bridge_connection_state` 仍保持原 TCP 拓扑语义；控制器失联动作、PX4 参数和任务恢复策略没有改变。

## TCP 发送

ROS 回调仅校验、编码和交接，已有 sender 线程独占 TCP socket。最多一帧在发、一帧待发；仅替换未开始发送的待发帧，单帧最大沿用原 1 MiB payload 上限。部分写保持帧完整性，沿用约 200 ms 发送截止时间；失败断开并清空队列。

本地连接代次检查防止编码期间跨重连交接旧帧。重连不重放队列，仍通过原 request_current 请求 EGO 重建当前有效链；它没有新增单机 TCP 快照接口，也不解决原 UDP 接收回调的 0.25 s 起始时间门限。

## 验证

```bash
./compile.sh
PYTHONWARNINGS=ignore::ResourceWarning python3 Modules/swarm_control/test/real_io_test.py
bash Modules/swarm_control/test/run_frame_tests.sh
python3 Simulation/run.py test
```

测试脚本使用自己的隔离 Master；运行前加载环境，检查 compile.sh 最终汇总。新增测试覆盖错误启动链、双机仿真 remap、两台真实 EGO 规划进程顺序启动、诊断时效与重复包、慢 TCP 对端、发送截止时间、断线清空和重连不重放。

2026-11-03：完整六模块构建、通信/接口 15 项、坐标 launch 4 项与 ROS adapter 7 项通过；Simulation/run.py test 的三组 33/31/80 项通过。完整双机 Mid-360 桥模式无 GUI 冒烟中，两机均产生/接收轨迹并运动，未发现 /broadcast_bspline 发布旁路，两台桥退出码均为 0。

冒烟只覆盖启动和初始运动；运行日志出现原 EGO 0.25 s 轨迹时间门限拒收，退出时还出现 boost mutex 异常，原直连模式对照也复现，具体仿真组件尚未定位。不能据此声称完整任务、弱网避碰或实机飞行已验证。双主机真实网络地面测试仍需部署环境执行。
