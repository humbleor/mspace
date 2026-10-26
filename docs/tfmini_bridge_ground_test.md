# TFmini 接入与双机桥接地面测试

更新：2026-10-26。这里只启动测距/通信，不执行飞行任务。两机必须部署本次同一版本、重新构建，不能一台新桥接、一台旧桥接。

## 1. 本次修改的范围

- 控制器的参数键和变量统一为 takeoff_height；ego_swarm_control.launch 可传 takeoff_height:=1.2。
- MAVROS 启用物理测距接收；单机/双机入口发布每机独立 TFmini Range。
- 真机双机脚本显式 use_bridge:=true；地面站显式 swarm_num:=2。
- 桥接分离本机 TX/远端 RX，新增有界 MSB2 数据帧、TCP 拆包/粘包处理、来源校验、UDP 去重、断线重连。
- 仅接收当前 EGO 支持的三次样条，拒绝退化 knots 和非有限数值，避免异常轨迹进入规划器。
- TCP 已连接时，桥接每 0.5 s 请求 EGO 重建当前仍有效的链，覆盖首次轨迹早于连接的场景；保留原始轨迹时间，不从桥接缓存重放过期轨迹。
- 多机接收周期前驱链时合并前驱范围，保留已收到的其他无人机轨迹及时间更新的 UDP 条目。
- bridge_connection_state 为 latched、0.5 s 周期的 TCP 拓扑连接状态，仍不代表 UDP 或轨迹数据新鲜。
- 树干检测/补偿保持关闭；不改变 PX4 EKF 参数，不实现 TFmini 定位高度融合或自动离地保护。

现有仿真/不带桥接的规划器默认 use_bridge=false，仍直接使用共享 ROS 广播；真机 bridge 模式改走独立通道，不用直接 ROS 轨迹旁路。

## 2. 构建与部署

两台机分别：

~~~bash
cd ~/workspace/mspace
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
export ROS_PARALLEL_JOBS="-j2 -l2"
./compile.sh
source devel/setup.bash
~~~

检查完整汇总和退出码。ROS 消息结构没有改变，但桥接的线协议改为 MSB2；旧的 native-size_t 数据包会被拒绝，不承诺与旧桥接互通。两机必须同步更新 bridge 节点、bridge.launch 和 EGO launch。

Takeoff_height 旧大写键不再使用；同时更新现场参数文件。确认：

~~~bash
rosparam get /swarm_controller_uav_1/takeoff_height
rosparam get /swarm_controller_uav_2/takeoff_height
~~~

这两条只在控制器已经启动时查看参数；地面通信测试不必启动控制器。正确键也会在控制器启动参数中打印。

## 3. TFmini：先只开 MAVROS，确认 ROS 测距

传感器已在 NSH 验证准确。现在还需验证 FCU→MAVLink→MAVROS：

- 默认 wire sensor ID=0，须与 QGC MAVLink Inspector 的 DISTANCE_SENSOR.id 一致。
- 该 ID 不是 listener 打印的 device_id。
- 默认 FOV=2°（弧度 0.034906585），可通过 tfmini_fov 参数覆盖，须与实际设备一致。
- 每机 frame 为 uavN/tfmini_link。当前不自动发布安装 TF；没有真实外参时不伪造传感器安装位置。后续可视化/间隙处理须补充实测安装变换。

在共享 Master、网络环境正确且没有重复 MAVROS 的前提下，分别执行：

~~~bash
roslaunch mavros_bringup px4_mavros_swarm_basic.launch \
  uav_id:=1 enable_world_frame:=true tfmini_id:=0
~~~

~~~bash
roslaunch mavros_bringup px4_mavros_swarm_basic.launch \
  uav_id:=2 enable_world_frame:=true tfmini_id:=0
~~~

串口不是默认 /dev/ttyACM0 时，加 serial_port:=实际稳定路径。这个测试不需要 EGO、控制器和桥接。

检查两机：

~~~bash
rostopic echo -n 1 /uav1/mavros/distance_sensor/tfmini_down
rostopic echo -n 1 /uav2/mavros/distance_sensor/tfmini_down
rostopic hz /uav1/mavros/distance_sensor/tfmini_down
rostopic delay /uav1/mavros/distance_sensor/tfmini_down
~~~

若无数据，先核对 wire ID、插件日志和 QGC 中该消息是否存在。需要请求当前 MAVLink 链路发送 20 Hz 测距时，可在确认已连接后：

~~~bash
rosservice call /uav1/mavros/set_message_interval "{message_id: 132, message_rate: 20.0}"
rosservice call /uav2/mavros/set_message_interval "{message_id: 132, message_rate: 20.0}"
~~~

必须检查服务 success，再看实际频率；请求消息频率不是启用 EKF 高度融合。超时/串口/固件不支持时不能把无数据判断为量程错误。

每机分别改变传感器到标靶距离，核对自己的话题随之变化，另一机不变化。单位为 m；当前固件上报最小有效距离 0.4 m，低于此值不计入有效测距。

整套脚本配置 ID 的入口：

- 双机 MAVROS 脚本：环境变量 TFMINI_ID_UAV1、TFMINI_ID_UAV2，缺省均为 0。
- 单机 Livox 脚本：TFMINI_ID，缺省 0。
- 修改 ID 后重新启动对应 MAVROS，不在其他节点运行时重复开一份。

## 4. 先做本机 loopback 测试

这个入口只启动两个桥接进程，绑定 localhost 和独立端口，不启动飞控、MAVROS、规划器或控制器。

先在终端 1：

~~~bash
roscore -p 11321
~~~

其余测试终端均 source 环境并设置：

~~~bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
~~~

终端 2：

~~~bash
roslaunch rosmsg_tcp_bridge ground_test.launch
~~~

终端 3、4 分别运行，让两段时间重叠：

~~~bash
rosrun rosmsg_tcp_bridge bridge_probe.py --uav-id 1 --duration 15 --rate 5
~~~

~~~bash
rosrun rosmsg_tcp_bridge bridge_probe.py --uav-id 2 --duration 15 --rate 5
~~~

probe 仅发布有限通信样本，不能作为真实飞行轨迹。不要在运行 EGO/控制器的 Master 上执行它。

预期：

- 两端 rx_udp >0，self_rx=0。
- UAV2 rx_tcp >0；UAV1 rx_tcp=0 是正常的，因为 TCP 按前驱链 1→2 单向传输。
- 两端 tcp_connected=true。
- 发送停止后 RX 不再持续增长。启动订阅发现期间可能漏掉头几条，不把发现阶段丢失与稳定传输混算。
- age_ms 指源轨迹时间到接收观测时间，真实跨机比较必须先同步系统时钟。

## 5. 两台机真实网络地面测试

两机拆桨，不运行控制器/EGO/真机全链路脚本。两机控制网应能双向 ROS TCPROS 通信；当前配置 UAV1=.50、UAV2=.40，根据现场修改。

UAV1 各终端：

~~~bash
export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.50
unset ROS_HOSTNAME
~~~

UAV1 启动本轮自己的 roscore（已有正确测试 Master 则复用），再启动：

~~~bash
roslaunch rosmsg_tcp_bridge bridge.launch \
  uav_id:=1 swarm_num:=2 next_drone_ip:=192.168.1.40 \
  broadcast_ip:=192.168.1.255
~~~

UAV2 各终端：

~~~bash
export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.40
unset ROS_HOSTNAME
roslaunch rosmsg_tcp_bridge bridge.launch \
  uav_id:=2 swarm_num:=2 next_drone_ip:=192.168.1.50 \
  broadcast_ip:=192.168.1.255
~~~

运行 §4 的两个 probe，各在对应电脑上执行。共享 Master 只负责注册；TX/RX 已分离，样本必须经网络桥接才能到达对端 RX，没有全局 /broadcast_bspline 旁路。

实际通道：

| 本机发送 | 对端接收 | 传输 |
| --- | --- | --- |
| /uav1/bridge/tx/multi_traj | /uav2/bridge/rx/prev_multi_traj | TCP，连接 UAV2:8080 |
| /uav1/bridge/tx/bspline | /uav2/bridge/rx/bspline | UDP，8081 |
| /uav2/bridge/tx/bspline | /uav1/bridge/rx/bspline | UDP，8081 |

仅 UAV2 在本轮 2 机链中监听 TCP 8080；UAV1 没有 TCP 监听是预期。广播网络不支持时，可将各端 broadcast_ip 指定为对端实际 IP，进行 UDP 单播对照，再单独验收广播覆盖。TreeDetection、DroneState、frame_ready 不由该桥接传输。/uavN/bridge/request_current 是本机桥接→本机 EGO 的请求通道，不跨机传输；末机不需要向后发送链。

检查：

~~~bash
rostopic echo /uav1/bridge_connection_state
rostopic echo /uav2/bridge_connection_state
rostopic info /uav1/bridge/tx/bspline
rostopic info /uav2/bridge/rx/bspline
rostopic hz /uav2/bridge/rx/bspline
ss -tnp | rg ':8080'
ss -lunp | rg ':8081'
~~~

失效实验仅在这个无飞控控制的地面环境：

1. 保持 UAV1 probe 运行，停止 UAV2 bridge，观察双方 TCP 状态。
2. 重启 UAV2 bridge，等待连接恢复，发送新的轨迹编号确认再次到达。probe 持续发送测试样本；真实 EGO 则由 request_current 周期请求恢复当前有效链，已到期或前驱缺失时不响应。
3. 重复 5 次并记录恢复时长、收发/重复/丢失数量。
4. 只关闭本轮网络，再恢复；TCP keepalive 失效检测可能需要约十余秒，不能等同即时数据超时保护。
5. 单独验证 UDP 丢失：TCP=true 不能证明 UDP 正常。
6. 程序退出只清理本轮进程，不按全局可执行文件名杀进程。

## 6. 后续接入真实规划器时

真实双机脚本已显式 use_bridge:=true；手动启动 EGO 时也须一致：

~~~bash
roslaunch ego_planner real_ego_run.launch \
  uav_id:=1 enable_world_frame:=true use_bridge:=true
~~~

UAV2 编号改 2。这不是地面通信测试步骤；现场小航区、目标、frame_ready、定位与接管验收未完成前，不启动真实自动任务。现有 real_ego_run 的预设扫描任务不会因修桥接自动变安全。

use_bridge=false 用于现有直接 ROS 共享的仿真/单机模式，不能一端桥接、一端直接 ROS 混用。树匹配与在线补偿仍保持关闭。此次修改不解决所有飞行失效策略，也不构成双机飞行性能验收。

## 7. 本机软件验证

自动回归入口：

~~~bash
PYTHONWARNINGS=ignore::ResourceWarning python3 Modules/swarm_control/test/real_io_test.py
bash Modules/swarm_control/test/run_frame_tests.sh
python3 Simulation/run.py test
~~~

real_io_test 自己创建随机端口的 ROS Master、模拟 FCU MAVLink 数据和桥接进程，只连接 localhost，不接触实际飞控。覆盖参数实际作用、MAVROS wire ID 映射、桥接拓扑、TCP 拆包/粘包、异常包、自身/重复 UDP 拒绝和重连。另用实际 EGO 规划器测试晚启动桥接恢复及过期拒绝；bridge_chain_test 直接验证实际接收回调的三机缓存合并和新旧消息顺序。软件测试通过不代替本文件 §3/§5 的真机地面测量。

### 2026-10-26 本机验证结果

- ./compile.sh：六模块均构建成功。
- real_io_test.py：11 项全部通过，含实际 MAVROS 接收模拟 FCU、实际 EGO 晚连接恢复与过期拒绝、三机回调合并、异常报文和 TCP 半帧超时/断线/重连。
- Simulation/run.py test：33+16+80，共 129 项检查通过；这是软件回归检查，不代表完整 SITL 飞行验收。
- run_frame_tests.sh：C++ 坐标变换、4 项 launch 配置、7 项 ROS 适配回归通过。
- ground_test.launch 与两个已安装 probe：localhost 互通通过，无自身回灌；两端各发送 50 个 UDP 样本、各接收 48 个，UAV2 接收 48 个 TCP 样本。统计包含 ROS 初始订阅发现阶段，不能直接当作稳定网络丢包率。
- 独立静态复审发现的轨迹边界、首次链丢失和多机缓存覆盖问题已修复并增加回归；最终复审未发现新的阻断问题。

以上均为软件/localhost 验证。§3 的真实 TFmini 链路及 §5 的两机物理网络仍需现场完成；未启动实机飞行任务，未改变 PX4 EKF 参数。
