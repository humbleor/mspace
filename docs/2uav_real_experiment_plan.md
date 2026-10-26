# 双机森林真机实验计划与脚本核查

日期：2026-10-26。适用：两架 Mid-360 / Fast-LIO2 / EGO / PX4 无 GNSS 无人机，Ubuntu 20.04、ROS Noetic。PX4 按 v1.13.2 分析，实际版本以每机 ver all 为准。

本文是待执行的实验计划，不能当作真机已经通过。本次只新增本文，不修改真机脚本，不覆盖已有 Experiment/plan/2uav_real_experiment_plan.md 或其他用户修改。

> **2026-10-26 修复更新：**起飞参数已统一为 takeoff_height；TFmini 已配置 ROS 接收；桥接采用 MSB2 和独立 TX/RX；双机地面站已显式传入机数。下方问题表和 §6 保留修复前审查记录，当前地面测试操作、话题与剩余边界请以 [TFmini 与桥接地面测试](tfmini_bridge_ground_test.md) 为准。B1/B2 树匹配/在线补偿仍未启用；未完成真机验收。

## 1. 目标、顺序与能力边界

| 阶段 | 实验内容 | 场景及放行条件 |
| --- | --- | --- |
| E0 | 配置、网络、人员及记录准备 | 拆桨；配置表及失效动作明确 |
| E1 | 两机分别验证 TFmini-S 及 ROS 接入 | 拆桨；测距准确、时间有效 |
| E2 | 公共 world 坐标、外部定位链路 | 拆桨静止/人工移动；独立测量验证方向、尺度、对齐 |
| E3 | rosmsg_tcp_bridge TCP/UDP | 独立试验 Master、无控制器；真实传输与重连通过 |
| E4 | drone_detect_lidar 观测及离线偏移纠正 | 拆桨；关闭在线修正 |
| E5 | 一机飞行、另一机地面观测 | 各机独立完成定位、实际起飞高度、接管验收 |
| E6 | 双机分隔航区、低速短程 | 生产通信拓扑及失效处理通过 |
| E7 | 在线偏移纠正闭环 | 当前未具备，须先完成 §8 前置改造 |

顺序不可跳过。未实现/未验证功能写为 BLOCKED。

- TFmini 测到的是下方表面距离，不等于融合进定位 Z。
- 当前仓库没有实现本次讨论的 TFmini 间隙保护/沿地形规划；E1/E5 先验证观测，不宣称自动保护已生效。
- 树干匹配估计坐标表示间 XY/yaw 偏差，不是识别邻机机体，不修正 Z。
- 共享 ROS Master 下跨机可见的话题，不一定经 rosmsg_tcp_bridge 传输。

## 2. 修复前真机脚本问题清单（历史审查）

核查范围：三个双机脚本及其 launch、C++ 实际消费端。语法能解析不表示功能已接通。

| 编号 | 当前事实/位置 | 影响与实验处置 |
| --- | --- | --- |
| B1 | ego_fastlio2_swarm_uav1.sh / uav2.sh 未启动 drone_detect_lidar，注释明确暂不树匹配 | 当前脚本不能验证相对偏移纠正；E4 需专用观测入口 |
| B2 | ego_swarm_control.launch 在 world 模式强制 drift_correction/enable=false | 保持关闭做离线实验；不是只改 true 就能闭环 |
| B3 | px4_pluginlists.yaml 禁用 distance_sensor；px4_config.yaml 仍是 sonar/lidarlite 示例 | NSH 正常不等于 ROS 有 TFmini；先配置专用发布映射 |
| B4 | UAV1 脚本调用 ego_station.launch 未传 swarm_num，默认 1 | 双机应显式传 swarm_num:=2；否则缺少默认 UAV2 控制终端/完整双机显示 |
| B5 | launch 写 takeoff_height=1.2；C++ 读取 Takeoff_height，缺省 1.0 | 大小写错误；不能按 1.2 m 设计飞行。修正并核对运行值后放飞 |
| B6 | drone_detect_lidar.launch 云输入写死 pcl_render_node/cloud，默认 odom/relay 为 lidar_slam/odom | 仿真接线不能直接用在真机，需重接并关闭仿真 relay |
| B7 | TCP 接收 node2 发布 /uav1/planning/swarm_trajs，发送 node1 正订阅同名话题 | 共享 Master 存在回灌闭环，不可原样双 bridge + 双 EGO 飞行 |
| B8 | 两 bridge 共订阅 /broadcast_bspline、发布 /broadcast_bspline2 | 共享 Master 下重发/重复；需独立本机 TX/远端 RX 及去重 |
| B9 | bridge 未实现树、DroneState、frame_ready 传输；odom/STOP 注册被注释 | 不是通用透传；共享 Master 下这些数据走 ROS，不能算 bridge 成果 |
| B10 | TCP 状态是变化事件，不是完整数据年龄心跳；断链控制部分仅缩放速度前馈，位置目标继续 | 不等同可靠停车；仅地面验证实际 setpoint 和 PX4 失效动作 |
| B11 | 控制器检查所有配置 UAV 的 frame_ready，任一失效后停止发 setpoint | 可能引起两机各自 Offboard 丢失处理，不是自动 Hold |
| B12 | MAVROS 脚本按命令名 pkill 清理，cleanup 统一 exit 0 | 不能按退出码判断成功；落地后按本轮 PID 检查，不误杀无关进程 |
| B13 | 固定 sleep、无限等 Master、GNOME 子终端无全链路监控 | 终端打开不代表节点健康；用频率、时间、发布者验收 |

部署还需核对固定 IP、串口、雷达话题、默认摆位及机体-IMU 外参。LIDAR_ID 仅改话题后缀，不改硬件配置；隔离雷达网段可同 IP，共享二层网络不能地址冲突。

其他源码限制：

1. tree_loc_node 保存最近输入定时计算，未见完整输入年龄/跨机时间差拒绝，结果时间为计算时刻；旧树可能被刷新为“新结果”。
2. 树修正仅作用于控制器内部邻机位置，不同步改 PX4/LIO、world、EGO 邻机轨迹或地图。
3. 树检测直接处理 XYZ、拼接多帧，没有通用 TF/运动补偿；PatchWork++ 坐标和传感器高度假设需验证，不能只改 frame_id。
4. world 是起飞前固定对齐，不持续修正漂移；显示初始 5 m 不证明独立精度。
5. 既有评估脚本面向漂移注入/仿真；真机不得把自身 LIO 当独立真值。

## 3. E0：配置冻结与准备

### 3.1 两机现场配置表

| 项 | UAV1 | UAV2 |
| --- | --- | --- |
| 控制网 IP（当前脚本） | 192.168.1.50 | 192.168.1.40 |
| 生产 Master | http://192.168.1.50:11311 | 同左 |
| ROS_IP | .50 | .40 |
| uav_id / MAV_SYS_ID / target_system_id | 1 / 1 / 1 | 2 / 2 / 2 |
| USB 稳定路径、MID-360 配置/IP/后缀 | 填实测值 | 填实测值 |
| TFmini 串口、光束、安装外参 | 填实测值 | 填实测值 |
| commit、本地 diff、PX4 ver all | 保存 | 保存并比对 |
| world 初始位姿 | (0,0,0), yaw=0 | (0,d,0), yaw=0，d 实测 |
| 当前脚本 GCS | .100，接收 14550 | .100，接收 14551 |

当前默认 d=5 m，UAV2 在 UAV1 左侧；两机水平、同向、共同地面高度。本轮先不覆盖不同地面/航向。旧 Experiment/plan 文档的 .70 GCS、1.2 m 起飞不能替代当前源码值核对。

各机独立接管飞手，另设观察/记录人员。拆桨完成 E0–E4；选平整、可直接看清两机、有共视树干的区域。机间距和航区根据机体尺寸、定位误差、制动距离登记，不以首次近距交叉/对向冲突验证功能。

### 3.2 每机 Ubuntu Bash 检查

每个新终端都加载环境，工作目录为项目根目录：

~~~bash
cd ~/workspace/mspace
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
git rev-parse HEAD
git status --short
ls -l /dev/serial/by-id/
ip -br addr
ip route
chronyc tracking
chronyc sources -v
~~~

无 chrony 时用已有同步工具。保存实际时钟偏差，建议先以小于 20 ms 为初筛，最终满足轨迹时间预算；不同步不能测单向网络延迟。ping 不能证明 TCPROS、桥接端口正常。

需要构建时按 AGENTS：加载 ROS、ws_livox 后 ./compile.sh，检查完整汇总和退出码；运行前 source devel/setup.bash。保留用户修改。公共消息/跨模块改动需完整构建；本次文档无须编译。

### 3.3 PX4 参数基线

| 参数 | 值 |
| --- | --- |
| EKF2_HGT_MODE | 3，LIO 外部高度 |
| EKF2_AID_MASK | 24，外部位置和航向 |
| EKF2_RNG_AID | 0 |
| MPC_ALT_MODE | 0 |
| SENS_TFMINI_CFG | 每机已确认的实际串口 |

该配置不融合 TFmini 到定位 Z，不在同一试次更换主高度源。QGC 导出每机完整参数，NSH 保存：

~~~text
ver all
tfmini status
listener distance_sensor
listener vehicle_local_position
listener estimator_status_flags
param show EKF2_HGT_MODE
param show EKF2_AID_MASK
param show EKF2_RNG_AID
param show MPC_ALT_MODE
param show COM_OF_LOSS_T
~~~

核实该机 Offboard 丢失、遥控丢失、低电量、围栏和人工接管动作。不能假定断网自动悬停，林下无 GNSS 返航不作默认可用策略。

## 4. E1：下视传感器

### 4.1 静态标定，两机分别完成

1. 拆桨，稳定支撑，平整标靶；距离从传感器光学参考点测量。
2. 0.5、1、1.5、2、3 m 各静止 30 s，重复 3 轮。
3. 更换硬地、落叶、草和低灌木，注明命中表面，不把灌木顶部当地面误差。
4. 小角度倾斜，记录斜距/姿态；拆桨测试遮挡、超范围、断开数据线，检查是否滞留旧值。
5. 当前固件有效下限 0.4 m，以下只观察盲区，不计入精度。

记录均值偏差、绝对误差 P95、标准差、无效比例、跳变。建议平整目标 0.5–3 m 初筛：均值绝对偏差 ≤0.10 m、P95 ≤0.15 m。这是待确认的实验判据，不是厂家保证或飞行间隙。

### 4.2 ROS 接入

先关闭 B3：启用 distance_sensor、配置实际 MAVLink ID、向下方向、FOV、安装外参。FCU→ROS，不用 NSH device_id 代替 MAVLink ID；TF 仅一位发布者。确认飞控到机载电脑的 MAVLink 实例发送 DISTANCE_SENSOR，目标先设 20 Hz 并实测。

拟定接口（当前尚未配置）：

~~~text
/uav1/mavros/distance_sensor/tfmini_down
/uav2/mavros/distance_sensor/tfmini_down
sensor_msgs/Range
frame_id: uav1/tfmini_link / uav2/tfmini_link
~~~

~~~bash
rostopic list | rg 'distance_sensor|tfmini'
rostopic info /uav1/mavros/distance_sensor/tfmini_down
rostopic hz /uav1/mavros/distance_sensor/tfmini_down
rostopic delay /uav1/mavros/distance_sensor/tfmini_down
rostopic echo -n 1 /uav1/mavros/distance_sensor/tfmini_down
~~~

UAV2 替换编号。确认本机唯一发布者、单位 m、采样时间有效。建议初筛 ≥10 Hz、延迟 P95 <0.15 s，最终由控制预算确定。Range 没有原始强度字段，不能假定有质量值。

仅 NSH 正常时，硬件 PASS、ROS 接入 BLOCKED。测距正常不等于保护功能存在。

## 5. E2：分步启动与坐标检查

先 MAVROS、LIO、world 控制节点观测，不启动 EGO、bridge、人工指令终端。每条前台命令分别开终端运行，不重复启动相同节点。

UAV1 各终端设置并加载 §3 环境：

~~~bash
export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.50
unset ROS_HOSTNAME
~~~

一个终端 roscore，其他终端分别执行：

~~~bash
roslaunch mavros_bringup px4_mavros_swarm_basic.launch \
  uav_id:=1 enable_world_frame:=true gcs_ip:=192.168.1.100
~~~

~~~bash
roslaunch fast_lio swarm_mid360_driver.launch uav_id:=1
~~~

~~~bash
LIDAR_ID=192_168_2_174  # 替换为真实话题后缀
roslaunch fast_lio mapping_mid360.launch uav_id:=1 enable_world_frame:=true \
  lidar_topic:=/uav1/livox/lidar_$LIDAR_ID imu_topic:=/uav1/livox/imu_$LIDAR_ID
~~~

~~~bash
roslaunch prometheus_swarm_control ego_swarm_control.launch \
  swarm_num:=2 uav_id:=1 enable_world_frame:=true \
  uav_init_x:=0 uav_init_y:=0 uav_init_z:=0 uav_init_yaw:=0
~~~

MAVROS 默认 /dev/ttyACM0，不同则传 serial_port:=现场路径。

UAV2 各终端设同一 Master、ROS_IP=192.168.1.40、unset ROS_HOSTNAME，加载环境；MAVROS/driver/mapping 命令所有编号改为 2，后缀为实物值。其控制入口：

~~~bash
roslaunch prometheus_swarm_control ego_swarm_control.launch \
  swarm_num:=2 uav_id:=2 enable_world_frame:=true \
  uav_init_x:=0 uav_init_y:=5.0 uav_init_z:=0 uav_init_yaw:=0
~~~

5.0 替换为实测 d。B5 未修复不发 Takeoff。UAV1 开一份地面站：

~~~bash
roslaunch prometheus_swarm_control ego_station.launch \
  swarm_num:=2 start_terminal_control:=false
~~~

检查：

~~~bash
rosnode list
rostopic info /uav1/mavros/vision_pose/pose
rostopic hz /uav1/mavros/vision_pose/pose
rostopic echo -n 1 /uav1/world/odom
rostopic echo -n 1 /uav2/world/odom
rostopic hz /uav1/frame_ready
rostopic hz /uav2/frame_ready
rostopic echo -n 1 /uav1/frame_ready
rostopic echo -n 1 /uav2/frame_ready
rosparam get /swarm_controller_uav_1/drift_correction/enable
rosparam get /swarm_controller_uav_1/Takeoff_height
rosparam get /swarm_controller_uav_1/takeoff_height
~~~

两个大小写参数用于暴露 B5：小写存在不代表 C++ 消费，正确键及启动打印均需核对。

两机静止等待 READY，再通过各机 NSH 确认外部高度/位置/航向实际融合及水平位置有效。READY 不是完整 EKF 验收。RViz Fixed Frame=world，查看双方 world/cloud 和 world/odom：

- 多个非共线、不同距离的同一标靶/树干重合。
- 无桨一机沿已测直线移动 1–2 m，核对方向、尺度，另一机不随之移动。
- 小角度转动复位，核对 yaw 和外参。
- 初始化显示 d 不是独立精度；用卷尺标定点、全站仪等参照。
- 数据异常锁存后停止、重新摆位初始化，不在空中重启。
- 建议初筛位置误差 ≤0.15 m、yaw ≤2°，航区还要考虑误差上界。

本轮两机均就绪，每次只一机飞。不能 UAV2 断电时直接用 swarm_num=2 做 UAV1 单机；也不能假定 UAV2 配 swarm_num=1 就行，当前 readiness 按 1..swarm_num 枚举，会检查 UAV1。单机编号/门控入口需要专门核对。

## 6. E3：通信台架与生产拓扑

### 6.1 实际通道

| 数据 | 当前实现 |
| --- | --- |
| /uavN/planning/swarm_trajs，MultiBsplines | TCP 8080 |
| /broadcast_bspline → /broadcast_bspline2，Bspline | UDP 8081 |
| 树消息、邻机 odom、DroneState、frame_ready | 当前 bridge 未接通；生产共享 Master 下可由 ROS/relay 传 |
| bridge_connection_state | TCP 连接变化，不证明 UDP 或消息新鲜 |

### 6.2 隔离台架

两机拆桨，停止生产 EGO/bridge，不启控制器。同机不能多个 bridge 占固定端口。

1. 两机各启动 roscore -p 11321，试验终端各设 ROS_MASTER_URI=http://127.0.0.1:11321、ROS_IP 为自身地址，两个 Master 独立，排除 ROS 旁路。
2. UAV1 启动下方命令；UAV2 用 uav_id=2、next_drone_ip=.50。

~~~bash
roslaunch rosmsg_tcp_bridge bridge.launch \
  uav_id:=1 next_drone_ip:=192.168.1.40 broadcast_ip:=192.168.1.255
~~~

3. 有限时长发送/回放已知有效 Bspline/MultiBsplines，仅选轨迹消息，不回放控制/解锁/起飞/setpoint。
4. UAV1 TCP TX 应到达 UAV2 的 /uav1/planning/swarm_trajs；UDP 双向到对端 /broadcast_bspline2。node1 无有效 TCP 轨迹接收发布出口，当前并非对称透传。
5. 统计源 ID、traj_id、数组内容/长度、发送/接收数、重复和丢失。旧 bag start_time 不是本轮发送时刻，不能直接测网络延迟。
6. 拆桨停止对端 bridge 再恢复至少 5 次；再做本轮 AP/网线断开恢复，记录重连、残留消息及日志。
7. 关闭试验 Master/程序，恢复生产环境，确认后续飞行节点未连到 11321。

~~~bash
ss -ltnp | rg ':8080'
ss -lunp | rg ':8081'
ss -tnp | rg ':8080'
rostopic info /broadcast_bspline
rostopic info /broadcast_bspline2
rostopic echo -n 1 /uav1/bridge_connection_state
~~~

connection_state 非周期/非完整 latched 状态，echo 可能等不到新事件；以 socket 和有效负载共同判断。不构造非法空轨迹测试反序列化器。

### 6.3 当前共享 Master 不可直接集成

代码中存在回灌路径：

~~~text
UAV1 EGO → /uav1/planning/swarm_trajs → UAV1 bridge TCP TX
→ UAV2 bridge RX → 发布同一个 /uav1/planning/swarm_trajs
→ 共享 Master 再交给 UAV1 bridge → 再发送……
~~~

本次仅静态核查此路径，未实际制造网络风暴。独立 Master 台架通过仍不代表生产整图通过。

建议保留共享 Master，将本机 TX/远端 RX 话题分离，按本机编号过滤并去重，planner 每个邻机只消费一份。当前没有现成 launch 开关。若改独立 Master，还需显式传状态、frame_ready、树消息及必要 TF，不能仅改环境变量。

整改后先无控制器发有限消息，源停后消息数不能持续增长；再加入 EGO 复验。未完成不放行 E6。

## 7. E4：树干匹配与离线纠正

### 7.1 语义与接线

当前 SVD：P=本机树，Q=邻机树，输出 Q ≈ R*P+t，即本机表示到邻机表示。已对齐 world 的理想结果接近单位变换，不等于实测机间 5 m。

两机控制器当前使用邻机节点的 tree_pose_error 修正邻机位置；配对及正/逆方向须用已知数据验证。Vector3 无来源/目标 ID 和源时间，不能只靠话题名/输出频率。

真机观测入口待补齐，不能直接跑仿真整套入口：

- drone_id=1/2、uav_num=2、enable_graph_opt=false、enable_relay=false。
- 本机 odom=/uavN/world/odom，邻机 odom/树检测各一条单向 relay 到相应 neighbor 输入，无回环。
- drift_correction/enable=false，不写控制闭环。
- 验证 PatchWork++ 坐标假设；建议在传感器/重力对齐局部系剔地，再按同一时刻位姿将树特征转 world。直接 world 云必须先证明分割正确。
- 不积累未运动补偿 body 云；初次静态可 accumulation_frame_count=0，随后验证补偿/固定坐标积累。
- 闭环前补输入年龄、树/odom 时间差拒绝；不能用计算时间伪装测量新鲜度。
- 实测检测频率/CPU，compute_frequency 不等于检测器随点云回调的输出频率。

共享 ROS 下 relay 只能验证 ROS 接线，不能算 bridge 已传输树消息。

### 7.2 步骤与评价

1. 两机固定，共视至少 3 棵非共线树，实际建议 ≥5 棵，记录几何布局。
2. 静态观测 120 s，重复 3 轮，记录匹配数量、RMS、dx/dy/yaw、失败比例及原始输入年龄。
3. 人工平移一机至已测位置；再增加遮挡、减少共视、改变距离。失去共视时不能把旧输入持续发布成有效新测量。
4. 复制数据到离线专用话题，对一套树及 odom 同步注入已知 SE(2) 偏差，例如平移 0.3 m、yaw 3°，不污染实时定位。
5. 对比估计与注入真值，检查互逆关系、方向及修正前后邻机相对位置误差。
6. 无独立真值只报告注入恢复能力、共视残差、重复性，不报绝对定位精度。

建议初筛高质量段平移误差 ≤0.15 m、yaw ≤2°；修正后误差应真实减少，报告全部试次。阈值试验前登记，RMS 小不能排除误关联。现有 evaluate_drift_estimation.py / evaluate_correction_offline.py 使用前核对数据格式、真值和当前 C++ 公式。

## 8. E5–E7：飞行放行

### E5：一机飞行

相关 B 项修复，参数/单机定位/接管通过后，两机保持就绪，一机解锁，另一机不进入航区。

1. 用已核对的 Takeoff_height 悬停 30–60 s，TFmini 处于有效量程。
2. 记录测距、融合状态、world 位置、姿态、CPU/磁盘。
3. 在测量过的无障碍单机航区低速短程往返；UAV1 落地后 UAV2 重复。
4. EGO 任务另行放行，悬停成功不证明自动任务可飞；树匹配仅观测。
5. 自动间隙保护未实现，不能因为有测距就取消人工监控。

### E6：双机协同

生产桥接整图通过、EGO 接收新鲜邻机轨迹、坐标一致、地面故障演练完成后，顺序起飞、分隔航区悬停，再做平行有限任务。速度/航高/间距依据净空、误差、制动能力确定，首次不做交叉冲突。

real_ego_run.launch 固定 flight_type=value 3、扫描框 20×16 m，不可当现场试验任务；不能假定传 flight_type:=1 能覆盖。需专用小航区入口/明确目标并重新解析。

连接状态、轨迹年龄、坐标、机间距分别验收。飞行中不主动断网、停止 frame_adapter、重启 LIO、注入漂移或改参数；当前前馈缩放不等于整条轨迹降速。

### E7：在线纠正目前 BLOCKED

只开检测或 drift_correction/enable=true 不放行，先完成：

1. 明确修正层：控制器邻机位置、EGO 邻机轨迹等必须语义一致，不能宣称仅改一处就全系统纠正。
2. 明确锚机、变换方向/所有权、来源/目标 ID、采样时间及协方差，避免互相追逐修正。
3. 输入时效、质量门控、限幅、迟滞、过期退出。
4. 同一真机包先关闭/影子修正对照，再完成仿真或地面闭环演练。
5. 独立参考/已知注入真值证明收益，验证误配/退化/断链；不在线重置已锁定 world。

未完成以 E4 离线纠正结果交付，不安排真机闭环。

## 9. 记录、终止与结果

### 9.1 本地记录

两机相同 RUN_ID、不同 UAV_ID：

~~~bash
RUN_ID=E2_run01
UAV_ID=1
RUN_DIR="$HOME/data/mspace_real/$(date +%Y%m%d)_${RUN_ID}_uav${UAV_ID}"
mkdir -p "$RUN_DIR"
git rev-parse HEAD > "$RUN_DIR/commit.txt"
git status --short > "$RUN_DIR/worktree_status.txt"
rosparam dump "$RUN_DIR/rosparams.yaml"
rosnode list > "$RUN_DIR/nodes.txt"
rostopic list > "$RUN_DIR/topics.txt"
~~~

保存每机 .ulg、参数、实测摆位/外参、时钟偏差、风/地面/遮挡。不转储凭据或带凭据整份脚本。最小包示例：

~~~bash
rosbag record --split --duration=60 --buffsize=128 -O "$RUN_DIR/core.bag" \
  /tf /tf_static /rosout \
  /uav${UAV_ID}/drone_Odom_high_freq \
  /uav${UAV_ID}/mavros/vision_pose/pose \
  /uav${UAV_ID}/mavros/local_position/odom \
  /uav${UAV_ID}/mavros/state \
  /uav${UAV_ID}/mavros/statustext/recv \
  /uav${UAV_ID}/world/odom /uav${UAV_ID}/frame_ready \
  /uav${UAV_ID}/prometheus/drone_state \
  /uav${UAV_ID}/prometheus/swarm_command \
  /uav${UAV_ID}/planning/ego/traj_cmd \
  /uav${UAV_ID}/world/setpoint_raw/local \
  /uav${UAV_ID}/mavros/setpoint_raw/local \
  /uav${UAV_ID}/bridge_connection_state
~~~

E1 加实际 TFmini；E3 加 TX/RX；E4 加双方树检测、neighbor_odom、tree_pose_error、tree_relative_pose_msg 及需要的点云。未启用修正不期待 drift_correction_diag 出现。不存在话题先查清，空包不算通过。

按阶段选择大点云/原始 IMU，先比较录包前后 CPU、内存、磁盘、消息频率/丢包。保留原始 header，接收时间不替代测量年龄；跨机单向延迟先校验时钟。

### 9.2 中止与停机

定位无效/重置、点云错层、相对位置不符、测距异常仍被信任、邻机轨迹过期、回灌/负载超门槛、任一 frame_ready 丢失、侵入航区/间距下限/视线丢失、人员进入、电池/电机/遥控异常，均停止本轮任务，由各机飞手按已验证方式接管落地。

不把空中 Ctrl+C/拔电脑电源当正常暂停，不把 Disarm 当一般刹停。落地上锁后停止新任务/规划，按本轮 PID 退出控制/传感器/bridge，最后 MAVROS 与自行启动 roscore；核对两机残留，不杀无关进程。

### 9.3 每轮结果模板

| 字段 | 记录 |
| --- | --- |
| RUN_ID/阶段、两机版本和参数差异 | |
| 实测摆位/外参/时钟偏差 | |
| 测距偏差/P95/无效率、ROS 频率和延迟 | |
| world 对齐、位移尺度/yaw 误差 | |
| TCP/UDP 收发、重复、丢失、重连 | |
| 树匹配数/RMS/输入年龄/XY-yaw 误差 | |
| 修正前后误差及独立参照/注入真值 | |
| 实际任务、最小间距、接管情况 | |
| bag/.ulg/现场记录路径和记录负载 | |
| PASS/FAIL/BLOCKED、原因、后续放行 | |

## 10. 本次静态核查与验证范围

已执行（2026-10-26）：

- bash -n：双机 MAVROS、UAV1、UAV2、单机 Livox 四脚本语法通过。
- ShellCheck 排除动态外部 source 的 SC1090/SC1091 后，仍有 UAV1_HOSTNAME 未使用警告，不是无警告全通过。
- 本机加载 ROS、ws_livox、devel 后，roslaunch --nodes 解析双机 MAVROS、Livox driver、Fast-LIO、world 控制、EGO、树检测及双机地面站观测入口成功。
- roslaunch --dump-params 确认实机 EGO flight_type=3、drone_id=2。
- 代码核对确认参数键、测距禁用、树匹配关闭、桥接话题及失效行为。

未执行：未连接两架真机、未做实际网络吞吐/断线试验、未验证 ROS TFmini/保护闭环、未启动飞行节点、未发送任何解锁/起飞/降落；未修改/编译业务代码。

主要依据：三个双机脚本；Modules/swarm_control/WORLD_FRAME.md、src/swarm_frame_adapter.cpp、src/swarm_controller.cpp、include/swarm_controller.h、launch/ego_swarm_control.launch；Experiment/mavros/launch/；Modules/ego_planner_swarm/drone_detect_lidar/launch/、src/tree_loc_node.cpp；Modules/ego_planner_swarm/rosmsg_tcp_bridge/src/bridge_node.cpp。

PX4 v1.13.2 高度融合参考：https://github.com/PX4/PX4-Autopilot/blob/v1.13.2/src/modules/ekf2/EKF/control.cpp
