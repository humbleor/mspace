# Mspace 森林仿真

本目录提供森林数据准备、传感器/LIO 轻量闭环和单机/多机 PX4/Gazebo 物理闭环。森林飞行链路为：

```text
森林 PCD → 模拟 Mid-360 / IMU → Fast-LIO2 → 状态估计 / PX4 EKF
    → EGO 规划 → 控制模块 / MAVROS → PX4 / Gazebo → 新的传感器数据
```

## 1. 目录与数据

```text
~/workspace/
├── mspace/                         # 本项目
│   ├── Simulation/
│   │   ├── start.sh                # 一键构建、测试和启动
│   │   ├── prepare_scene.sh        # 一键准备森林场景
│   │   ├── run.py                  # Python 工具入口
│   │   ├── forest_lio/             # 数据准备、传感器与轻量闭环
│   │   └── px4_forest/             # PX4/Gazebo 物理闭环
│   └── artifacts/                  # 本地生成，不随仓库提供
│       ├── forest_lio/             # 地图、覆盖文件、场景及离线结果
│       └── px4_forest/
│           ├── PX4-Autopilot/      # 隔离的 PX4 源码与构建
│           ├── plant_build/       # 本项目 Gazebo 物理接口插件
│           └── <本次运行目录>/     # 飞行日志与报告
├── PX4_Firmware/                   # 构建时使用的外部 PX4 Git 仓库
└── ws_livox/                       # 已构建的 Livox 驱动工作区
```

两个 `artifacts` 子目录均已由 Git 忽略，克隆仓库后不会包含其中的文件。下文中的路径是本地输入或输出位置，不是可在仓库中查看的文档链接。原始 Mid-360 bag 也需自行准备，默认放在 `~/bagfiles/`。

## 2. 环境与构建

使用 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 和 PX4 v1.13.2。需先安装 MAVROS、项目依赖和外部 Livox 驱动；WSL 打开 Gazebo/RViz 需要 WSLg 或 X Server。项目依赖见 [根目录说明](../README.md)，PX4 详细构建要求见 [PX4 模块说明](px4_forest/README.md)。

以下命令均在 Ubuntu/WSL Bash 中执行。尚无 PX4 源仓库时先获取：

```bash
cd ~/workspace
git clone --branch v1.13.2 --recursive \
  https://github.com/PX4/PX4-Autopilot.git PX4_Firmware
# 尚未安装 PX4 工具链时执行：
bash ~/workspace/PX4_Firmware/Tools/setup/ubuntu.sh --no-nuttx
```

当前构建固定提交为 `46a12a09bf11c8cbafc5ad905996645b4fe1a9df`。已有仓库时先核对版本，无需重复 clone。

```bash
cd ~/workspace/mspace
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
./compile.sh                     # 检查最终汇总无失败
source devel/setup.bash
bash Simulation/start.sh build   # 构建隔离 PX4 和物理插件
```

PX4 源目录不同可设置 `PX4_SOURCE`；Livox 工作区不同可设置 `LIVOX_WORKSPACE`。日常运行使用 `artifacts/px4_forest/` 下的构建产物，无需每次重建。

## 3. 首次准备森林场景

已有有效场景、地图和覆盖文件时可跳过。首次使用或更换森林数据时，需要先完成 bag 审计、Fast-LIO2 回放、覆盖分析和场景生成。

使用 [prepare_scene.sh](prepare_scene.sh) 一键完成四步，脚本自动加载 ROS、Livox 和项目环境；任何一步失败即停止，并拒绝覆盖非空输出目录：

```bash
cd ~/workspace/mspace
BAG="$HOME/bagfiles/yxb_20260208-104116.bag"
DATA="artifacts/forest_lio/scene_$(date +%Y%m%d_%H%M%S)"
bash Simulation/prepare_scene.sh --bag "$BAG" --output "$DATA"
```

不传参数时使用示例 bag，并自动创建带时间和 PID 的输出目录。完成后打印 `scene.json` 路径和短程启动命令，不自动启动飞行。

默认参数针对该 bag：

| 参数 | 含义 |
| --- | --- |
| `--map-window 40 195` | 仅积累 bag 起点后 40～195 秒的地图与观测，LIO 仍从首条原始数据初始化 |
| `--duration 192.8` | 从首条原始雷达/IMU记录起回放 192.8 秒，该 bag 中终点约为起点后 194.932 秒 |
| `--window 125 155` | 从 bag 起点后 125～155 秒的估计轨迹提取场景坐标系和参考目标 |

这些窗口来自既有实验选择，不会自动识别飞行阶段。可通过同名参数调整，`--rate` 控制回放倍速（默认 0.5）；全部选项见 `bash Simulation/prepare_scene.sh --help`。

场景文件引用生成的地图和覆盖文件，移动数据时需同步核对路径。换 bag 还需核对话题与坐标；PX4 传感器模板仍使用示例 bag 的固定话题和 40～120 秒窗口。详细工具用法见 [森林数据与 LIO 说明](forest_lio/README.md)。

## 4. 启动仿真

使用上一节生成的场景，在同一终端按需选择一条命令：

```bash
bash Simulation/start.sh short --scene "$DATA/scene.json" --template-bag "$BAG"
bash Simulation/start.sh small --scene "$DATA/scene.json" --template-bag "$BAG"
bash Simulation/start.sh full  --scene "$DATA/scene.json" --template-bag "$BAG"
```

换终端后，先将 `DATA` 和 `BAG` 设置为实际路径。脚本会自动加载 ROS、Livox 和项目环境。

如果已经按默认路径准备好场景和 bag，可直接使用：

```bash
bash Simulation/start.sh short # 起飞 + 场景第一个目标，时限 90 秒
bash Simulation/start.sh small # 六目标网格，时限 180 秒
bash Simulation/start.sh full  # 完整 15 点网格，时限 600 秒
```

默认场景为 `artifacts/forest_lio/route_125_155.json`，默认 bag 为 `~/bagfiles/yxb_20260208-104116.bag`。脚本检查这些文件，不会自动生成；不带模式参数启动等同于 `full`。

`full`、`small`、`short` 默认启用 `world-frame`，Python `px4 run --stage forest` 入口也默认开启；多机森林仿真始终使用该链路。起飞前等待坐标初始化就绪。`hover` 保持独立飞控测试，不启用 LIO 坐标适配。

默认打开 Gazebo + RViz，以 0.5 倍速运行；时限按仿真时间计算。成功后暂停并保留窗口，查看完成后关闭窗口或按 Ctrl+C，等待结果保存和进程清理。

其他常用命令：

```bash
bash Simulation/start.sh hover           # 仅 PX4/Gazebo 悬停诊断
bash Simulation/start.sh test            # 自动化检查
bash Simulation/start.sh full --headless # 无窗口运行，使用默认场景
bash Simulation/start.sh --help          # 查看全部选项
```

`--scene`、`--template-bag` 可用于指定森林数据，`--speed` 调整倍速，`--output` 指定新的空输出目录。完整网格配置见 [grid_route.json](px4_forest/config/grid_route.json)。建议先通过短程和小网格，再运行完整任务。

`test` 命令需要本地保留的 `tests/` 目录。测试文件已由 Git 忽略，不随仓库提供，新克隆的工作区不能直接运行完整历史测试；多机随代码提供的检查可用 `python3 Simulation/run.py px4 test` 单独运行。

### 多机 PX4 SITL

使用共享世界和配置列表，先双机，再通过追加飞机配置扩展：

```bash
bash Simulation/start.sh swarm --fleet Simulation/px4_forest/config/fleet_two.json \
  --output "artifacts/px4_forest/two_forest_$(date +%Y%m%d_%H%M%S)" --duration 600 --speed .25
```

多机森林入口现在可追加 `--use-bridge`，使用实际 `rosmsg_tcp_bridge` 的 TCP 前驱链与 UDP 单机轨迹；不加开关继续 ROS 直连。每次运行自动分配各机端口，同机 UDP 按对端端口列表分发，端口写入 `manifest.json`。例如：

```bash
bash Simulation/start.sh swarm --fleet Simulation/px4_forest/config/fleet_two.json \
  --use-bridge --duration 120 --speed .25 \
  --output "artifacts/px4_forest/two_bridge_$(date +%Y%m%d_%H%M%S)"
```

`--use-bridge` 用于 forest 阶段；hover 不启动 EGO，传入该组合会报错。桥只承载轨迹，状态/readiness 等仍使用本次共享 ROS Master 下的话题。

双机示例起点为 `(0,0,0)` / `(0,5,0)`，每机根据自己的 `grid` 范围和步长生成航点；也可改用显式 `goals`，两者二选一。三机改用 `config/fleet_three.json`。多机入口默认同时启动 RViz 和 Gazebo，可用 `--gui gazebo` / `--gui rviz` 只开一个窗口，或 `--gui none` 无窗口运行；每机独立雷达/IMU、LIO、MAVROS 和规划控制链。配置、坐标、速度、验收与验证边界见 [多机说明](px4_forest/docs/fleet.md)。

## 5. 查看结果

当前仿真用的是独立的 41887 端口。查看仿真话题：

```bash
source ~/workspace/mspace/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:41887

rostopic list
```

启动终端会打印本次输出目录，默认位于 `artifacts/px4_forest/<模式>_<时间>_<PID>/`。运行结束后在本地查看：

| 文件 | 内容 |
| --- | --- |
| `result.json` | 本次状态、错误、目标到达和碰撞验收结果 |
| `report.md`、`flight.png` | 自动生成的报告和轨迹图 |
| `requested_waypoint_evidence.json` | 原请求航点的顺序到达检查 |
| `cleanup.json` | 进程清理结果 |
| `navigation.log`、`px4.log`、`mavros.log` | 定位、规划、控制和飞控日志 |

以本次 `result.json` 的 `status` 和 `errors` 判断是否通过，同时检查碰撞、接触和清理结果。历史导航通过记录不能替代当前配置的运行结果。

任务完成前 Ctrl+C 会记为中断失败。若需从另一终端清理指定实验，先把 `RUN` 设为该实验的实际输出目录，再执行：

```bash
python3 Simulation/run.py px4 stop --output "$RUN" --dry-run
python3 Simulation/run.py px4 stop --output "$RUN"
```

清理完成时，`cleanup.json` 应为 `status=complete`、`remaining=[]`。

## 进一步说明

- [森林数据、传感器与轻量闭环](forest_lio/README.md)
- [PX4 构建与运行参数](px4_forest/README.md)
- [验证结果与适用边界](px4_forest/docs/validation.md)
- [原模块接入方式](px4_forest/docs/module_alignment.md)
- [森林物理碰撞](px4_forest/docs/forest_physical_collision.md)

已提供配置驱动的多机 PX4 SITL 入口；具体机数、航线和协同能力须以当前运行结果验证。实体飞控 HIL 尚未验证。森林点云及简化碰撞几何受原始观测覆盖限制。
