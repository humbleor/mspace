#!/usr/bin/env python3
"""Summarize verified conservative-renderer closed-loop runs."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from common import write_json
from summarize_results import differences

def main():
    root=Path(__file__).resolve().parents[3]
    data=root/"artifacts/forest_lio"
    out=data/"phase2_summary";out.mkdir(exist_ok=True)
    results={}
    fig,axs=plt.subplots(3,1,figsize=(10,8))
    for label,directory in [("Ideal","closed_loop_conservative"),("IMU scatter + bias","closed_loop_conservative_noise")]:
        d=data/directory
        r=json.loads((d/"result.json").read_text())
        results[label]=r
        tr=np.loadtxt(d/"truth.tum",ndmin=2); est=np.loadtxt(d/"lio.tum",ndmin=2)
        t=tr[:,0]-tr[0,0]
        axs[0].plot(t,np.linalg.norm(tr[:,1:4]-np.array(r["goal"]),axis=1),label=label)
        te,e=differences(est,tr)
        axs[1].plot(te-tr[0,0],e,label=label)
        axs[2].plot(tr[:,1],tr[:,3],label=label)
    axs[0].set_ylabel("Truth distance to goal (m)");axs[0].set_xlabel("Simulation time (s)")
    axs[1].set_ylabel("LIO position error (m)");axs[1].set_xlabel("Simulation time (s)")
    axs[2].set_ylabel("z (m)");axs[2].set_xlabel("x (m)")
    for ax in axs:ax.grid(alpha=.3);ax.legend()
    fig.suptitle("PCD / Mid-360 / Fast-LIO2 / EGO closed-loop smoke tests")
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/"closed_loop.png",dpi=150);plt.close(fig)
    benchmark=json.loads((data/"renderer_benchmark_v2.json").read_text())
    write_json(out/"results.json",{"closed_loop":results,"renderer":benchmark})
    lines=["# 森林雷达 / LIO / EGO 在线闭环实验","",
      "用户确认末段由遥控器主动切回 MANUAL。此前把模式切换与自主导航错误关联的描述已修正；尾段不参与自主飞行模型统计。",
      "", "## 本轮完成", "",
      "- 将原始 bag 分为静止段 2.2–17 s、自主飞行训练段 40–120 s、同次飞行时间留出段 120–195 s。201.300 s 后标注手动模式。",
      "- 从 2960 个静止 IMU 样本估计各轴样本散布和陀螺偏置；加速度均值未当作偏置，未虚构距离噪声。",
      "- 在线发布 10 Hz CustomMsg 与 200 Hz IMU；Fast-LIO2 输出里程计/配准点云给 EGO，轨迹服务器指令驱动受加速度限制的二阶运动模型。",
      "- 检查实际 ROS 订阅图，EGO 的里程计和点云均来自 LIO，导航节点没有订阅真值；没有将完整 PCD 直接提供给规划器。",
      "- 两次 12 s 测试使用同一 PCD、目标、渲染策略和随机种子，分别为理想 IMU、实测散布加偏置的 IMU。轨迹由闭环生成，不是严格相同运动轨迹的噪声消融。",
      "", "## 实际运行结果", "",
      "| 输入 | 目标距离 | LIO 位置 RMSE | LIO 姿态 RMSE | 轨迹数 | 雷达帧 |",
      "|---|---:|---:|---:|---:|---:|"]
    for name,r in results.items():
        c=r["comparison"]
        lines.append("| %s | %.2f cm | %.2f cm | %.3f° | %d | %d |"%(name,r["goal_error_m"]*100,c["position_rmse_m"]*100,c["orientation_rmse_deg"],r["plans"],r["frames"]))
    lines+=["","目标为 IMU 初始原点坐标系下 (-0.7, 0, 0.5) m。比较没有做轨迹拟合/对齐。最初约三帧为 LIO 初始化，没有里程计输出。",
            "数据为局部短距离闭环冒烟测试，不能据此宣称完整森林任务避障、实机定位或多机性能。","",
            "![闭环测试](../../../artifacts/forest_lio/phase2_summary/closed_loop.png)","","## 渲染敏感性","",
            "对相同地图、射线模板、解析轨迹，在 0/6/8 s 比较 1/5/10/20 ms 子扫描、整帧缓存、0.1/0.2 m 表面片、0.5/1°角度栅格和 0.15/0.3 m 地图体素。1 ms 每子扫描重建是数值参考，不是真实激光真值。",
            "整帧缓存运动时出现明显距离差异，因此默认 conservative 仅在帧内位移小于 1e-6 m 时复用栅格，运动时每 10 ms 重建。cached 只保留为实验选项。",
            "改变表面片、角度栅格和地图密度会改变遮挡与回波；当前模型参数尚未证明对不同森林稳健。",
            "", "| 时刻 | 方案 | 回波有无差异 | 共同回波距离差 P95 |","|---|---|---:|---:|"]
    for r in benchmark["results"]:
        if r["variant"] in ("full_raster_5ms","full_raster_10ms","cached_10ms","cached_voxel_0.3m"):
            lines.append("| %.0f s | %s | %.2f%% | %.3f m |"%(r["time_s"],r["variant"],r["hit_disagreement_fraction"]*100,r["common_hit_range_difference_m"]["p95"]))
    lines+=["","渲染耗时随地图、机器和并行负载变化。当前保守渲染及 ROS 等待使仿真慢于实时；结果文件保留逐次耗时统计。未声称达到实时 10 Hz。",
            "", "## 验证与边界", "",
            "8 项传感器/跟踪器测试通过；ego_planner_swarm 模块 catkin 编译通过；在线闭环实跑成功。各次独立 ROS master、日志和结果保存于 artifacts/forest_lio/，不覆盖原 bag/PCD。",
            "默认地图为前 30 s 原始数据回放重建的 223119 点地图。扫描方向模板仅取自主飞行训练段；同源地图与观测的测试不能替代独立数据验证。",
            "传感器仍采用点图表面片近似；未知几何缺失、枝叶透射/材质/风动未实现。点云距离检查不能把未观测区域证明为自由空间。没有碰撞响应。",
            "运动模型姿态固定，IMU 比力来自实际模拟加速度；尚无旋翼姿态动力学、飞控、PX4 SITL 或硬件在环。当前只验证单机局部闭环。",
            "", "## 下一步", "",
            "1. 优先改进渲染候选面/遮挡查询，降低敏感性，同时用性能剖析推进实时化；不以错误缓存换取实时。",
            "2. 重建更长的自主飞行正常段地图，加入已观测覆盖约束及树干/枝叶困难路线，用独立采集验证回波分布。",
            "3. 接入 SO(3) 旋翼动力学及 PX4 SITL，随后再扩展多机独立 LIO 原点、统一坐标和通信；具备这些条件后才推进硬件在环。",
            "", "运行步骤见 [工具说明](../../../Simulation/forest_lio/README.md)。"]
    (out/"report.md").write_text("\n".join(lines)+"\n")
    print(out/"report.md")
if __name__=="__main__":
    main()
