#!/usr/bin/env python3
"""Summarize route tests in their common recorded-world frame."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scene import to_world

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene",type=Path,required=True)
    ap.add_argument("--runs",type=Path,nargs="+",required=True)
    ap.add_argument("--baseline-failure",type=Path)
    ap.add_argument("--output",type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    scene=json.loads(a.scene.read_text());origin=np.asarray(scene["origin_world"]);R=np.asarray(scene["rotation_world_from_local"])
    route=to_world(np.vstack([np.zeros(3),scene["goals_local"]]),origin,R)
    fig,ax=plt.subplots(figsize=(10,5))
    ax.plot(route[:,0],route[:,1],"k--o",ms=3,label="Estimated-route goals")
    rows=[];distances=[]
    for path in a.runs:
        result=json.loads((path/"result.json").read_text());rows.append((path,result))
        truth=np.loadtxt(path/"truth_world.tum",ndmin=2)
        distances.append((result["parameters"]["surface_model"],float(np.linalg.norm(np.diff(truth[:,1:4],axis=0),axis=1).sum())))
        lio=np.loadtxt(path/"lio.tum",ndmin=2)
        est=to_world(lio[:,1:4],origin,R)
        label=result["parameters"]["surface_model"]
        ax.plot(truth[:,1],truth[:,2],label=label+" simulated path")
        ax.plot(est[:,0],est[:,1],":",lw=1,label=label+" LIO estimate")
    ax.set(xlabel="Original LIO world x (m)",ylabel="Original LIO world y (m)",title="Online route in estimated forest map")
    ax.axis("equal");ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(a.output/"route_xy.png",dpi=160);plt.close(fig)
    lines=["# 完整森林地图中的分段路线闭环","",
      "本次在正常飞行段生成的百万点地图上执行约 12 m 路线；导航只接收模拟雷达和 IMU，通过 LIO、EGO 和二阶位置跟踪器形成反馈。",
      "完整地图留在传感器渲染进程中，不发布为 EGO 全局障碍地图。原飞行轨迹只用于预先定义任务目标，不作为在线定位或控制指令输入。","",
      "## 坐标与任务","",
      "- 来源：原 bag 后 %s s 的新回放 LIO 轨迹；这是估计轨迹，不能证明原森林飞行精度。"%scene["source_window_s"],
      "- 参考路径 %.3f m；分段目标连线 %.3f m，共 %d 个目标。"%(scene["reference_path_length_m"],scene["goal_polyline_length_m"],len(scene["goals_local"])),
      "- 仿真原点在原地图坐标 %s m；保留初始航向 %.3f 度。"%(np.round(origin,4).tolist(),scene["source_initial_rpy_deg"][2]),
      "- 原始初始横滚/俯仰 %.3f/%.3f 度被水平姿态替代。局部变换保持重力方向，雷达/IMU 外参也按同一局部姿态使用。"%tuple(scene["source_initial_rpy_deg"][:2]),
      "- 目标切换仅使用 LIO 估计位置；中间目标进入 0.25 m 范围后切换，最终目标 0.20 m，最后稳定 3 s。",
      "- 预选参考连线的地图点最小距离 %.3f m；机体评估使用 %.2f m 半径球代理。"%(scene["minimum_reference_point_clearance_m"],scene["body_radius_m"]),
      "",
      "![路线与估计轨迹](route_xy.png)","",
      "## 实跑结果","",
      "| 模型 | 状态 / 目标到达 | 时长 / 雷达帧 | 规划轨迹数 | LIO 位置 RMSE | 最终目标误差 | 最小地图点距离 |",
      "| --- | --- | --- | ---: | ---: | ---: | ---: |"]
    for path,r in rows:
        lines.append("| %s | %s / %d | %.1f s / %d | %d | %.3f m | %.3f m | %.3f m |"%(r["parameters"]["surface_model"],r["status"],len(r["goal_arrivals"]),r["sim_duration_s"],r["frames"],r["plans"],r["comparison"].get("position_rmse_m",float("nan")),r["goal_error_m"],r["minimum_sampled_truth_point_clearance_m"]))
    lines += ["", "实际模拟路径长度："+"；".join("%s %.3f m"%(name,d) for name,d in distances)+"。"]
    lines += ["","上述 RMSE 比较的是模拟固定地图世界中的已知位移，不是原飞行真值。目标到达是本次路线测试条件，不代表真实森林飞行性能达标。","",
      "## 机体周围未知区域","",
      "每 0.1 s 在原地图坐标评估球代理与覆盖体素的相交情况，包含边界相交体素；只用于评估，不修改规划地图或运动轨迹。","",
      "| 模型 | 评估样本 | 中心未知 | 机体范围含未知 | 未知体素比例 P95 | 含回波端点证据样本 |",
      "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for path,r in rows:
        e=r["footprint_evidence"]
        lines.append("| %s | %d | %d | %d | %.2f%% | %d |"%(r["parameters"]["surface_model"],e["samples"],e["center_unknown_samples"],e["footprint_has_unknown_samples"],100*e["unknown_cell_fraction"]["p95"],e["footprint_endpoint_evidence_samples"]))
    lines += ["",
      "**未知数为零仅表示相交体素具有某种抽样观测证据，不能证明整个体素或机体 swept volume 已确认可通行。** 含端点证据也不是精确碰撞结论，体素化、扫描末端起点近似和 LIO 漂移均会影响结果。",
      "",
      "## 耗时与边界","",
      "| 模型 | 单帧渲染 P95 | 闭环墙钟时长 | 仿真时长 / 墙钟 |",
      "| --- | ---: | ---: | ---: |"]
    for path,r in rows:
        lines.append("| %s | %.2f ms | %.2f s | %.3f |"%(r["parameters"]["surface_model"],1000*r["lidar_render_wall_s"]["p95"],r["loop_wall_s"],r["simulation_to_wall_ratio"]))
    lines += ["",
      "墙钟时长不含索引与模板加载、节点启动和结果保存。没有为加速缓存旧位姿遮挡；每条射线使用点时刻插值位置重新求交。",
      "EGO 的 waypointCallback 当前包含紧急停止及 2 s 的 sleep 阻塞；这会影响多目标切换行为。尚未剖析它占完整墙钟时长的比例，不能只用渲染耗时解释闭环性能。",
      "仍为单机、零噪声、固定姿态的二阶位置反馈模型；没有旋翼动力学、碰撞接触、PX4 SITL 或硬件在环。本路线经过事先有观测证据的区域，未验证主动探索未知森林。",
      "自适应几何减少部分已知场景误遮挡，但粗体素敏感性仍存在；两种表面模型的闭环结果不能替代实测激光真实性验证。",
      "",
      "## 复现与后续","",
      "入口：prepare-scene 与 closed-loop --scene；完整命令见模块 README。结果目录：",""]
    lines += ["- %s"%p for p,_ in rows]
    if a.baseline_failure:
        failed=json.loads((a.baseline_failure/"result.json").read_text())
        lines += ["", "## 原局部配置的边界检查", "",
          "原 20 m × 20 m 地图覆盖 XY ±10 m，后两个参考目标超过 X 上界。日志显示第 8 个目标被重定位到约 (10.025, -2.125, 0.025) m；原目标没有到达。",
          "该运行保留为失败：到达 %d/%d 个目标，最终原目标误差 %.3f m，结果目录 %s。"%(len(failed["goal_arrivals"]),len(failed["goals_local"]),failed["goal_error_m"],a.baseline_failure),
          "scene 入口现按任务范围和局部更新范围扩展 XY 地图，保存实际参数，并在启动前检查目标/机体代理是否超出地图与垂直边界。未修改 EGO 障碍判断或把重定位当作原目标到达。"]
    lines += ["","后续先剖析目标切换与 ROS 等待的耗时，再加入变化的姿态与一致的角速度/比力，验证转弯、加减速时的 LIO；然后接动力学或 PX4 SITL。"]
    (a.output/"report.md").write_text("\n".join(lines)+"\n")
    print(a.output/"report.md")
if __name__=="__main__":main()
