#!/usr/bin/env python3
"""Summarize normal-flight map, observation evidence and adaptive geometry experiments."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from renderer import read_pcd

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--replay",type=Path,required=True)
    ap.add_argument("--coverage",type=Path,required=True)
    ap.add_argument("--surfaces",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    replay=json.loads((args.replay/"replay_result.json").read_text())
    provenance=json.loads((args.replay/"provenance.json").read_text())
    coverage=json.loads((args.coverage/"coverage.json").read_text())
    surfaces=json.loads((args.surfaces/"surfaces.json").read_text())
    xyz=read_pcd(args.replay/"forest_map.pcd")
    origins=np.loadtxt(args.coverage/"sensor_origins.csv",delimiter=",",ndmin=2)
    sample=xyz[::max(1,len(xyz)//100000)]
    fig,ax=plt.subplots(figsize=(9,7))
    pc=ax.scatter(sample[:,0],sample[:,1],c=sample[:,2],s=.3,cmap="viridis",rasterized=True)
    ax.plot(origins[:,1],origins[:,2],color="red",lw=1,label="Estimated sensor path")
    ax.set(xlabel="LIO world x (m)",ylabel="LIO world y (m)",title="Normal-flight map and estimated sensor path")
    ax.axis("equal");ax.legend();fig.colorbar(pc,ax=ax,label="z (m)")
    fig.tight_layout();fig.savefig(args.output/"normal_map_xy.png",dpi=160);plt.close(fig)
    lines=["# 正常飞行地图、观测证据与自适应几何验证","",
      "数据来自原始 Mid-360/IMU 重放。地图与覆盖都依赖 Fast-LIO2 估计，不能作为原始飞行的独立真值。",
      "用户确认最后切 MANUAL 为主动遥控操作；本次采用保守的正常飞行窗口，不把尾段解释成导航错误。","",
      "## 地图与覆盖","",
      "- 来源窗口：原 bag 起点后 %s s；从首个原始记录启动 LIO 初始化，仅窗口内配准点积累进导出地图。"%provenance["map_window_relative_to_bag_s"],
      "- 回放状态：%s；输入 %d 帧，LIO 输出 %d，配准点云 %d。"%(replay["status"],replay["expected_lidar_frames"],replay["estimated_odom_messages"],replay["registered_cloud_messages"]),
      "- 地图：%d 点，体素 %.2f m；XYZ 最小 %s，最大 %s。"%(len(xyz),provenance["map_voxel_m"],np.round(xyz.min(axis=0),2).tolist(),np.round(xyz.max(axis=0),2).tolist()),
      "- 末帧时间差 %.6f s；回调错误 %s。"%(replay["last_stamp_difference_s"],replay["callback_errors"]),
      "- 覆盖处理 %d 帧、%d 条抽样实际回波射线；时间匹配跳过 %d 帧。"%(coverage["frames"],coverage["selected_rays"],coverage["skipped_pose_mismatch"]),
      "- 稀疏证据体素 %d，边长 %.2f m；射线经过 %d、实际端点 %d、传感器位置 %d（标志可重叠）。"%(coverage["cells"],coverage["voxel_m"],coverage["ray_trace_cells"],coverage["surface_endpoint_cells"],coverage["sensor_position_cells"]),
      "- 覆盖生成耗时 %.2f s；每帧最多 %d 条射线，量程上限 %.1f m。"%(coverage["wall_s"],coverage["max_rays_per_scan"],coverage["max_range_m"]),
      "",
      "coverage.npz 标志位：1=射线中心线经过证据，2=实际回波端点，4=传感器位置；不存在的体素=未知。**没有任何标志表示整格或无人机体积已确认可通行。**",
      "使用扫描末端位姿近似反畸变后的射线起点；逐点原始起点、无回波方向与未抽样射线未被补造。遮挡后方仍未知。覆盖目前用于评估，不注入规划器作为自由空间。",
      "",
      "![正常地图与估计轨迹](normal_map_xy.png)","",
      "## 已知几何验证","",
      "| 场景 | 模型 | 误命中率 | 漏命中率 | 同为真命中距离误差 P95 |",
      "| --- | --- | ---: | ---: | ---: |"]
    for row in surfaces["known_geometry"]:
        lines.append("| %s | %s | %.2f%% | %.2f%% | %.4f m |"%(row["scene"],row["model"],100*row["false_positive_fraction"],100*row["false_negative_fraction"],row["common_true_hit_error_m"]["p95"]))
    lines += ["","误命中率以真实无命中射线为分母，漏命中率以真实命中射线为分母。孔洞场景都有后墙，必须另看前后遮挡误判：", ""]
    for row in surfaces["known_geometry"]:
        if "incorrect_front_back_fraction" in row:
            lines.append("- %s：前后墙误判 %.2f%%。"%(row["model"],100*row["incorrect_front_back_fraction"]))
    lines += ["","自适应支持范围由局部间距、PCA 平面性与切向分布确定；平面可信点使用有限圆盘，法向不可靠点保留为小球障碍代理并标记不确定。不能据此恢复未采集的物理表面。",
      "混合球/圆盘、不同逐点半径已与独立穷举求交参考核对；原生 BVH 仍逐射线求最近交点，不缓存旧位姿遮挡。","",
      "## 完整地图敏感性","",
      "相同 4000 条射线；每个模型分别与自身基准比较。此处仅为单一初始原点的局部诊断，不是全地图航线或真实距离精度验证。", "",
      "| 模型 | 变体 | 点数 | 有无回波变化 | 共同命中距离差 P95 | 查询 P95 |",
      "| --- | --- | ---: | ---: | ---: | ---: |"]
    for row in surfaces["forest"]:
        lines.append("| %s | %s | %d | %.2f%% | %.3f m | %.2f ms |"%(row["model"],row["variant"],row["points"],100*row["hit_disagreement_fraction"],row["common_hit_difference_m"]["p95"],1000*row["query_s"]["p95"]))
    adaptive=next(r for r in surfaces["forest"] if r["model"]=="adaptive" and r["variant"]=="baseline")["surface_model"]
    lines += ["",
      "自适应基准：%d 个圆盘、%d 个不确定小球；半径中位数 %.3f m、P95 %.3f m。"%(adaptive["disk_points"],adaptive["uncertain_sphere_points"],adaptive["radii_m"]["median"],adaptive["radii_m"]["p95"]),
      "**已知几何的障碍膨胀改善不等于森林点云降采样鲁棒性已解决。** 默认仍为 fixed；adaptive 需显式选择。粗化会改变邻域、支持半径和遮挡，缺失几何不能通过索引性能优化修复。",
      "",
      "## 复现与下一步","",
      "命令见模块 README 的“正常飞行地图与观测证据”。参数和原始结果分别保存在以下目录：","",
      "- 回放：%s"%args.replay,
      "- 覆盖：%s"%args.coverage,
      "- 几何：%s"%args.surfaces,
      "",
      "已在此前局部地图上完成 adaptive 12 s LIO/EGO 冒烟闭环；完整新地图尚未完成长航线闭环或全地图实时性能验证。",
      "下一步按实测轨迹建立局部仿真初始坐标，选择有覆盖证据的路径，并评估机体范围内未知区域；然后再做较长轨迹与运动姿态测试。"]
    (args.output/"report.md").write_text("\n".join(lines)+"\n")
    print(args.output/"report.md")

if __name__=="__main__":main()
