#!/usr/bin/env python3
"""Save measured renderer/closed-loop results with explicit model limitations."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def main():
    root=Path(__file__).resolve().parents[3];base=root/"artifacts/forest_lio"
    profile=json.loads((base/"bvh_profile_opaque/profile.json").read_text())
    runs={name:json.loads((base/folder/"result.json").read_text()) for name,folder in
          [("理想 IMU","bvh_closed_loop_opaque"),("实测散布与偏置","bvh_closed_loop_opaque_noise")]}
    analytic=json.loads((base/"bvh_analytic_opaque_replay/replay_result.json").read_text())
    out=base/"bvh_summary";out.mkdir(exist_ok=True)
    fig,axs=plt.subplots(2,1,figsize=(10,7))
    for threads in (1,2,4):
        rows=[p for p in profile["profiles"] if p["threads"]==threads]
        axs[0].plot([p["time_s"] for p in rows],[p["frame_s"]["p95"]*1000 for p in rows],
                    marker="o",label=f"BVH {threads} threads")
    axs[0].axhline(100,linestyle="--",color="gray",label="100 ms / 10 Hz budget")
    axs[0].set_xlabel("Analytic scan time (s)");axs[0].set_ylabel("Isolated render P95 (ms)")
    for label,folder in [("Ideal","bvh_closed_loop_opaque"),("IMU scatter + bias","bvh_closed_loop_opaque_noise")]:
        tr=np.loadtxt(base/folder/"truth.tum",ndmin=2)
        r=json.loads((base/folder/"result.json").read_text())
        axs[1].plot(tr[:,0]-tr[0,0],np.linalg.norm(tr[:,1:4]-r["goal"],axis=1),label=label)
    axs[1].set_xlabel("Simulation time (s)");axs[1].set_ylabel("Truth distance to goal (m)")
    for ax in axs:ax.grid(alpha=.3);ax.legend()
    fig.suptitle("World-space surfel BVH: measured query speed and local closed loop")
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/"bvh_results.png",dpi=150);plt.close(fig)
    lines=["# 森林雷达渲染：候选面查询与性能验证","",
      "本轮不需要原飞行视频或额外飞控日志。使用已知几何测试、模型内独立求交参考和森林 PCD 对照。","",
      "## 实现","",
      "- 新增 C++/OpenMP 世界坐标 BVH（包围盒层次索引），Python 通过 ctypes 调用。",
      "- 为每个有限半径面片建立保守包围盒，沿每条射线搜索可能相交的面片；精确计算射线与面片交点并取最近有效值。",
      "- 不再每角度格子只保留一个中心点，因此不会因最近中心点的面片未命中而丢掉同格内其他候选面。",
      "- 固定地图索引可长期复用；射线原点和方向按当前采样位姿更新，不缓存旧遮挡。地图改变时需要重新构建索引。",
      "- 默认生成器/闭环使用 --renderer bvh，--render-threads 控制线程数；旧 raster/conservative/full/cached 保留用于对照。",
      "- 离线生成器按解析轨迹计算每个点的位姿；固定姿态在线模型按 200 Hz 位置历史为每个点插值。在线插值仍是运动离散近似。",
      "- 仿真时钟在下一 IMU 样本之前的小时间窗口内推进，让 ROS 节点及时唤醒；不把旧的空等计为渲染耗时。","",
      "盲区门限在找到最近不透明交点后应用：近障即使在输出盲区内，也会挡住远处面片。",
      "## 已知几何与模型内正确性","",
      "16 项测试通过，包含原传感器/运动模型测试及新增平面前后遮挡、孔洞、薄圆柱、盲区/量程、逐射线位姿/旋转、随机面片、多线程和旧位姿遮挡失效测试。",
      "薄圆柱测试使用解析圆柱交点作参考，距离差最大值要求低于 2 mm；这是构造场景的几何验证，不是森林传感器精度。",
      "对森林地图 0/6/8 s 各抽查 96 条射线，与独立 NumPy 全部面片求交参考一致，最大绝对差低于 1e-8 m。",
      "该参考只证明固定面片模型内的候选查询/遮挡正确，不能证明面片就是现实树枝或枝叶。","",
      "## 性能剖析","",
      f"地图 {profile['map_points']} 点，每帧 {profile['ray_count']} 条射线，每条件预热后重复 {profile['repeats']} 次。角度模板取原 bag 40–120 s。",
      "CPU 单机测试，OPENBLAS_NUM_THREADS=1；时间包含逐点位姿/方向构造、Python 输入整理和原生求交，不含 ROS、LIO、EGO。",
      f"一次性 KD/PCA 预处理 {profile['preprocess_s']:.3f} s，BVH 构建 {profile['index_build_s']:.3f} s；首次 C++ 编译额外耗时未包含。",
      "", "| 扫描时刻 | 线程 | 渲染中位数 | 渲染 P95 |","|---|---:|---:|---:|"]
    for p in profile["profiles"]:
        lines.append("| %.0f s | %d | %.2f ms | %.2f ms |"%(p["time_s"],p["threads"],p["frame_s"]["median"]*1000,p["frame_s"]["p95"]*1000))
    lines+=["","旧方法每 10 ms 子扫描重新投影一遍地图，三次整帧耗时 %.1f/%.1f/%.1f ms；这里只记录各一次耗时，不作为严格重复测量的加速比。"%
      tuple(c["legacy_full_10ms_wall_s"]*1000 for c in profile["checks"]),
      "完整剖析记录了位姿变换、输入整理、原生求交、AABB 测试和面片测试。稳态瓶颈主要在原生查询，而不是重复地图投影。",
      "", "## 森林 PCD 对照","",
      "旧方法与同点时刻的全部面片求交参考相比，抽查回波有无不一致比例约 %.1f%%–%.1f%%。旧方法可能漏掉近处面片而返回后面的面片；这些数值不代表真实激光误差。"%
      (min(c["legacy_matched_time_oracle_hit_disagreement"] for c in profile["checks"])*100,
       max(c["legacy_matched_time_oracle_hit_disagreement"] for c in profile["checks"])*100),
      "新索引消除了角度格子和旧位姿缓存导致的候选漏查。但几何建模敏感性仍然存在：","",
      "| 改变模型几何 | 回波有无变化 | 共同回波距离差 P95 |","|---|---:|---:|"]
    for row in profile["geometry_sensitivity"]:
        lines.append("| %s | %.2f%% | %.3f m |"%(row["variant"],row["hit_disagreement_fraction"]*100,row["common_hit_abs_difference_m"]["p95"]))
    lines+=["","更小面片会暴露地图孔洞，粗体素也会改变法向和遮挡关系。当前不能宣称已经解决面片半径/密度敏感性；下一步需改进局部表面支持和观测覆盖表示，配合独立采集判断物理合理性。",
            "", "扫描模板只含实际回波方向，未恢复无回波射线；反射率仍为模板代理。枝叶透射、材质与风动未建模，同源 PCD 合成不能替代独立数据验证。","",
            "## LIO / EGO 实际验证","",
            "| IMU 输入 | 状态 | LIO 位置 RMSE | 最终距目标 | 渲染 P95 | 主循环墙钟时间 / 12 s 仿真 |",
            "|---|---|---:|---:|---:|---:|"]
    for name,r in runs.items():
        lines.append("| %s | %s | %.2f cm | %.2f cm | %.2f ms | %.2f s |"%
          (name,r["status"],r["comparison"]["position_rmse_m"]*100,r["goal_error_m"]*100,
           r["lidar_render_wall_s"]["p95"]*1000,r["loop_wall_s"]))
    c=analytic["synthetic_truth_comparison"]
    lines+=["","两次闭环均为同一局部目标的 120 帧测试，LIO/EGO 只读取估计定位和观测点云。真值仅用于评估。",
            "主循环约接近实时；启动、建图索引及进程关闭另有耗时。调度时延和其他场景尚未验证，不承诺硬实时或硬件在环。",
            "另有 12 s 带转弯/升降的解析轨迹输入实际回放 Fast-LIO2：位置 RMSE %.2f cm，姿态 RMSE %.3f°，输出 %d/%d 帧。"%
            (c["position_rmse_m"]*100,c["orientation_rmse_deg"],analytic["estimated_odom_messages"],analytic["expected_lidar_frames"]),
            "", "![测量结果](../../../artifacts/forest_lio/bvh_summary/bvh_results.png)","",
            "## 构建与运行","",
            "原生内核由首次 BVH 调用使用 g++ -O3 -std=c++14 -shared -fPIC -fopenmp 自动构建，产物按源码散列缓存在 artifacts/forest_lio/native/。",
            "需要支持 OpenMP 的 g++，不需要 GPU 或新增 catkin 包。完整 ./compile.sh 的六个模块均通过。","",
            "在项目根目录加载 ROS、Livox 和项目环境后：","",
            "```bash",
            "OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 Simulation/run.py test",
            "OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 Simulation/run.py profile \\",
            "  --map artifacts/forest_lio/replay_smoke/forest_map.pcd \\",
            "  --bag ~/bagfiles/yxb_20260208-104116.bag \\",
            "  --output artifacts/forest_lio/my_bvh_profile",
            "```","",
            "运行命令及边界见 [模块说明](../../../Simulation/forest_lio/README.md)。原始 bag/PCD 未覆盖，结果保存在 artifacts/forest_lio/。"]
    (out/"report.md").write_text("\n".join(lines)+"\n")
    print(out/"report.md")
if __name__=="__main__":main()
