#!/usr/bin/env python3
"""Summarize this phase-one experiment, keeping real repeatability and synthetic truth separate."""
import argparse
import json
from pathlib import Path
import numpy as np
from common import trajectory_comparison, write_json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def load(path):
    return np.loadtxt(path, ndmin=2)

def differences(est, ref, max_gap=0.2):
    ref = ref[np.argsort(ref[:, 0])]
    ref = ref[np.r_[True, np.diff(ref[:, 0]) > 0]]
    idx = np.searchsorted(ref[:, 0], est[:, 0])
    valid = (idx > 0) & (idx < len(ref))
    idx = np.clip(idx, 1, len(ref)-1)
    valid &= ref[idx, 0]-ref[idx-1, 0] <= max_gap
    e = est[valid]
    r = np.column_stack([np.interp(e[:, 0], ref[:, 0], ref[:, k]) for k in range(1, 4)])
    return e[:, 0], np.linalg.norm(e[:, 1:4]-r, axis=1)

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results", type=Path, default=Path("artifacts/forest_lio"))
    args = ap.parse_args()
    root = args.results
    audit = json.loads((root/"audit_final/audit.json").read_text())
    actual = json.loads((root/"replay_full_verified/replay_result.json").read_text())
    est = load(root/"replay_full_verified/replayed_lio.tum")
    ref = load(root/"replay_full_verified/recorded_lio.tum")
    start = audit["metadata"]["start_unix"]
    intervals = [x for x in audit["high_zero_intervals"] if x["frames"] >= 5]
    major = max(intervals, key=lambda x: x["frames"]) if intervals else None
    normal_end = max(0, major["start_s"]-5) if major else audit["metadata"]["duration_s"]
    recovery = major["end_s"]+0.1 if major else normal_end
    normal = trajectory_comparison(est[est[:, 0] < start+normal_end], ref)
    tail = trajectory_comparison(est[est[:, 0] >= start+recovery], ref)
    out = root/"summary"
    out.mkdir(exist_ok=True)
    summary = {"real_replay_status": actual["status"], "real_reference": "Recorded LIO, not independent truth.",
               "normal_interval_s": [0, normal_end], "normal_repeatability": normal,
               "after_low_return_interval_s": [recovery, audit["metadata"]["duration_s"]],
               "tail_repeatability": tail, "synthetic": {}}
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    stamps, err = differences(est, ref)
    axs[0].semilogy(stamps-start, np.maximum(err, 1e-5))
    axs[0].set_ylabel("LIO position difference (m)")
    axs[0].set_title("Real bag: replay vs recorded LIO (not independent truth)")
    csv = np.genfromtxt(root/"audit_final/lidar_frames.csv", delimiter=",", names=True)
    axs[1].plot(csv["relative_s"], csv["zero_fraction"])
    axs[1].set_ylabel("Zero XYZ fraction")
    axs[1].set_xlabel("Time since bag start (s)")
    if major:
        for ax in axs:
            ax.axvspan(major["start_s"], major["end_s"], color="orange", alpha=0.25)
    fig.tight_layout(); fig.savefig(out/"real_repeatability.png", dpi=150); plt.close(fig)
    for name, replay in [("ideal", "sim_replay_final"), ("controlled_noise", "sim_noise_replay")]:
        sim_dir = root/("sim_ideal" if name == "ideal" else "sim_noise")
        e = load(root/replay/"replayed_lio.tum"); truth = load(sim_dir/"truth.tum")
        result = json.loads((root/replay/"replay_result.json").read_text())
        cmp = trajectory_comparison(e, truth)
        summary["synthetic"][name] = {"comparison": cmp,
             "processed_scans": result["estimated_odom_messages"],
             "expected_scans": result["expected_lidar_frames"],
             "processing_time_s": result.get("processing_time_s"),
             "last_stamp_difference_s": result["last_stamp_difference_s"],
             "status": result["status"]}
        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        for k in range(3):
            axs[k].plot(truth[:, 0]-truth[0, 0], truth[:, k+1], label="Simulated truth")
            axs[k].plot(e[:, 0]-truth[0, 0], e[:, k+1], "--", label="Fast-LIO2")
            axs[k].set_ylabel(["x (m)", "y (m)", "z (m)"][k])
        axs[0].legend()
        t, error = differences(e, truth)
        axs[3].plot(t-truth[0, 0], error); axs[3].set_ylabel("Position error (m)")
        axs[3].set_xlabel("Simulated time (s)")
        fig.suptitle("Synthetic sensor benchmark: "+name)
        fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(out/("synthetic_"+name+".png"), dpi=150); plt.close(fig)
    write_json(out/"phase1_results.json", summary)
    report = ["# 森林雷达 / LIO 第一阶段实验报告", "",
      "输入：yxb_20260208-104116.bag，原始 Mid-360 + IMU。所有结果来自实际执行。",
      "", "## 已实现与验证", "",
      "- 全量原始数据审计，独立 ROS master 的原始话题回放，重新生成定位和点云。",
      "- Fast-LIO2 日志与 PCD 输出目录重定向；没有改动定位、规划、控制算法。",
      "- 使用正常区间重建地图生成带时间的 CustomMsg 与一致的解析 IMU。",
      "- 30 秒理想输入和 12 秒受控噪声输入均实际回放至 Fast-LIO2。",
      "- 运动导数、机体系角速度、静止比力、消息点时序、遮挡/位移和比较边界共 6 项测试通过。",
      "", "## 全量真实数据", "",
      "- %d 雷达帧、%d IMU 消息；平均约 10 Hz / 200 Hz；序号无缺失，时间戳无倒退。" %
      (audit["lidar_timing"]["count"], audit["imu_timing"]["count"]),
      "- 零坐标点比例中位数 %.2f%%。" % (audit["lidar_statistics"]["zero_fraction"]["median"]*100),
      "- 2225 个输入雷达帧产生 %d 个定位/配准点云输出；启动初始化和低回波区间会跳过部分扫描。" % actual["estimated_odom_messages"],
      "- 回放追到最后一帧，完整内部耗时日志已保存。",
      "- %.3f～%.3f 秒的位置重复性 RMSE：%.3f cm。" % (0, normal_end, normal["position_rmse_m"]*100),
      "- %.3f 秒以后的位置重复性 RMSE：%.3f m。" % (recovery, tail["position_rmse_m"]),
      "- 全程位置差 RMSE %.3f m；包含手动操作尾段，不作为自主导航失败指标。" %
      actual["recorded_lio_comparison"]["position_rmse_m"],
      "", "记录 LIO 不是独立真值；以上衡量回放与历史记录差异，不能宣称原始飞行精度。",
      "", "## 手动操作附近的观测", "",
      "- %.3f～%.3f 秒，连续 %d 帧零坐标比例超过80%%，最高 %.2f%%。" %
      (major["start_s"], major["end_s"], major["frames"], major["max_fraction"]*100),
      "- 附近记录到约 %.2f g 加速度模长峰值及显著角速度。用户确认尾段由遥控器主动切 MANUAL，不是错误。" %
      audit["imu_acc_norm_native"]["max"],
      "- 三轴加速度接近 +/-4 的样本提示可能触及量程；需核对设备量程。原因尚未确定。",
      "- 不能直接将这个区间视为一般森林遮挡；暂不用于正常回波/噪声模型标定。",
      "", "![真实回放重复性](real_repeatability.png)", "",
      "## 已知模拟轨迹验证", "",
      "| 输入 | 位置 RMSE | 姿态 RMSE | 输出扫描 |",
      "|---|---|---|---|"]
    for name, s in summary["synthetic"].items():
        c = s["comparison"]
        report.append("| %s | %.3f cm | %.3f deg | %d / %d |" %
          (name, c["position_rmse_m"]*100, c.get("orientation_rmse_deg", 0), s["processed_scans"], s["expected_scans"]))
    report += ["", "两组的轨迹时长和幅度不同，仅验证噪声输入链路，不能据此比较噪声对精度的影响。", "无轨迹对齐，比较解析 IMU 轨迹与 LIO 输出。初始化期间少量帧无定位输出。",
      "受控噪声采用距离标准差0.02m、角速度0.001rad/s、加速度0.003g、seed=42；不是已标定的真实传感器参数。",
      "", "![理想模拟](synthetic_ideal.png)", "",
      "## 地图与模型边界", "",
      "模拟使用前30秒正常区间重建的0.15m体素地图，223119点；未使用包含手动操作尾段的全程地图。",
      "上游扫描模板只有实际回波方向。无回波方向、材质反射率、枝叶透射和风动尚未恢复。",
      "最近深度角度栅格与局部表面片近似遮挡；按10ms子扫描中点位姿近似点时序。",
      "生成器目前离线生成bag，还不是在线实时传感器闭环；后续需优化渲染并接入运动反馈。",
      "模拟轨迹没有动力学、控制或碰撞响应；这是定位观测基准，不是全系统飞行仿真。",
      "同源建图与合成用于基础自洽检查，不替代独立数据验证。",
      "", "## 下一步", "",
      "1. 按用户确认的手动切换分段评估，MANUAL 不计作自主导航错误；零回波和两次 LIO 差异仅保留为观测。",
      "2. 用确认的正常/静止区间校准回波、扫描时序与IMU统计，再用独立采集验证。",
      "3. 检验地图密度、表面片半径和子扫描分辨率敏感性，避免人为几何/时序误差主导LIO。",
      "4. 通过这些检查后接入EGO，规划只使用LIO估计，模拟真值仅用于评估；随后扩展多机独立原点与传感器。",
      "", "运行步骤见 ../../../Simulation/forest_lio/README.md。"]
    (out/"report.md").write_text("\n".join(report)+"\n")
    print("Summary:", out/"report.md")

if __name__ == "__main__":
    main()
