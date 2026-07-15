#!/usr/bin/env python3
# ============================================================================
# 方案C: 离线评估 tree_pose_error 对邻居位置的修正效果
#
# 核心问题：应用 swarm_controller 修正公式后，邻居位置是否更接近真值？
#
# 原理:
#   1. 从方案B rosbag 读取 tree_pose_error + odom（真值）
#   2. 用 eval_config.json 中的注入漂移模拟 "raw neighbor position"
#      （模拟邻居的里程计漂移导致其报告的 world-frame 位置有偏差）
#   3. 应用与 swarm_controller::drift_correct_pos_nei 完全一致的修正公式
#   4. 对比 |corrected - gt| vs |raw - gt|
#
# swarm_controller 修正公式 (drift_correct_pos_nei):
#   c = cos(est_yaw), s = sin(est_yaw)
#   corr_x =  c * (raw_x - est_dx) + s * (raw_y - est_dy)
#   corr_y = -s * (raw_x - est_dx) + c * (raw_y - est_dy)
#   即: corrected = R(yaw) · pos + t
#
# swarm_controller 订阅关系:
#   /uav{self}/tree_drift_sub[{nei}] ← /uav{nei}/tree_pose_error
#   即: UAV1 用 /uav2/tree_pose_error 修正 UAV2 的报告位置
#
# 输入:
#   方案B 的 rosbag (含 tree_pose_error, odom)
#   eval_config.json (含注入漂移参数)
#
# 输出:
#   <日志目录>/correction_offline_report.pdf
#   <日志目录>/correction_offline_summary.json
#   <日志目录>/correction_offline_per_frame.csv
#
# 用法:
#   python3 evaluate_correction_offline.py <日志目录>
# ============================================================================

import sys
import os
import json
import csv
import math
import argparse
from collections import namedtuple

import numpy as np

# ---------------------------------------------------------------------------
# matplotlib 配置
# ---------------------------------------------------------------------------
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    _MPL = True
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 150
    import logging
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
except ImportError:
    _MPL = False

# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
CorrRecord = namedtuple('CorrRecord', [
    't', 'observer_id', 'neighbor_id',
    # 真值
    'gt_self_x', 'gt_self_y',
    'gt_nei_x', 'gt_nei_y',
    'gt_rel_x', 'gt_rel_y',
    # 注入漂移 (模拟的里程计漂移)
    'drift_nei_dx', 'drift_nei_dy', 'drift_nei_yaw',
    # raw = 邻居报告的漂移位置
    'raw_nei_x', 'raw_nei_y',
    # tree_pose_error 估计
    'est_dx', 'est_dy', 'est_yaw',
    # 修正后
    'corr_nei_x', 'corr_nei_y',
    # 误差
    'err_raw_trans', 'err_raw_dx', 'err_raw_dy',
    'err_corr_trans', 'err_corr_dx', 'err_corr_dy',
    # 改进
    'improvement_pct', 'correct_direction',
])

C_BLUE   = '#2196F3'
C_ORANGE = '#FF9800'
C_GREEN  = '#4CAF50'
C_RED    = '#F44336'
C_GREY   = '#9E9E9E'
C_PURPLE = '#9C27B0'


# ---------------------------------------------------------------------------
# Rosbag I/O
# ---------------------------------------------------------------------------
def _require_rosbag():
    try:
        import rosbag
        return rosbag
    except ImportError:
        print("ERROR: rosbag 模块未找到。请先 source ROS 环境:")
        print("  source /opt/ros/noetic/setup.bash")
        print("  source devel/setup.bash")
        sys.exit(1)


def extract_tree_pose_error(bag_path, topic):
    """提取 tree_pose_error (geometry_msgs/Vector3: x=dx, y=dy, z=yaw)."""
    rosbag = _require_rosbag()
    samples = []
    with rosbag.Bag(bag_path) as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            samples.append({
                't':   t.to_sec(),
                'dx':  msg.x,
                'dy':  msg.y,
                'yaw': msg.z,
            })
    return sorted(samples, key=lambda s: s['t'])


def extract_odometry(bag_path, topic):
    """提取里程计 (x, y)."""
    rosbag = _require_rosbag()
    samples = []
    with rosbag.Bag(bag_path) as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            samples.append({
                't': t.to_sec(),
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
            })
    return sorted(samples, key=lambda s: s['t'])


# ---------------------------------------------------------------------------
# 时间对齐
# ---------------------------------------------------------------------------
def nearest_sample(samples, query_t, max_dt=0.5):
    if not samples:
        return None
    times = [s['t'] for s in samples]
    lo, hi = 0, len(times)
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < query_t:
            lo = mid + 1
        else:
            hi = mid
    best, best_dt = None, float('inf')
    for i in (lo - 1, lo):
        if 0 <= i < len(samples):
            dt = abs(samples[i]['t'] - query_t)
            if dt < best_dt:
                best_dt = dt
                best = samples[i]
    return best if best_dt <= max_dt else None


# ---------------------------------------------------------------------------
# 核心: 模拟 swarm_controller 修正链
# ---------------------------------------------------------------------------
def apply_drift_to_position(x, y, dx, dy, yaw):
    """对位置施加 2D 刚体变换漂移: pos' = R(yaw) * pos + (dx, dy).
    模拟: 里程计漂移后，无人机报告的 world-frame 位置。
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * x - s * y + dx,
        s * x + c * y + dy,
    )


def apply_swarm_correction(raw_x, raw_y, est_dx, est_dy, est_yaw):
    """应用 swarm_controller::drift_correct_pos_nei 的修正公式:
    corrected = R(yaw) * pos + t
    等价于:
      corr_x =  cos(yaw)*raw_x - sin(yaw)*raw_y + dx
      corr_y =  sin(yaw)*raw_x + cos(yaw)*raw_y + dy
    """
    c = math.cos(est_yaw)
    s = math.sin(est_yaw)
    return (
        c * raw_x - s * raw_y + est_dx,
        s * raw_x + c * raw_y + est_dy,
    )


def build_records(tree_pose_samples, odom_observer, odom_neighbor,
                  drift_nei, observer_id, neighbor_id, max_dt):
    """构建逐帧评估记录.

    模拟场景: observer 从 neighbor 收到 DroneState（已漂移的位置），
    用 neighbor 发布的 tree_pose_error 修正 neighbor 的位置。

    Args:
        tree_pose_samples: neighbor 的 tree_pose_error 估计值列表
        odom_observer: observer 的里程计 (真值)
        odom_neighbor: neighbor 的里程计 (真值)
        drift_nei: (dx, dy, yaw_rad) — neighbor 的模拟里程计漂移
    """
    records = []
    drift_dx, drift_dy, drift_yaw = drift_nei
    has_drift = abs(drift_dx) > 1e-9 or abs(drift_dy) > 1e-9 or abs(drift_yaw) > 1e-9

    if not has_drift:
        return records  # 无漂移无需评估

    for sample in tree_pose_samples:
        gt_self = nearest_sample(odom_observer, sample['t'], max_dt)
        gt_nei  = nearest_sample(odom_neighbor,  sample['t'], max_dt)
        if gt_self is None or gt_nei is None:
            continue

        # 真值
        gt_sx, gt_sy = gt_self['x'], gt_self['y']
        gt_nx, gt_ny = gt_nei['x'],  gt_nei['y']
        gt_rel_x = gt_nx - gt_sx
        gt_rel_y = gt_ny - gt_sy

        # 模拟邻居报告位置（里程计漂移后）
        raw_x, raw_y = apply_drift_to_position(gt_nx, gt_ny,
                                               drift_dx, drift_dy, drift_yaw)

        # 应用 swarm_controller 修正公式
        est_dx, est_dy, est_yaw = sample['dx'], sample['dy'], sample['yaw']
        corr_x, corr_y = apply_swarm_correction(raw_x, raw_y,
                                                est_dx, est_dy, est_yaw)

        # 误差 (绝对邻居位置)
        err_raw_dx   = raw_x - gt_nx
        err_raw_dy   = raw_y - gt_ny
        err_raw_trans = math.hypot(err_raw_dx, err_raw_dy)

        err_corr_dx   = corr_x - gt_nx
        err_corr_dy   = corr_y - gt_ny
        err_corr_trans = math.hypot(err_corr_dx, err_corr_dy)

        # 改进
        if err_raw_trans > 1e-6:
            improvement = (err_raw_trans - err_corr_trans) / err_raw_trans * 100.0
        else:
            improvement = 0.0

        # 修正方向是否正确 (修正后误差 < 修正前误差)
        correct_dir = (err_corr_trans < err_raw_trans)

        records.append(CorrRecord(
            t=sample['t'],
            observer_id=observer_id,
            neighbor_id=neighbor_id,
            gt_self_x=gt_sx, gt_self_y=gt_sy,
            gt_nei_x=gt_nx, gt_nei_y=gt_ny,
            gt_rel_x=gt_rel_x, gt_rel_y=gt_rel_y,
            drift_nei_dx=drift_dx, drift_nei_dy=drift_dy, drift_nei_yaw=drift_yaw,
            raw_nei_x=raw_x, raw_nei_y=raw_y,
            est_dx=est_dx, est_dy=est_dy, est_yaw=est_yaw,
            corr_nei_x=corr_x, corr_nei_y=corr_y,
            err_raw_trans=err_raw_trans, err_raw_dx=err_raw_dx,
            err_raw_dy=err_raw_dy,
            err_corr_trans=err_corr_trans, err_corr_dx=err_corr_dx,
            err_corr_dy=err_corr_dy,
            improvement_pct=improvement,
            correct_direction=correct_dir,
        ))
    return records


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------
def stats(arr):
    if not arr or len(arr) == 0:
        return {}
    a = np.array(arr, dtype=np.float64)
    return {
        'count': int(len(a)),
        'mean': float(np.mean(a)),
        'std': float(np.std(a, ddof=1)) if len(a) > 1 else 0.0,
        'rms': float(np.sqrt(np.mean(np.square(a)))),
        'min': float(np.min(a)),
        'max': float(np.max(a)),
        'p50': float(np.percentile(a, 50)),
        'p90': float(np.percentile(a, 90)),
        'p95': float(np.percentile(a, 95)),
    }


def compute_metrics(records):
    if not records:
        return {'error': 'no valid records'}

    raw_trans = [r.err_raw_trans for r in records]
    corr_trans = [r.err_corr_trans for r in records]
    improvements = [r.improvement_pct for r in records]
    n_correct = sum(1 for r in records if r.correct_direction)
    n_total = len(records)

    # 分层：仅看有足够漂移的帧（raw error > 0.05m，否则改进无意义）
    meaningful = [r for r in records if r.err_raw_trans >= 0.05]
    imp_meaningful = [r.improvement_pct for r in meaningful]

    # 分层：分别看有正向改进和负向改进的帧
    positive = [r for r in records if r.improvement_pct > 0]
    negative = [r for r in records if r.improvement_pct < 0]

    return {
        'total_frames': n_total,
        'correct_direction_pct': n_correct / n_total * 100 if n_total > 0 else 0,
        'n_correct': n_correct,
        'n_wrong': n_total - n_correct,

        'raw_error': {
            'trans': stats(raw_trans),
            'dx': stats([abs(r.err_raw_dx) for r in records]),
            'dy': stats([abs(r.err_raw_dy) for r in records]),
        },
        'corrected_error': {
            'trans': stats(corr_trans),
            'dx': stats([abs(r.err_corr_dx) for r in records]),
            'dy': stats([abs(r.err_corr_dy) for r in records]),
        },
        'improvement': stats(improvements),
        'improvement_meaningful': stats(imp_meaningful) if imp_meaningful else {},
        'improvement_positive': stats([r.improvement_pct for r in positive]) if positive else {},
        'improvement_negative': stats([abs(r.improvement_pct) for r in negative]) if negative else {},
        'n_meaningful': len(meaningful),
        'n_positive': len(positive),
        'n_negative': len(negative),
        'verdict': 'PASS' if (stats(improvements).get('mean', 0) > 0
                              and n_correct / n_total > 0.5) else 'FAIL',
    }


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------
def plot_error_time_series(records, pdf):
    """图1: 修正前后位置误差随时间变化."""
    if not records or not _MPL:
        return
    t = np.array([r.t - records[0].t for r in records])
    raw_t  = np.array([r.err_raw_trans for r in records])
    corr_t = np.array([r.err_corr_trans for r in records])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig.suptitle('Swarm-Controller Correction Effect: Neighbor Position Error',
                 fontweight='bold', fontsize=13)

    # 平移误差
    ax = axes[0]
    ax.plot(t, raw_t, color=C_RED, linewidth=0.7, alpha=0.7, label='Before correction')
    ax.plot(t, corr_t, color=C_GREEN, linewidth=0.7, alpha=0.7, label='After correction')
    ax.set_ylabel('Abs Position Error (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.set_title('Absolute Neighbor Position Error')

    # dx/dy 分量
    ax2 = axes[1]
    raw_dx = np.array([abs(r.err_raw_dx) for r in records])
    raw_dy = np.array([abs(r.err_raw_dy) for r in records])
    corr_dx = np.array([abs(r.err_corr_dx) for r in records])
    corr_dy = np.array([abs(r.err_corr_dy) for r in records])
    ax2.plot(t, raw_dx, color=C_RED, linewidth=0.4, alpha=0.4, label='|dx| raw')
    ax2.plot(t, raw_dy, color=C_ORANGE, linewidth=0.4, alpha=0.4, label='|dy| raw')
    ax2.plot(t, corr_dx, color=C_GREEN, linewidth=0.6, alpha=0.8, label='|dx| corrected')
    ax2.plot(t, corr_dy, color=C_BLUE, linewidth=0.6, alpha=0.8, label='|dy| corrected')
    ax2.set_ylabel('Component Error (m)')
    ax2.legend(fontsize=7, ncol=2)
    ax2.grid(True, alpha=0.25)
    ax2.set_title('dx / dy Error Components')

    # 改进 %
    ax3 = axes[2]
    imp = np.array([r.improvement_pct for r in records])
    ax3.fill_between(t, 0, imp, where=(np.array(imp) > 0),
                     alpha=0.3, color=C_GREEN, label='Positive')
    ax3.fill_between(t, 0, imp, where=(np.array(imp) < 0),
                     alpha=0.3, color=C_RED, label='Negative')
    ax3.plot(t, imp, color=C_BLUE, linewidth=0.5, alpha=0.6)
    ax3.axhline(y=0, color=C_GREY, linestyle='--', linewidth=0.8)
    ax3.axhline(y=np.mean(imp), color=C_ORANGE, linestyle='--', linewidth=1.0,
                label=f'mean={np.mean(imp):.1f}%')
    ax3.set_ylabel('Improvement (%)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.25)
    ax3.set_title('Correction Improvement Over Time')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


def plot_before_after_scatter(records, pdf):
    """图2: 修正前误差 vs 修正后误差 散点图."""
    if not records or not _MPL:
        return
    raw_t  = np.array([r.err_raw_trans for r in records])
    corr_t = np.array([r.err_corr_trans for r in records])

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    fig.suptitle('Before vs After Correction — Does It Help?', fontweight='bold')

    # 散点图: raw vs corrected
    ax = axes[0]
    ax.scatter(raw_t, corr_t, s=8, c=C_BLUE, alpha=0.4, edgecolors='none')
    lims = [0, max(np.max(raw_t), np.max(corr_t)) * 1.1]
    ax.plot(lims, lims, '--', color=C_RED, linewidth=1.2, label='y=x (no change)')
    ax.fill_between(lims, 0, lims, alpha=0.05, color=C_GREEN,
                    label='Below line = Better')
    ax.set_xlim(lims); ax.set_ylim(lims)
    ax.set_xlabel('Error Before Correction (m)')
    ax.set_ylabel('Error After Correction (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.set_aspect('equal')
    ax.set_title('Error Before vs After')

    # 直方图: 改进分布
    ax2 = axes[1]
    imp = np.array([r.improvement_pct for r in records])
    bins = np.linspace(min(imp) - 5, max(imp) + 5, 50)
    colors = [C_GREEN if b >= 0 else C_RED for b in (bins[:-1] + bins[1:]) / 2]
    ax2.hist(imp, bins=bins, color=C_BLUE, alpha=0.6, edgecolor='white', linewidth=0.4)
    ax2.axvline(x=0, color=C_RED, linestyle='--', linewidth=1.2, label='No change')
    ax2.axvline(x=np.mean(imp), color=C_GREEN, linestyle='--', linewidth=1.2,
                label=f'mean={np.mean(imp):.1f}%')
    n_pos = sum(1 for v in imp if v > 0)
    n_neg = sum(1 for v in imp if v < 0)
    ax2.set_xlabel('Improvement (%)')
    ax2.set_ylabel('Count')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.2, axis='y')
    ax2.set_title(f'Improvement Distribution ({n_pos}+ / {n_neg}-)')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


def plot_2d_position_comparison(records, pdf):
    """图3: 2D 位置对比 — GT vs Raw vs Corrected."""
    if not records or not _MPL:
        return

    # 采样以保持可读性
    step = max(1, len(records) // 300)
    sub = records[::step]

    gt_x   = np.array([r.gt_nei_x for r in sub])
    gt_y   = np.array([r.gt_nei_y for r in sub])
    raw_x  = np.array([r.raw_nei_x for r in sub])
    raw_y  = np.array([r.raw_nei_y for r in sub])
    corr_x = np.array([r.corr_nei_x for r in sub])
    corr_y = np.array([r.corr_nei_y for r in sub])

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    fig.suptitle('2D Neighbor Position: GT vs Raw vs Corrected', fontweight='bold')

    # 左: 绝对位置
    ax = axes[0]
    ax.plot(gt_x, gt_y, '.', color=C_GREY, markersize=2, alpha=0.5, label='GT')
    ax.scatter(raw_x, raw_y, s=6, c=C_RED, alpha=0.5, edgecolors='none', label='Raw (drifted)')
    ax.scatter(corr_x, corr_y, s=6, c=C_GREEN, alpha=0.5, edgecolors='none', label='Corrected')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)
    ax.set_aspect('equal')
    ax.set_title('Absolute Positions (world frame)')

    # 右: 误差向量 (raw_err → corr_err)
    ax2 = axes[1]
    err_raw_dx = np.array([r.err_raw_dx for r in sub])
    err_raw_dy = np.array([r.err_raw_dy for r in sub])
    err_corr_dx = np.array([r.err_corr_dx for r in sub])
    err_corr_dy = np.array([r.err_corr_dy for r in sub])

    ax2.scatter(err_raw_dx, err_raw_dy, s=10, c=C_RED, alpha=0.4,
                edgecolors='none', label='Raw error')
    ax2.scatter(err_corr_dx, err_corr_dy, s=10, c=C_GREEN, alpha=0.4,
                edgecolors='none', label='Corrected error')
    ax2.axhline(y=0, color=C_GREY, linestyle='--', linewidth=0.5)
    ax2.axvline(x=0, color=C_GREY, linestyle='--', linewidth=0.5)

    # 1σ / 2σ 圆 (raw)
    raw_sigma = np.std([r.err_raw_trans for r in records])
    for n, ls, lbl in [(1, '-', '1σ raw'), (2, '--', '2σ raw')]:
        c = plt.Circle((0, 0), n * raw_sigma, fill=False, color=C_RED,
                       linestyle=ls, linewidth=0.8, alpha=0.4)
        ax2.add_patch(c)
    ax2.set_xlabel('X Error (m)')
    ax2.set_ylabel('Y Error (m)')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.25)
    ax2.set_aspect('equal')
    ax2.set_title('Position Error Vectors')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


def plot_summary_dashboard(records, metrics, drift_self, drift_nei, pdf):
    """图4: 汇总仪表板."""
    if not records or not _MPL:
        return
    fig = plt.figure(figsize=(13, 9))
    fig.suptitle('Correction Effect Evaluation — Summary Dashboard',
                 fontweight='bold', fontsize=14)
    gs = fig.add_gridspec(3, 2, hspace=0.4, wspace=0.35)

    # 左上: 指标表格
    ax_tbl = fig.add_subplot(gs[0, 0])
    ax_tbl.axis('off')
    raw_m = metrics['raw_error']['trans']
    corr_m = metrics['corrected_error']['trans']
    imp_m = metrics['improvement']
    verdict = metrics['verdict']
    verdict_color = C_GREEN if verdict == 'PASS' else C_RED

    rows = [
        ['Metric',        'Before',         'After'],
        ['RMS (m)',       f'{raw_m["rms"]:.4f}',     f'{corr_m["rms"]:.4f}'],
        ['Mean (m)',      f'{raw_m["mean"]:.4f}',    f'{corr_m["mean"]:.4f}'],
        ['P50 (m)',       f'{raw_m["p50"]:.4f}',     f'{corr_m["p50"]:.4f}'],
        ['P90 (m)',       f'{raw_m["p90"]:.4f}',     f'{corr_m["p90"]:.4f}'],
        ['P95 (m)',       f'{raw_m["p95"]:.4f}',     f'{corr_m["p95"]:.4f}'],
        ['', '', ''],
        ['Improvement',   '',                f'{imp_m["mean"]:.1f}%'],
        ['Correct dir.',  '',                f'{metrics["correct_direction_pct"]:.1f}%'],
        ['', '', ''],
        ['Verdict',       '',                verdict],
    ]
    tbl = ax_tbl.table(cellText=rows, loc='center', cellLoc='center',
                       colWidths=[0.28, 0.22, 0.22])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.45)
    for (row, col), cell in tbl.get_celld().items():
        if row == 0:
            cell.set_facecolor('#E3F2FD')
            cell.set_text_props(fontweight='bold')
        if row == len(rows) - 1:
            cell.set_facecolor(verdict_color)
            cell.set_text_props(fontweight='bold', color='white')

    # 右上: 场景说明
    ax_info = fig.add_subplot(gs[0, 1])
    ax_info.axis('off')
    info = (
        "Scenario: Simulated odometry drift\n"
        "Correction = swarm_controller formula\n\n"
        f"UAV1 injected drift:\n"
        f"  dx={drift_self[0]:.3f} dy={drift_self[1]:.3f} yaw={math.degrees(drift_self[2]):.2f}°\n\n"
        f"UAV2 injected drift:\n"
        f"  dx={drift_nei[0]:.3f} dy={drift_nei[1]:.3f} yaw={math.degrees(drift_nei[2]):.2f}°\n\n"
        f"Formula: corrected = R(yaw)*pos + t\n\n"
        f"Total frames: {metrics['total_frames']}\n"
        f"Positive improvement: {metrics['n_positive']}\n"
        f"Negative (worse): {metrics['n_negative']}"
    )
    ax_info.text(0.05, 0.5, info, transform=ax_info.transAxes,
                 fontsize=9.5, verticalalignment='center',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='#E8F5E9', alpha=0.8))

    # 中左: 误差 CDF 对比
    ax_cdf = fig.add_subplot(gs[1, 0])
    raw_vals = np.sort([r.err_raw_trans for r in records])
    corr_vals = np.sort([r.err_corr_trans for r in records])
    y_cdf = np.arange(1, len(raw_vals) + 1) / len(raw_vals) * 100
    ax_cdf.plot(raw_vals, y_cdf, color=C_RED, linewidth=2, label='Before')
    ax_cdf.plot(corr_vals, y_cdf, color=C_GREEN, linewidth=2, label='After')
    for pct in [50, 90, 95]:
        ax_cdf.axhline(y=pct, color=C_GREY, linestyle=':', linewidth=0.6)
    ax_cdf.set_xlabel('Position Error (m)')
    ax_cdf.set_ylabel('Cumulative %')
    ax_cdf.legend(fontsize=9)
    ax_cdf.grid(True, alpha=0.25)
    ax_cdf.set_xlim(left=0)
    ax_cdf.set_title('Error CDF: Before vs After')

    # 中右: 误差 vs 漂移量
    ax_dr = fig.add_subplot(gs[1, 1])
    drift_mag = np.array([math.hypot(r.drift_nei_dx, r.drift_nei_dy) for r in records])
    corr_err = np.array([r.err_corr_trans for r in records])
    ax_dr.scatter(drift_mag, corr_err, s=5, c=C_BLUE, alpha=0.3, edgecolors='none')
    ax_dr.set_xlabel('Injected Drift Magnitude (m)')
    ax_dr.set_ylabel('Corrected Error (m)')
    ax_dr.grid(True, alpha=0.25)
    ax_dr.set_title('Corrected Error vs Drift Magnitude')

    # 下左: 修正效果分类饼图
    ax_pie = fig.add_subplot(gs[2, 0])
    sizes = [metrics['n_positive'], metrics['n_negative']]
    labels = [f'Better\n({metrics["n_positive"]})',
              f'Worse\n({metrics["n_negative"]})']
    colors_pie = [C_GREEN, C_RED]
    if sum(sizes) > 0:
        ax_pie.pie(sizes, labels=labels, colors=colors_pie, autopct='%1.1f%%',
                   startangle=90, textprops={'fontsize': 10})
    ax_pie.set_title('Correction Direction')

    # 下右: 修正后残余误差分布
    ax_hist = fig.add_subplot(gs[2, 1])
    ax_hist.hist([r.err_raw_trans for r in records], bins=40,
                 color=C_RED, alpha=0.5, edgecolor='white', linewidth=0.3,
                 label='Before')
    ax_hist.hist([r.err_corr_trans for r in records], bins=40,
                 color=C_GREEN, alpha=0.5, edgecolor='white', linewidth=0.3,
                 label='After')
    ax_hist.axvline(x=np.mean([r.err_raw_trans for r in records]),
                    color=C_RED, linestyle='--', linewidth=1.0)
    ax_hist.axvline(x=np.mean([r.err_corr_trans for r in records]),
                    color=C_GREEN, linestyle='--', linewidth=1.0)
    ax_hist.set_xlabel('Position Error (m)')
    ax_hist.set_ylabel('Count')
    ax_hist.legend(fontsize=8)
    ax_hist.grid(True, alpha=0.2, axis='y')
    ax_hist.set_title('Error Distribution: Before vs After')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------
def generate_report(records, metrics, drift_self, drift_nei, out_dir):
    prefix = os.path.join(out_dir, 'correction_offline')

    # JSON
    json_path = prefix + '_summary.json'
    summary = {
        'scenario': 'swarm_controller_correction_effect_offline',
        'ground_truth': 'simulation_odometry',
        'correction_formula': 'corrected = R(yaw) * pos + t  '
                             '(identical to swarm_controller::drift_correct_pos_nei)',
        'description': (
            '从方案B rosbag 离线评估: 注入漂移模拟邻居里程计漂移 → '
            '应用 swarm_controller 修正公式 → 对比修正前后邻居位置误差'
        ),
        'injected_drift': {
            'uav1': {'dx': drift_self[0], 'dy': drift_self[1],
                     'yaw_rad': drift_self[2],
                     'yaw_deg': math.degrees(drift_self[2])},
            'uav2': {'dx': drift_nei[0], 'dy': drift_nei[1],
                     'yaw_rad': drift_nei[2],
                     'yaw_deg': math.degrees(drift_nei[2])},
        },
        'evaluation_logic': (
            'raw = R(nei_yaw)*gt_nei + (nei_dx, nei_dy)  # 模拟邻居报告位置\n'
            'corrected = R(est_yaw)*raw + [est_dx, est_dy]  # swarm_controller 修正公式\n'
            'err_raw = |raw - gt_nei|, err_corr = |corrected - gt_nei|\n'
            'improvement = (err_raw - err_corr) / err_raw * 100%'
        ),
        'metrics': metrics,
    }
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"  JSON: {json_path}")

    # CSV
    csv_path = prefix + '_per_frame.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(CorrRecord._fields)
        for r in records:
            writer.writerow([
                f'{r.t:.6f}', r.observer_id, r.neighbor_id,
                f'{r.gt_self_x:.4f}', f'{r.gt_self_y:.4f}',
                f'{r.gt_nei_x:.4f}', f'{r.gt_nei_y:.4f}',
                f'{r.gt_rel_x:.4f}', f'{r.gt_rel_y:.4f}',
                f'{r.drift_nei_dx:.4f}', f'{r.drift_nei_dy:.4f}',
                f'{r.drift_nei_yaw:.6f}',
                f'{r.raw_nei_x:.4f}', f'{r.raw_nei_y:.4f}',
                f'{r.est_dx:.4f}', f'{r.est_dy:.4f}', f'{r.est_yaw:.6f}',
                f'{r.corr_nei_x:.4f}', f'{r.corr_nei_y:.4f}',
                f'{r.err_raw_trans:.6f}', f'{r.err_raw_dx:.6f}',
                f'{r.err_raw_dy:.6f}',
                f'{r.err_corr_trans:.6f}', f'{r.err_corr_dx:.6f}',
                f'{r.err_corr_dy:.6f}',
                f'{r.improvement_pct:.2f}', str(r.correct_direction),
            ])
    print(f"  CSV:  {csv_path}")

    # PDF
    if _MPL:
        pdf_path = prefix + '_report.pdf'
        with PdfPages(pdf_path) as pdf:
            plot_error_time_series(records, pdf)
            plot_before_after_scatter(records, pdf)
            plot_2d_position_comparison(records, pdf)
            plot_summary_dashboard(records, metrics, drift_self, drift_nei, pdf)
        print(f"  PDF:  {pdf_path} ({os.path.getsize(pdf_path)/1024:.0f} KB)")
    else:
        print("  WARNING: matplotlib 不可用，跳过 PDF")


def print_report(metrics, drift_self, drift_nei):
    """终端报告."""
    hr = "=" * 72
    print(f"\n{hr}")
    print(f"  tree_pose_error 修正效果评估 (方案C)")
    print(f"  Ground Truth: 仿真里程计")
    print(f"  修正公式: corrected = R(yaw) * pos + t  (swarm_controller)")
    print(f"{hr}")

    print(f"\n  [注入漂移]")
    print(f"    Observer (self):  dx={drift_self[0]:.3f}m  "
          f"dy={drift_self[1]:.3f}m  yaw={math.degrees(drift_self[2]):.2f}°")
    print(f"    Neighbor:         dx={drift_nei[0]:.3f}m  "
          f"dy={drift_nei[1]:.3f}m  yaw={math.degrees(drift_nei[2]):.2f}°")

    raw_m  = metrics['raw_error']['trans']
    corr_m = metrics['corrected_error']['trans']
    imp_m  = metrics['improvement']

    print(f"\n  [邻居位置误差 — 修正前 vs 修正后]")
    print(f"    总帧数: {metrics['total_frames']}")
    print(f"    (std=标准差, rms=均方根)")
    print(f"    {'指标':<12} {'修正前':>8} {'修正后':>8} {'改进':>8}")
    print(f"    {'-'*44}")
    for lbl, key in [('RMS (m)', 'rms'), ('Mean (m)', 'mean'),
                     ('Max (m)', 'max'), ('P50 (m)', 'p50'),
                     ('P90 (m)', 'p90'), ('P95 (m)', 'p95')]:
        print(f"    {lbl:<12} {raw_m[key]:>8.4f} {corr_m[key]:>8.4f} "
              f"{'-' if corr_m[key] > raw_m[key] else '↓'}")

    print(f"\n  [修正效果]")
    print(f"    平均改进:     {imp_m['mean']:>+.1f}%")
    print(f"    P50 改进:     {imp_m['p50']:>+.1f}%")
    print(f"    正向修正比例: {metrics['correct_direction_pct']:.1f}% "
          f"({metrics['n_correct']}/{metrics['total_frames']})")
    print(f"    负向修正帧:   {metrics['n_wrong']}")

    if metrics.get('improvement_meaningful'):
        imp_mf = metrics['improvement_meaningful']
        print(f"\n  [有效帧 (raw error >= 0.05m)]: {metrics['n_meaningful']} 帧")
        print(f"    平均改进: {imp_mf.get('mean', 0):+.1f}%")

    print(f"\n  [判定] {metrics['verdict']}")
    print(f"    (PASS = 平均改进 > 0 且 正向比例 > 50%)")
    print(f"{hr}\n")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def find_bag(log_dir):
    candidates = [os.path.join(log_dir, f) for f in os.listdir(log_dir)
                  if f.endswith('.bag')]
    if not candidates:
        return None
    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def load_drift_from_config(log_dir):
    cfg_path = os.path.join(log_dir, 'eval_config.json')
    if not os.path.isfile(cfg_path):
        print("ERROR: eval_config.json 未找到")
        return None
    with open(cfg_path) as f:
        cfg = json.load(f)
    d1_yaw = math.radians(cfg.get('drift1_yaw_deg', 0.0))
    d2_yaw = math.radians(cfg.get('drift2_yaw_deg', 0.0))
    drift1 = (cfg.get('drift1_dx', 0.0), cfg.get('drift1_dy', 0.0), d1_yaw)
    drift2 = (cfg.get('drift2_dx', 0.0), cfg.get('drift2_dy', 0.0), d2_yaw)
    has_drift = any(abs(v) > 1e-9 for d in [drift1, drift2]
                    for v in [d[0], d[1], math.degrees(d[2])])
    return drift1, drift2, has_drift, cfg


def main():
    parser = argparse.ArgumentParser(
        description='离线评估 tree_pose_error 对邻居位置的修正效果 (方案C)')
    parser.add_argument('log_dir', help='日志目录 (含 drift_eval.bag + eval_config.json)')
    parser.add_argument('--max-dt', type=float, default=0.5,
                        help='时间对齐最大偏差 (s)')
    args = parser.parse_args()

    log_dir = args.log_dir
    if not os.path.isdir(log_dir):
        print(f"ERROR: 目录不存在: {log_dir}")
        sys.exit(1)

    result = load_drift_from_config(log_dir)
    if result is None:
        sys.exit(1)
    drift1, drift2, has_drift, cfg = result

    if not has_drift:
        print("WARNING: 双方漂移均为0，修正评估无意义")
        sys.exit(1)

    bag_path = find_bag(log_dir)
    if bag_path is None:
        print(f"ERROR: 目录中未找到 .bag: {log_dir}")
        sys.exit(1)

    print(f"\n{'='*72}")
    print(f"  修正效果评估 (方案C) — 离线应用 swarm_controller 修正公式")
    print(f"  Bag: {bag_path}")
    print(f"{'='*72}")

    # ---- 1. 加载数据 ----
    print("\n[1/3] 加载 rosbag ...")
    tpe1 = extract_tree_pose_error(bag_path, '/uav1/tree_pose_error')
    tpe2 = extract_tree_pose_error(bag_path, '/uav2/tree_pose_error')
    odom1 = extract_odometry(bag_path, '/uav1/lidar_slam/odom')
    odom2 = extract_odometry(bag_path, '/uav2/lidar_slam/odom')
    print(f"  /uav1/tree_pose_error: {len(tpe1)} 条")
    print(f"  /uav2/tree_pose_error: {len(tpe2)} 条")
    print(f"  odom1: {len(odom1)} 条, odom2: {len(odom2)} 条")

    if not tpe1 and not tpe2:
        print("ERROR: 无 tree_pose_error 数据")
        sys.exit(1)

    # ---- 2. 构建评估帧 ----
    print("\n[2/3] 构建评估帧 ...")

    # swarm_controller 订阅关系:
    #   UAV1 订阅 /uav2/tree_pose_error 来修正 UAV2 的报告位置
    # swarm_controller 订阅: /uav{nei}/tree_pose_error
    # UAV1 以自己的里程计为参考系，用 /uav2 的估计修正 UAV2 的报告位置
    all_records = build_records(tpe2, odom1, odom2, drift2, 1, 2, args.max_dt)

    print(f"  有效评估帧: {len(all_records)}")

    if not all_records:
        if not has_drift:
            print("\nINFO: UAV2 无漂移，无需评估修正效果")
        else:
            print("\nWARNING: UAV2 有漂移但无有效评估帧。")
            print("  可能原因: UAV2 树匹配失败或 dt 对齐超时")
        sys.exit(0)

    # ---- 3. 计算 + 输出 ----
    print("\n[3/3] 计算指标 + 生成输出 ...")

    metrics = compute_metrics(all_records)
    print_report(metrics, drift1, drift2)
    generate_report(all_records, metrics, drift1, drift2, log_dir)

    print("  完成.\n")


if __name__ == '__main__':
    main()
