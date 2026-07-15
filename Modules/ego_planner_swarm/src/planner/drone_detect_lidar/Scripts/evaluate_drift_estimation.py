#!/usr/bin/env python3
# ============================================================================
# 方案B: 漂移估计精度评估
#
# 在双方树位置注入已知独立漂移后，对比 tree_pose_error 估计的相对漂移
# 与理论真值相对漂移，评估树匹配对坐标系偏移的估计精度。
#
# 数学:
#   UAV1 drift: R1(yaw1) * pos + t1(dx1, dy1)
#   UAV2 drift: R2(yaw2) * pos + t2(dx2, dy2)
#
#   理论上 tree_pose_error (UAV1's view) 应估计:
#     R_est = R2 * R1^T = R(yaw2 - yaw1)
#     t_est = t2 - R_est * t1
#
# 输入:
#   有已知漂移参数的 rosbag (含 /uav{1,2}/tree_pose_error)
#   漂移配置 (eval_config.json)
#
# 输出:
#   <日志目录>/drift_est_report.pdf
#   <日志目录>/drift_est_summary.json
#   <日志目录>/drift_est_per_frame.csv
#
# 用法:
#   python3 evaluate_drift_estimation.py <日志目录>
# ============================================================================

import sys
import os
import json
import csv
import math
import argparse
from collections import namedtuple

import numpy as np

_MPL_AVAILABLE = False
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    _MPL_AVAILABLE = True
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 150
    import logging
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
except ImportError:
    pass


DriftEstRecord = namedtuple('DriftEstRecord', [
    't', 'observer_id',
    'est_dx', 'est_dy', 'est_yaw_deg',
    'gt_dx', 'gt_dy', 'gt_yaw_deg',
    'err_dx', 'err_dy', 'err_trans', 'err_yaw_deg',
])

C_BLUE   = '#2196F3'
C_ORANGE = '#FF9800'
C_GREEN  = '#4CAF50'
C_RED    = '#F44336'
C_GREY   = '#9E9E9E'
C_PURPLE = '#9C27B0'


def _require_rosbag():
    try:
        import rosbag
        return rosbag
    except ImportError:
        print("ERROR: rosbag 模块未找到。请先 source ROS 环境")
        sys.exit(1)


def extract_tree_pose_error(bag_path, topic):
    """提取 tree_pose_error (geometry_msgs/Vector3: x=dx, y=dy, z=yaw)."""
    rosbag = _require_rosbag()
    samples = []
    with rosbag.Bag(bag_path) as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            samples.append({
                't':     t.to_sec(),
                'dx':    msg.x,
                'dy':    msg.y,
                'yaw':   msg.z,   # already in radians
            })
    return sorted(samples, key=lambda s: s['t'])


def compute_ground_truth_drift(drift1, drift2):
    """
    计算理论上的 tree_pose_error (从 drone1 帧到 drone2 帧的变换).

    drift1: (dx1, dy1, yaw1_rad) — UAV1 注入的漂移
    drift2: (dx2, dy2, yaw2_rad) — UAV2 注入的漂移

    Returns (gt_dx, gt_dy, gt_yaw_rad):
      R_est = R(yaw2) * R(yaw1)^T = R(yaw2 - yaw1)
      t_est = t2 - R_est * t1
    """
    dx1, dy1, yaw1 = drift1
    dx2, dy2, yaw2 = drift2

    # Rotation from frame1 to frame2
    rel_yaw = yaw2 - yaw1
    c = math.cos(rel_yaw)
    s = math.sin(rel_yaw)

    # Translation: t2 - R(rel_yaw) * t1
    gt_dx = dx2 - (c * dx1 - s * dy1)
    gt_dy = dy2 - (s * dx1 + c * dy1)

    return gt_dx, gt_dy, rel_yaw


def build_records(est_samples, gt_dx, gt_dy, gt_yaw, observer_id):
    """逐帧对齐估计值和理论真值."""
    records = []
    for est in est_samples:
        err_dx = est['dx'] - gt_dx
        err_dy = est['dy'] - gt_dy
        err_trans = math.hypot(err_dx, err_dy)

        # Yaw 差值规整到 [-180, 180]
        est_yaw_deg = math.degrees(est['yaw'])
        gt_yaw_deg = math.degrees(gt_yaw)
        err_yaw = est_yaw_deg - gt_yaw_deg
        while err_yaw <= -180:
            err_yaw += 360
        while err_yaw > 180:
            err_yaw -= 360

        records.append(DriftEstRecord(
            t=est['t'],
            observer_id=observer_id,
            est_dx=est['dx'],
            est_dy=est['dy'],
            est_yaw_deg=est_yaw_deg,
            gt_dx=gt_dx,
            gt_dy=gt_dy,
            gt_yaw_deg=gt_yaw_deg,
            err_dx=err_dx,
            err_dy=err_dy,
            err_trans=err_trans,
            err_yaw_deg=err_yaw,
        ))
    return records


def compute_metrics(records):
    if not records:
        return {'error': 'no records'}

    def _stats(arr):
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

    return {
        'total_frames': len(records),
        'trans_error': _stats([abs(r.err_trans) for r in records]),
        'dx_error': _stats([abs(r.err_dx) for r in records]),
        'dy_error': _stats([abs(r.err_dy) for r in records]),
        'yaw_error_deg': _stats([abs(r.err_yaw_deg) for r in records]),
        'est_dx': _stats([r.est_dx for r in records]),
        'est_dy': _stats([r.est_dy for r in records]),
        'est_yaw_deg': _stats([r.est_yaw_deg for r in records]),
        'gt_dx': float(records[0].gt_dx) if records else 0,
        'gt_dy': float(records[0].gt_dy) if records else 0,
        'gt_yaw_deg': float(records[0].gt_yaw_deg) if records else 0,
    }


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------

def plot_error_time_series(records, pdf):
    if not records or not _MPL_AVAILABLE:
        return

    t = np.array([r.t - records[0].t for r in records])
    trans  = np.array([abs(r.err_trans)   for r in records])
    yaw_arr = np.array([abs(r.err_yaw_deg) for r in records])

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle('Drift Estimation Accuracy — tree_pose_error vs Known Injected Drift',
                 fontweight='bold', fontsize=13)

    ax = axes[0]
    ax.plot(t, trans, color=C_BLUE, linewidth=0.7, alpha=0.8)
    ax.axhline(y=np.mean(trans), color=C_RED, linestyle='--', linewidth=0.8,
               label=f'mean={np.mean(trans):.3f} m')
    ax.set_ylabel('Translation Error (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.set_title('Translation Estimation Error')

    ax2 = axes[1]
    ax2.plot(t, yaw_arr, color=C_PURPLE, linewidth=0.7, alpha=0.8)
    ax2.axhline(y=np.mean(yaw_arr), color=C_RED, linestyle='--', linewidth=0.8,
                label=f'mean={np.mean(yaw_arr):.2f}°')
    ax2.set_ylabel('Yaw Error (deg)')
    ax2.set_xlabel('Time (s)')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.25)
    ax2.set_title('Yaw Estimation Error')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


def plot_est_vs_gt_scatter(records, pdf):
    if not records or not _MPL_AVAILABLE:
        return

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    fig.suptitle('Estimated vs Ground Truth Drift Values', fontweight='bold')

    for idx, (key, ylabel) in enumerate([
        ('est_dx', 'Estimated dx (m)'),
        ('est_dy', 'Estimated dy (m)'),
        ('est_yaw_deg', 'Estimated yaw (deg)'),
    ]):
        ax = axes[idx]
        est_vals = np.array([getattr(r, key) for r in records])
        gt_val = getattr(records[0], 'gt_' + key[4:])  # est_xxx → gt_xxx
        ax.scatter([r.t - records[0].t for r in records], est_vals,
                   s=5, c=C_BLUE, alpha=0.5, edgecolors='none')
        ax.axhline(y=gt_val, color=C_RED, linestyle='--', linewidth=1.2,
                   label=f'GT={gt_val:.3f}')
        ax.set_ylabel(ylabel)
        ax.set_xlabel('Time (s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.25)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


def plot_summary_dashboard(records, metrics, drift1, drift2, pdf):
    if not records or not _MPL_AVAILABLE:
        return

    fig = plt.figure(figsize=(12, 8))
    fig.suptitle('Drift Estimation Evaluation Summary (Plan B)', fontweight='bold', fontsize=14)
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.3)

    # 左上: 指标表格
    ax_table = fig.add_subplot(gs[0, 0])
    ax_table.axis('off')
    te = metrics['trans_error']
    ya = metrics['yaw_error_deg']
    est_dx = metrics['est_dx']
    est_dy = metrics['est_dy']
    est_yaw = metrics['est_yaw_deg']

    table_data = [
        ['Metric', 'Mean', 'RMS', 'P95'],
        ['trans err (m)', f'{te["mean"]:.3f}', f'{te["rms"]:.3f}', f'{te["p95"]:.3f}'],
        ['yaw err (deg)', f'{ya["mean"]:.2f}', f'{ya["rms"]:.2f}', f'{ya["p95"]:.2f}'],
        ['', '', '', ''],
        ['GT rel dx (m)',  f'{metrics["gt_dx"]:.3f}', '', ''],
        ['GT rel dy (m)',  f'{metrics["gt_dy"]:.3f}', '', ''],
        ['GT rel yaw (deg)', f'{metrics["gt_yaw_deg"]:.2f}', '', ''],
        ['Est dx mean (m)', f'{est_dx["mean"]:.3f}', '', ''],
        ['Est dy mean (m)', f'{est_dy["mean"]:.3f}', '', ''],
        ['Est yaw mean (deg)', f'{est_yaw["mean"]:.2f}', '', ''],
    ]

    tbl = ax_table.table(cellText=table_data, loc='center',
                         cellLoc='center', colWidths=[0.32, 0.22, 0.22, 0.22])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.4)
    for (row, col), cell in tbl.get_celld().items():
        if row == 0:
            cell.set_facecolor('#E3F2FD')
            cell.set_text_props(fontweight='bold')

    # 右上: 说明
    ax_info = fig.add_subplot(gs[0, 1])
    ax_info.axis('off')
    info = (
        f"Drift injection:\n"
        f"  UAV1: dx={drift1[0]:.3f} dy={drift1[1]:.3f} yaw={math.degrees(drift1[2]):.2f}°\n"
        f"  UAV2: dx={drift2[0]:.3f} dy={drift2[1]:.3f} yaw={math.degrees(drift2[2]):.2f}°\n\n"
        f"Expected tree_pose_error:\n"
        f"  dx={metrics['gt_dx']:.3f} m\n"
        f"  dy={metrics['gt_dy']:.3f} m\n"
        f"  yaw={metrics['gt_yaw_deg']:.2f}°\n\n"
        f"Translation error RMS: {te['rms']:.3f} m\n"
        f"Yaw error RMS: {ya['rms']:.2f}°\n"
        f"Frames: {metrics['total_frames']}"
    )
    ax_info.text(0.05, 0.5, info, transform=ax_info.transAxes,
                 fontsize=10, verticalalignment='center',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='#E8F5E9', alpha=0.8))

    # 左下: 2D 误差散点
    ax_2d = fig.add_subplot(gs[1, 0])
    err_dx = [r.err_dx for r in records]
    err_dy = [r.err_dy for r in records]
    ax_2d.scatter(err_dx, err_dy, s=6, c=C_BLUE, alpha=0.4, edgecolors='none')
    ax_2d.axhline(y=0, color=C_GREY, linestyle='--', linewidth=0.5)
    ax_2d.axvline(x=0, color=C_GREY, linestyle='--', linewidth=0.5)
    ax_2d.set_xlabel('dx error (m)')
    ax_2d.set_ylabel('dy error (m)')
    ax_2d.set_title('2D Estimation Error')
    ax_2d.grid(True, alpha=0.25)
    ax_2d.set_aspect('equal')

    # 右下: 误差分布
    ax_hist = fig.add_subplot(gs[1, 1])
    trans = [abs(r.err_trans) for r in records]
    ax_hist.hist(trans, bins=30, color=C_BLUE, alpha=0.7, edgecolor='white', linewidth=0.5)
    ax_hist.axvline(x=np.mean(trans), color=C_RED, linestyle='--', linewidth=1.2,
                    label=f'mean={np.mean(trans):.3f}m')
    ax_hist.set_xlabel('Translation Error (m)')
    ax_hist.set_ylabel('Count')
    ax_hist.legend(fontsize=8)
    ax_hist.grid(True, alpha=0.2, axis='y')
    ax_hist.set_title('Translation Error Distribution')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    pdf.savefig(fig, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def generate_report(records, metrics, drift1, drift2, out_dir):
    prefix = os.path.join(out_dir, 'drift_est')

    # JSON
    json_path = prefix + '_summary.json'
    summary = {
        'scenario': 'drift_estimation_accuracy',
        'ground_truth': 'analytically_computed_from_injected_drift',
        'description': (
            '双方树位置注入已知独立漂移后，对比 tree_pose_error 估计的 '
            '相对漂移与理论值。评估树匹配对方差偏移的估计精度。'
        ),
        'injected_drift': {
            'uav1': {'dx': drift1[0], 'dy': drift1[1], 'yaw_rad': drift1[2],
                     'yaw_deg': math.degrees(drift1[2])},
            'uav2': {'dx': drift2[0], 'dy': drift2[1], 'yaw_rad': drift2[2],
                     'yaw_deg': math.degrees(drift2[2])},
        },
        'metrics': metrics,
    }
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"  JSON: {json_path}")

    # CSV
    csv_path = prefix + '_per_frame.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            't', 'observer_id',
            'est_dx', 'est_dy', 'est_yaw_deg',
            'gt_dx', 'gt_dy', 'gt_yaw_deg',
            'err_dx', 'err_dy', 'err_trans', 'err_yaw_deg',
        ])
        for r in records:
            writer.writerow([
                f'{r.t:.6f}', r.observer_id,
                f'{r.est_dx:.4f}', f'{r.est_dy:.4f}', f'{r.est_yaw_deg:.3f}',
                f'{r.gt_dx:.4f}', f'{r.gt_dy:.4f}', f'{r.gt_yaw_deg:.3f}',
                f'{r.err_dx:.4f}', f'{r.err_dy:.4f}', f'{r.err_trans:.4f}',
                f'{r.err_yaw_deg:.3f}',
            ])
    print(f"  CSV:  {csv_path}")

    # PDF
    if _MPL_AVAILABLE:
        pdf_path = prefix + '_report.pdf'
        with PdfPages(pdf_path) as pdf:
            plot_error_time_series(records, pdf)
            plot_est_vs_gt_scatter(records, pdf)
            plot_summary_dashboard(records, metrics, drift1, drift2, pdf)
        print(f"  PDF:  {pdf_path} ({os.path.getsize(pdf_path)/1024:.0f} KB)")
    else:
        print("  WARNING: matplotlib 不可用")


def print_report(metrics, drift1, drift2):
    print(f"  Ground Truth: 注入漂移的理论相对值")

    print(f"\n  [注入漂移]")
    print(f"    UAV1: dx={drift1[0]:.3f}m dy={drift1[1]:.3f}m yaw={math.degrees(drift1[2]):.2f}°")
    print(f"    UAV2: dx={drift2[0]:.3f}m dy={drift2[1]:.3f}m yaw={math.degrees(drift2[2]):.2f}°")

    dx_e = metrics['dx_error']
    dy_e = metrics['dy_error']
    ya   = metrics['yaw_error_deg']

    print(f"\n  [估计精度] {metrics['total_frames']} 帧")
    print(f"    (|估计值 - 理论真值| 的统计, std=标准差, rms=均方根)")
    print(f"    {'指标':<14} {'mean':>8} {'std':>8} {'rms':>8} {'max':>8} {'p50':>8} {'p95':>8}")
    print(f"    {'-'*72}")
    print(f"    {'dx err(m)':<14} {dx_e['mean']:>8.3f} {dx_e['std']:>8.3f} "
          f"{dx_e['rms']:>8.3f} {dx_e['max']:>8.3f} {dx_e['p50']:>8.3f} {dx_e['p95']:>8.3f}")
    print(f"    {'dy err(m)':<14} {dy_e['mean']:>8.3f} {dy_e['std']:>8.3f} "
          f"{dy_e['rms']:>8.3f} {dy_e['max']:>8.3f} {dy_e['p50']:>8.3f} {dy_e['p95']:>8.3f}")
    print(f"    {'yaw err(deg)':<14} {ya['mean']:>8.2f} {ya['std']:>8.2f} "
          f"{ya['rms']:>8.2f} {ya['max']:>8.2f} {ya['p50']:>8.2f} {ya['p95']:>8.2f}")



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
    """从 eval_config.json 加载注入的漂移参数."""
    cfg_path = os.path.join(log_dir, 'eval_config.json')
    if not os.path.isfile(cfg_path):
        print("ERROR: eval_config.json 未找到")
        print("请用 run_tree_drift_eval.sh 运行以确保配置被写入")
        return None

    with open(cfg_path) as f:
        cfg = json.load(f)

    # 支持两种配置格式
    drift1_rad = math.radians(cfg.get('drift1_yaw_deg', 0.0))
    drift2_rad = math.radians(cfg.get('drift2_yaw_deg', 0.0))

    drift1 = (cfg.get('drift1_dx', 0.0), cfg.get('drift1_dy', 0.0), drift1_rad)
    drift2 = (cfg.get('drift2_dx', 0.0), cfg.get('drift2_dy', 0.0), drift2_rad)

    has_drift = any(abs(v) > 1e-9 for d in [drift1, drift2]
                    for v in [d[0], d[1], math.degrees(d[2])])
    return drift1, drift2, has_drift, cfg


def main():
    parser = argparse.ArgumentParser(
        description='漂移估计精度评估 — 对比 tree_pose_error 与注入漂移理论值')
    parser.add_argument('log_dir', help='日志目录 (含 drift_eval.bag + eval_config.json)')
    args = parser.parse_args()

    log_dir = args.log_dir
    if not os.path.isdir(log_dir):
        print(f"ERROR: 目录不存在: {log_dir}")
        sys.exit(1)

    # 加载漂移配置
    result = load_drift_from_config(log_dir)
    if result is None:
        sys.exit(1)
    drift1, drift2, has_drift, cfg = result

    if not has_drift:
        print("WARNING: 双方漂移均为0，评估无意义。请指定非零漂移。")
        print("  示例: --drift1 0.5 0.0 5.0 --drift2 0.0 0.0 0.0")
        sys.exit(1)

    # 找 rosbag
    bag_path = find_bag(log_dir)
    if bag_path is None:
        print(f"ERROR: 目录中未找到 .bag: {log_dir}")
        sys.exit(1)

    print(f"  Bag: {bag_path}")

    # ---- 1. 加载数据 ----
    print("\n[1/3] 加载 rosbag ...")
    est1 = extract_tree_pose_error(bag_path, '/uav1/tree_pose_error')
    est2 = extract_tree_pose_error(bag_path, '/uav2/tree_pose_error')
    print(f"  /uav1/tree_pose_error: {len(est1)} 条")
    print(f"  /uav2/tree_pose_error: {len(est2)} 条")

    if not est1 and not est2:
        print("ERROR: 没有 tree_pose_error 数据")
        sys.exit(1)

    # ---- 2. 计算理论真值 ----
    print("\n[2/3] 计算理论值 + 构建评估帧 ...")

    # UAV1 的 tree_pose_error 估计 UAV1→UAV2 的变换
    # 即从 UAV1 drifted frame 到 UAV2 drifted frame 的变换
    gt_dx_1to2, gt_dy_1to2, gt_yaw_1to2 = compute_ground_truth_drift(drift1, drift2)

    # UAV2 的 tree_pose_error 估计 UAV2→UAV1 的变换
    gt_dx_2to1, gt_dy_2to1, gt_yaw_2to1 = compute_ground_truth_drift(drift2, drift1)

    print(f"  理论 tree_pose_error (UAV1→UAV2): "
          f"dx={gt_dx_1to2:.3f}m dy={gt_dy_1to2:.3f}m yaw={math.degrees(gt_yaw_1to2):.2f}°")
    print(f"  理论 tree_pose_error (UAV2→UAV1): "
          f"dx={gt_dx_2to1:.3f}m dy={gt_dy_2to1:.3f}m yaw={math.degrees(gt_yaw_2to1):.2f}°")

    records1 = build_records(est1, gt_dx_1to2, gt_dy_1to2, gt_yaw_1to2, 1)
    records2 = build_records(est2, gt_dx_2to1, gt_dy_2to1, gt_yaw_2to1, 2)
    all_records = sorted(records1 + records2, key=lambda r: r.t)

    print(f"  UAV1 评估帧: {len(records1)}, UAV2 评估帧: {len(records2)}")

    if not all_records:
        print("ERROR: 无有效评估帧")
        sys.exit(1)

    # ---- 3. 计算 + 输出 ----
    print("\n[3/3] 计算指标 + 生成输出 ...")
    metrics = compute_metrics(all_records)
    print_report(metrics, drift1, drift2)
    generate_report(all_records, metrics, drift1, drift2, log_dir)

    print("  完成.\n")


if __name__ == '__main__':
    main()
