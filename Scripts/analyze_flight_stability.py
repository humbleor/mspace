#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mspace Flight Stability Analyzer
--------------------------------
Description: 
    Parses PX4 ULG files to quantify flight stability based on: 
    1. Attitude tracking error (Setpoint vs Estimate) 
    2. Rate tracking error (PID performance)
    3. Vibration levels (IMU noise) - Supports both estimator_status and sensor_combined

Usage:
    python3 analyze_flight_stability.py <path_to_log.ulg>

    eg. cd Scripts
    python3 analyze_flight_stability.py flight_data/log_296_2026-2-5-16-45-20.ulg

"""

import sys
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyulog import ULog

def quaternion_to_euler(q):
    """
    Convert quaternion (w, x, y, z) to euler angles (roll, pitch, yaw) in degrees.
    PX4 ULG quaternions are typically [w, x, y, z] or [x, y, z, w]. 
    Standard PX4 'vehicle_attitude' is [q[0], q[1], q[2], q[3]] where q[0] is W.
    """
    # q is array-like: [w, x, y, z]
    w, x, y, z = q[:,0], q[:,1], q[:,2], q[:,3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def resample_data(df_target, df_source, time_col='timestamp'):
    """
    Resample source dataframe to match target dataframe timestamps using nearest interpolation.
    """
    # Ensure sorted
    df_target = df_target.sort_values(by=time_col)
    df_source = df_source.sort_values(by=time_col)
    
    # Use merge_asof for efficient nearest matching
    merged = pd.merge_asof(df_target, df_source, on=time_col, direction='nearest', suffixes=('_ref', '_src'))
    return merged

def analyze_attitude(ulog):
    """
    Analyze attitude tracking (Setpoint vs Actual).
    """
    print("\n[1] Attitude Tracking Stability")
    print("-" * 30)

    try:
        # Load datasets
        att = ulog.get_dataset('vehicle_attitude').data
        att_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
        
        df_att = pd.DataFrame(att)
        df_sp = pd.DataFrame(att_sp)
        
        # Convert Quaternions to Euler
        q_att = df_att[['q[0]', 'q[1]', 'q[2]', 'q[3]']].values
        r, p, y = quaternion_to_euler(q_att)
        df_att['roll_deg'] = r
        df_att['pitch_deg'] = p
        
        q_sp = df_sp[['q_d[0]', 'q_d[1]', 'q_d[2]', 'q_d[3]']].values
        r_sp, p_sp, y_sp = quaternion_to_euler(q_sp)
        df_sp['roll_sp_deg'] = r_sp
        df_sp['pitch_sp_deg'] = p_sp

        # Sync data
        df_merged = resample_data(df_att, df_sp)
        
        # Calculate Errors
        roll_err = df_merged['roll_deg'] - df_merged['roll_sp_deg']
        pitch_err = df_merged['pitch_deg'] - df_merged['pitch_sp_deg']
        
        roll_rmse = np.sqrt(np.mean(roll_err**2))
        pitch_rmse = np.sqrt(np.mean(pitch_err**2))
        
        print(f"  Roll  RMSE: {roll_rmse:.4f} degrees")
        print(f"  Pitch RMSE: {pitch_rmse:.4f} degrees")
        
        if roll_rmse > 5.0 or pitch_rmse > 5.0:
            print("    WARNING: Large attitude tracking error! (> 5.0 deg)")
        
        stats = {
            'roll_rmse': roll_rmse,
            'pitch_rmse': pitch_rmse
        }
        return df_merged, stats
        
    except Exception as e:
        print(f"Error analyzing attitude: {e}")
        return None, None

def analyze_rate(ulog):
    """
    Analyze angular rate tracking (Setpoint vs Actual) - PID performance.
    """
    print("\n[2] Rate Tracking Performance (PID)")
    print("-" * 30)

    try:
        # Load datasets
        # Actual angular velocity: xyz[0]=rollrate, xyz[1]=pitchrate, xyz[2]=yawrate (rad/s)
        rate = ulog.get_dataset('vehicle_angular_velocity').data
        rate_sp = ulog.get_dataset('vehicle_rates_setpoint').data
        
        df_rate = pd.DataFrame(rate)
        df_sp = pd.DataFrame(rate_sp)
        
        # Convert rad/s to deg/s
        df_rate['roll_rate_deg'] = np.degrees(df_rate['xyz[0]'])
        df_rate['pitch_rate_deg'] = np.degrees(df_rate['xyz[1]'])
        
        df_sp['roll_rate_sp_deg'] = np.degrees(df_sp['roll'])
        df_sp['pitch_rate_sp_deg'] = np.degrees(df_sp['pitch'])

        # Sync data
        df_merged = resample_data(df_rate, df_sp)
        
        # Calculate Errors
        roll_rate_err = df_merged['roll_rate_deg'] - df_merged['roll_rate_sp_deg']
        pitch_rate_err = df_merged['pitch_rate_deg'] - df_merged['pitch_rate_sp_deg']
        
        roll_rate_rmse = np.sqrt(np.mean(roll_rate_err**2))
        pitch_rate_rmse = np.sqrt(np.mean(pitch_rate_err**2))
        
        print(f"  Roll Rate  RMSE: {roll_rate_rmse:.4f} deg/s")
        print(f"  Pitch Rate RMSE: {pitch_rate_rmse:.4f} deg/s")
        
        stats = {
            'roll_rate_rmse': roll_rate_rmse,
            'pitch_rate_rmse': pitch_rate_rmse
        }
        return df_merged, stats
        
    except Exception as e:
        print(f"Error analyzing rates: {e}")
        return None, None
    
def analyze_vibration(ulog):
    """
    Analyze vibration levels from 'estimator_status' or 'sensor_combined' if available.
    Returns: stats dictionary and a dataframe for plotting.
    """
    print("\n[3] Vibration Analysis")
    print("-" * 30)
    
    vibration_stats = {}
    plot_df = pd.DataFrame()

    # Method 1: Try to find estimator_status
    try:
        vib_data = ulog.get_dataset('estimator_status').data
        vib_df = pd.DataFrame(vib_data)
        cols = [c for c in vib_df.columns if 'vibe' in c]
        
        if cols:
            print("Using pre-computed vibration metrics from 'estimator_status'.")
            plot_df['timestamp'] = vib_df['timestamp']
            for axis, col in zip(['X', 'Y', 'Z'], cols[:3]):
                mean_vib = np.mean(vib_df[col])
                vibration_stats[axis] = mean_vib
                plot_df[f'vibe_{axis}'] = vib_df[col]
                print(f"  Vibration {axis}: Mean={mean_vib:.4f}")
            return vibration_stats, plot_df
    except Exception:
        pass
    
    # Method 2: Fallback to 'sensor_combined'
    print("Pre-computed metrics not found. Calculating from 'sensor_combined'...")
    try:
        sensor_data = ulog.get_dataset('sensor_combined').data
        sensor_df = pd.DataFrame(sensor_data)
        acc_cols = [c for c in sensor_df.columns if 'accelerometer_m_s2' in c]
        
        if acc_cols:
            window_size = 50
            plot_df['timestamp'] = sensor_df['timestamp']
            for axis, col in zip(['X', 'Y', 'Z'], acc_cols[:3]):
                raw_acc = sensor_df[col]
                smoothed_acc = raw_acc.rolling(window=window_size, center=True).mean()
                vibration_noise = raw_acc - smoothed_acc
                # Calculate rolling RMS as a proxy for the vibe metric
                rolling_vib = vibration_noise.rolling(window=window_size).std()
                
                vibration_stats[axis] = np.nanmean(rolling_vib)
                plot_df[f'vibe_{axis}'] = rolling_vib
                print(f"  Vibration {axis} (Est. Mean RMS): {vibration_stats[axis]:.4f} m/s^2")
            return vibration_stats, plot_df
    except Exception as e:
        print(f"  Error: {e}")

    return vibration_stats, None

def main():
    parser = argparse.ArgumentParser(description="Analyze flight stability from PX4 ULG.")
    parser.add_argument("ulg_file", help="Path to .ulg file")
    args = parser.parse_args()

    # --- ULG Analysis ---
    if not os.path.exists(args.ulg_file):
        print(f"Error: File '{args.ulg_file}' not found.")
        sys.exit(1)

    print(f"Processing ULG: {args.ulg_file}")
    try:
        ulog = ULog(args.ulg_file)
    except Exception as e:
        print(f"Failed to parse ULG. Error: {e}")
        sys.exit(1)

    df_att, att_stats = analyze_attitude(ulog)
    df_rate, rate_stats = analyze_rate(ulog)
    vib_stats, df_vib = analyze_vibration(ulog)

    # --- Unified Plotting (Combined Report) ---
    print("\nGenerating combined stability, rate & vibration report...")
    fig, axes = plt.subplots(4, 1, figsize=(12, 20))
    
    # 1. Roll Tracking
    if df_att is not None:
        ax = axes[0]
        t_att = df_att['timestamp'].values / 1e6
        ax.plot(t_att, df_att['roll_deg'].values, label='Actual', color='blue', alpha=0.7)
        ax.plot(t_att, df_att['roll_sp_deg'].values, label='Setpoint', color='orange', linestyle='--', alpha=0.9)
        ax.set_title(f"Attitude: Roll Tracking (RMSE: {att_stats['roll_rmse']:.4f}°)", fontsize=12, fontweight='bold')
        ax.set_ylabel('Roll (deg)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 2. Pitch Tracking
        ax = axes[1]
        ax.plot(t_att, df_att['pitch_deg'].values, label='Actual', color='green', alpha=0.7)
        ax.plot(t_att, df_att['pitch_sp_deg'].values, label='Setpoint', color='red', linestyle='--', alpha=0.9)
        ax.set_title(f"Attitude: Pitch Tracking (RMSE: {att_stats['pitch_rmse']:.4f}°)", fontsize=12, fontweight='bold')
        ax.set_ylabel('Pitch (deg)')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # 3. Rate Tracking (Roll)
    if df_rate is not None:
        ax = axes[2]
        t_rate = df_rate['timestamp'].values / 1e6
        ax.plot(t_rate, df_rate['roll_rate_deg'].values, label='Actual Roll Rate', color='purple', alpha=0.6)
        ax.plot(t_rate, df_rate['roll_rate_sp_deg'].values, label='Setpoint Roll Rate', color='cyan', linestyle='--', alpha=0.9)
        ax.set_title(f"Rate: Roll Rate Tracking (RMSE: {rate_stats['roll_rate_rmse']:.4f} deg/s)", fontsize=12, fontweight='bold')
        ax.set_ylabel('Rate (deg/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # 4. Vibration
    if df_vib is not None and not df_vib.empty:
        ax = axes[3]
        t_vib = df_vib['timestamp'].values / 1e6
        ax.plot(t_vib, df_vib['vibe_X'].values, label='X-axis', alpha=0.8)
        ax.plot(t_vib, df_vib['vibe_Y'].values, label='Y-axis', alpha=0.8)
        ax.plot(t_vib, df_vib['vibe_Z'].values, label='Z-axis', alpha=0.8)
        
        ax.set_title(f"IMU Vibration Levels (Mean: X={vib_stats['X']:.2f}m/s², Y={vib_stats['Y']:.2f}m/s², Z={vib_stats['Z']:.2f}m/s²)", fontsize=12, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Vibration (m/s²)')
        ax.legend()
        ax.grid(True, alpha=0.3)

    output_report = args.ulg_file + "_stability.png"
    plt.tight_layout()
    plt.savefig(output_report)
    print(f"Full report saved to: {output_report}")

if __name__ == "__main__":
    main()