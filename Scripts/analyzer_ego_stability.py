#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
EGO-Planner Flight Stability Analyzer
-------------------------------------
Date: 2026-02-07
Description: 
    Analyzes ROS bag files to evaluate flight stability, tracking accuracy, 
    and yaw smoothness for drone navigation tasks.

    左下角图 (Yaw Smoothness)：这是验证我们修改的核心。
       * 蓝线 (Actual Yaw)：应该是平滑的曲线，没有锯齿。
       * 红虚线 (Desired Yaw)：蓝线要求是圆滑的，如果这还是直角台阶状，说明 TrajServer 收到的是直角指令。
       * 黄线 (Yaw Rate)：关注它的峰值。
   * 右上角图 (Velocity Profile)：
       * 看速度是否平稳。理想情况下，在过弯时速度应保持非零（Min-Snap 连续性）。
   * 右下角图 (Tracking Error)：
       * 如果误差长时间大于 0.2m ~ 0.3m，说明飞行速度可能过快，或者 PID 参数太软。
    
Usage:
    python3 Scripts/ego_flight_stability_analyzer.py <path_to_rosbag>

Dependencies:
    pip install bagpy matplotlib numpy scipy pandas
"""

import sys
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import argparse

class FlightAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.odom_topic = '/uav1/mavros/local_position/odom'
        self.cmd_topic = '/uav1/planning/ego/traj_cmd'
        
        self.odom_data = {'t': [], 'pos': [], 'vel': [], 'yaw': []}
        self.cmd_data = {'t': [], 'pos': [], 'yaw': []}
        
    def read_bag(self):
        print(f"Reading bag file: {self.bag_path}...")
        try:
            bag = rosbag.Bag(self.bag_path)
        except Exception as e:
            print(f"Error opening bag: {e}")
            return False

        # 1. Extract Odometry (Actual State)
        for topic, msg, t in bag.read_messages(topics=[self.odom_topic]):
            self.odom_data['t'].append(msg.header.stamp.to_sec())
            self.odom_data['pos'].append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            self.odom_data['vel'].append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            
            # Extract Yaw from Quaternion
            q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            r = R.from_quat(q)
            yaw = r.as_euler('zyx', degrees=False)[0]
            self.odom_data['yaw'].append(yaw)

        # 2. Extract Position Command (Desired State from Planner)
        # Note: quadrotor_msgs/PositionCommand usually has position, velocity, acceleration, and yaw
        has_cmd = False
        for topic, msg, t in bag.read_messages(topics=[self.cmd_topic]):
            has_cmd = True
            self.cmd_data['t'].append(msg.header.stamp.to_sec())
            self.cmd_data['pos'].append([msg.position.x, msg.position.y, msg.position.z])
            self.cmd_data['yaw'].append(msg.yaw)

        bag.close()

        if not self.odom_data['t']:
            print(f"No data found for topic {self.odom_topic}")
            return False
        
        # Convert to numpy arrays
        for k in self.odom_data: self.odom_data[k] = np.array(self.odom_data[k])
        for k in self.cmd_data: self.cmd_data[k] = np.array(self.cmd_data[k])
        
        # Normalize time to start at 0
        t0 = self.odom_data['t'][0]
        self.odom_data['t'] -= t0
        if has_cmd:
            self.cmd_data['t'] -= t0
        
        # Unwrap Yaw (handle -pi to pi jump)
        self.odom_data['yaw'] = np.unwrap(self.odom_data['yaw'])
        if has_cmd and len(self.cmd_data['yaw']) > 0:
            self.cmd_data['yaw'] = np.unwrap(self.cmd_data['yaw'])
            
        print(f"Loaded {len(self.odom_data['t'])} odom samples and {len(self.cmd_data['t'])} command samples.")
        return True

    def analyze_smoothness(self):
        # Calculate Velocity Magnitude (Actual)
        vel_mag = np.linalg.norm(self.odom_data['vel'], axis=1)
        
        # Calculate Yaw Rate (rad/s)
        dt = np.diff(self.odom_data['t'])
        dt = np.where(dt == 0, 1e-5, dt) # Avoid divide by zero
        yaw_rate = np.diff(self.odom_data['yaw']) / dt
        yaw_rate = np.insert(yaw_rate, 0, 0) # Pad to match length
        
        return vel_mag, yaw_rate

    def plot_results(self):
        vel_mag, yaw_rate = self.analyze_smoothness()
        
        fig = plt.figure(figsize=(16, 10))
        fig.suptitle(f"Flight Stability Analysis: {os.path.basename(self.bag_path)}", fontsize=16)

        # 1. 3D Trajectory (Top Left)
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(self.odom_data['pos'][:,0], self.odom_data['pos'][:,1], self.odom_data['pos'][:,2], label='Actual (Odom)', color='b', linewidth=1)
        if len(self.cmd_data['pos']) > 0:
            ax1.plot(self.cmd_data['pos'][:,0], self.cmd_data['pos'][:,1], self.cmd_data['pos'][:,2], label='Desired (Cmd)', color='r', linestyle='--', alpha=0.7)
        ax1.set_title("3D Flight Trajectory")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.legend()

        # 2. Velocity Profile (Top Right)
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(self.odom_data['t'], vel_mag, label='Velocity Magnitude', color='g')
        ax2.set_title("Velocity Smoothness Profile")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Speed (m/s)")
        ax2.grid(True)
        # Highlight stops (near zero velocity)
        stops = vel_mag < 0.1
        # ax2.fill_between(self.odom_data['t'], 0, 2, where=stops, color='red', alpha=0.1, label='Stop/Hover')
        ax2.legend()

        # 3. Yaw & Yaw Rate (Bottom Left) - CRITICAL FOR YOUR CHANGES
        ax3 = fig.add_subplot(2, 2, 3)
        ax3_twin = ax3.twinx()
        
        l1, = ax3.plot(self.odom_data['t'], np.degrees(self.odom_data['yaw']), 'b-', label='Yaw (deg)')
        if len(self.cmd_data['yaw']) > 0:
             # Interpolate cmd time to match odom time for better visual if needed, but plotting raw is fine
            l1_cmd, = ax3.plot(self.cmd_data['t'], np.degrees(self.cmd_data['yaw']), 'r--', alpha=0.6, label='Desired Yaw')
        
        l2, = ax3_twin.plot(self.odom_data['t'], np.degrees(yaw_rate), 'orange', alpha=0.5, label='Yaw Rate (deg/s)')
        
        ax3.set_title("Yaw Smoothness & Rate")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Yaw Angle (deg)")
        ax3_twin.set_ylabel("Yaw Rate (deg/s)")
        
        # Add threshold line for max_yaw_rate (e.g., 90 deg/s approx 1.5 rad/s)
        ax3_twin.axhline(y=90, color='k', linestyle=':', alpha=0.3)
        ax3_twin.axhline(y=-90, color='k', linestyle=':', alpha=0.3)
        
        lines = [l1, l2]
        if len(self.cmd_data['yaw']) > 0: lines.append(l1_cmd)
        ax3.legend(lines, [l.get_label() for l in lines], loc='upper left')
        ax3.grid(True)

        # 4. Tracking Error (Bottom Right)
        ax4 = fig.add_subplot(2, 2, 4)
        if len(self.cmd_data['pos']) > 0:
            # We need to interpolate command positions to odom timestamps to calculate error
            cmd_pos_interp_x = np.interp(self.odom_data['t'], self.cmd_data['t'], self.cmd_data['pos'][:,0])
            cmd_pos_interp_y = np.interp(self.odom_data['t'], self.cmd_data['t'], self.cmd_data['pos'][:,1])
            cmd_pos_interp_z = np.interp(self.odom_data['t'], self.cmd_data['t'], self.cmd_data['pos'][:,2])
            
            error_x = self.odom_data['pos'][:,0] - cmd_pos_interp_x
            error_y = self.odom_data['pos'][:,1] - cmd_pos_interp_y
            error_z = self.odom_data['pos'][:,2] - cmd_pos_interp_z
            total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            ax4.plot(self.odom_data['t'], total_error, 'k-', label='Total Error')
            ax4.plot(self.odom_data['t'], error_x, 'r--', alpha=0.3, label='X Err')
            ax4.plot(self.odom_data['t'], error_y, 'g--', alpha=0.3, label='Y Err')
            ax4.set_title("Tracking Accuracy (Error)")
            ax4.set_ylabel("Error (m)")
            ax4.legend()
        else:
            ax4.text(0.5, 0.5, "No Command Data Found", ha='center')

        plt.tight_layout()
        output_file = self.bag_path.replace('.bag', '_stability_report.png')
        plt.savefig(output_file)
        print(f"Analysis saved to: {output_file}")
        # plt.show() # Uncomment if running locally with GUI

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze EGO-Planner Drone Flight Stability')
    parser.add_argument('bag_file', help='Path to the ROS bag file')
    args = parser.parse_args()

    if not os.path.exists(args.bag_file):
        print(f"File not found: {args.bag_file}")
        sys.exit(1)

    analyzer = FlightAnalyzer(args.bag_file)
    if analyzer.read_bag():
        analyzer.plot_results()