#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tree_drift_injector.py — 对 TreeDetection 中树干位置注入已知 2D 刚体变换漂移

模拟单架无人机里程计漂移后树干位置坐标系的偏移。
两台无人机各自注入不同的漂移，tree_loc_node 的 SVD 将估计两者间的相对漂移。

订阅: /uav{drone_id}/tree_detection  (原始树检测结果)
发布: /uav{drone_id}/tree_detection_drifted  (漂移后的树检测结果)

参数:
  ~drone_id:      无人机 ID
  ~drift_dx:      X 方向固定漂移 (m, 默认 0)
  ~drift_dy:      Y 方向固定漂移 (m, 默认 0)
  ~drift_dyaw_deg: Yaw 固定漂移 (deg, 默认 0)
  ~rate_dx:       X 漂移速率 (m/s, 默认 0)
  ~rate_dy:       Y 漂移速率 (m/s, 默认 0)
  ~rate_dyaw_deg: Yaw 漂移速率 (deg/s, 默认 0)
  ~noise_sigma:   位置高斯噪声 (m, 默认 0)

变换:
  tree_drifted = R(yaw_drift) * tree_original + (dx_drift, dy_drift)
"""

import rospy
from drone_detect_lidar.msg import Tree, TreeDetection
import math
import random
import numpy as np


class TreeDriftInjector:
    def __init__(self):
        rospy.init_node('tree_drift_injector', anonymous=True)

        drone_id = rospy.get_param('~drone_id', 1)
        self.drift_dx = rospy.get_param('~drift_dx', 0.0)
        self.drift_dy = rospy.get_param('~drift_dy', 0.0)
        self.drift_dyaw_deg = rospy.get_param('~drift_dyaw_deg', 0.0)
        self.rate_dx = rospy.get_param('~rate_dx', 0.0)
        self.rate_dy = rospy.get_param('~rate_dy', 0.0)
        self.rate_dyaw_deg = rospy.get_param('~rate_dyaw_deg', 0.0)
        self.noise_sigma = rospy.get_param('~noise_sigma', 0.0)

        self.drift_dyaw = math.radians(self.drift_dyaw_deg)
        self.rate_dyaw = math.radians(self.rate_dyaw_deg)
        self.start_time = None

        in_topic = '/uav{}/tree_detection'.format(drone_id)
        out_topic = '/uav{}/tree_detection_drifted'.format(drone_id)

        self.pub = rospy.Publisher(out_topic, TreeDetection, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, TreeDetection, self._callback)

        has_drift = (abs(self.drift_dx) > 1e-9 or abs(self.drift_dy) > 1e-9 or
                     abs(self.drift_dyaw_deg) > 1e-9 or
                     abs(self.rate_dx) > 1e-9 or abs(self.rate_dy) > 1e-9 or
                     abs(self.rate_dyaw_deg) > 1e-9 or abs(self.noise_sigma) > 1e-9)

        rospy.loginfo('[TreeDriftInjector] UAV{}: {} → {}  drift={}'.format(
            drone_id, in_topic, out_topic, has_drift))
        if has_drift:
            rospy.logwarn('[TreeDriftInjector] UAV{} drift: '
                          'bias=({:.3f}m, {:.3f}m, {:.2f}deg) '
                          'rate=({:.3f}m/s, {:.3f}m/s, {:.2f}deg/s) '
                          'noise_sigma={:.3f}m'.format(
                              drone_id,
                              self.drift_dx, self.drift_dy, self.drift_dyaw_deg,
                              self.rate_dx, self.rate_dy, self.rate_dyaw_deg,
                              self.noise_sigma))

    def _get_current_drift(self):
        """计算当前时刻的累积漂移量."""
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        t = (rospy.Time.now() - self.start_time).to_sec()

        dx = self.drift_dx + self.rate_dx * t
        dy = self.drift_dy + self.rate_dy * t
        dyaw = self.drift_dyaw + self.rate_dyaw * t

        if self.noise_sigma > 0:
            dx += random.gauss(0, self.noise_sigma)
            dy += random.gauss(0, self.noise_sigma)

        return dx, dy, dyaw

    def _transform_tree(self, tree, dx, dy, yaw):
        """对单棵树施加 2D 刚体变换: tree' = R(yaw) * tree + (dx, dy)."""
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = c * tree.x - s * tree.y + dx
        ty = s * tree.x + c * tree.y + dy

        drifted = Tree()
        drifted.id = tree.id
        drifted.x = tx
        drifted.y = ty
        drifted.z_base = tree.z_base  # Z 不变
        drifted.height = tree.height
        drifted.diameter = tree.diameter
        drifted.linearity = tree.linearity
        drifted.planarity = tree.planarity
        drifted.roundness = tree.roundness
        drifted.confidence = tree.confidence
        return drifted

    def _callback(self, msg):
        """收到原始 TreeDetection，注入漂移后转发."""
        dx, dy, dyaw = self._get_current_drift()

        drifted = TreeDetection()
        drifted.header = msg.header
        drifted.drone_id = msg.drone_id
        drifted.odometry = msg.odometry
        drifted.trees = [self._transform_tree(t, dx, dy, dyaw)
                         for t in msg.trees]
        self.pub.publish(drifted)


if __name__ == '__main__':
    TreeDriftInjector()
    rospy.spin()
