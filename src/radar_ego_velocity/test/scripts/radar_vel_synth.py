#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 注意: 显式用系统 /usr/bin/python3 (ROS Noetic 解释器), 避免被 pyenv/conda 的 python3 截走
"""
radar_vel_synth.py —— 合成毫米波雷达自速度 /radar_velocity（无需真雷达，用于消融验证）

原理:
  参考轨迹(全视场、不开雷达跑出的 FAST-LIO /Odometry)在几何良好区域是准确的,
  其 twist.linear = 世界系速度 v_W, pose.orientation = R_WI。
  按雷达->IMU外参把 v_W 投到雷达系, 叠加高斯噪声, 当作"真雷达"会测到的自速度:

      v_radar = R_IS^T ( R_WI^T v_W + omega x t_IS )

  其中 omega 取自 IMU(去零偏前的原始角速度即可, 杆臂项很小)。
  这样在"退化run"里, 这部分速度信息对被裁剪的LiDAR是真实缺失的, 雷达确实在补约束。

用法:
  # 先把参考run录的 /Odometry 以 /ref_odom 重映射播放, 本节点订阅 /ref_odom
  rosbag play ref_odom.bag /Odometry:=/ref_odom
  rosrun radar_ego_velocity radar_vel_synth.py
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


def quat_to_R(qx, qy, qz, qw):
    """四元数 -> 旋转矩阵 R_WI (world<-imu)"""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


class RadarVelSynth(object):
    def __init__(self):
        # 雷达->IMU 外参: 默认与 mid360.yaml 的 radar 段一致, 可被参数覆盖
        ext_R = rospy.get_param("~extrinsic_R",
                                [0.70710678, 0.0, -0.70710678,
                                 0.0, 1.0, 0.0,
                                 0.70710678, 0.0, 0.70710678])
        ext_T = rospy.get_param("~extrinsic_T", [0.20, 0.0, -0.05])
        self.R_IS = np.array(ext_R, dtype=float).reshape(3, 3)  # IMU<-Radar
        self.t_IS = np.array(ext_T, dtype=float).reshape(3)

        # 观测噪声: 与 vel_cov 对齐 (std = sqrt(vel_cov)); 默认 0.04 -> 0.2 m/s
        vel_cov = float(rospy.get_param("~vel_cov", 0.04))
        self.noise_std = float(rospy.get_param("~noise_std", np.sqrt(vel_cov)))
        self.use_lever_arm = bool(rospy.get_param("~use_lever_arm", True))
        self.drop_z = bool(rospy.get_param("~drop_z", True))  # 2D雷达模拟: z速度置0
        seed = int(rospy.get_param("~seed", 12345))
        self.rng = np.random.default_rng(seed)

        in_topic = rospy.get_param("~ref_odom_topic", "/ref_odom")
        out_topic = rospy.get_param("~output_topic", "/radar_velocity")
        imu_topic = rospy.get_param("~imu_topic", "/livox/imu")

        self.omega = np.zeros(3)
        self.pub = rospy.Publisher(out_topic, TwistStamped, queue_size=50)
        rospy.Subscriber(imu_topic, Imu, self.imu_cb, queue_size=200)
        rospy.Subscriber(in_topic, Odometry, self.odom_cb, queue_size=50)

        self.n = 0
        rospy.loginfo("[radar_synth] ref=%s imu=%s -> out=%s | noise_std=%.3f lever_arm=%s drop_z=%s",
                      in_topic, imu_topic, out_topic, self.noise_std, self.use_lever_arm, self.drop_z)

    def imu_cb(self, msg):
        self.omega = np.array([msg.angular_velocity.x,
                               msg.angular_velocity.y,
                               msg.angular_velocity.z])

    def odom_cb(self, msg):
        v_W = np.array([msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z])
        q = msg.pose.pose.orientation
        R_WI = quat_to_R(q.x, q.y, q.z, q.w)

        v_imu = R_WI.T.dot(v_W)
        if self.use_lever_arm:
            v_imu = v_imu + np.cross(self.omega, self.t_IS)
        v_radar = self.R_IS.T.dot(v_imu)

        # 叠加观测噪声
        v_radar = v_radar + self.rng.normal(0.0, self.noise_std, 3)
        if self.drop_z:
            v_radar[2] = 0.0

        out = TwistStamped()
        out.header.stamp = msg.header.stamp   # 关键: 用参考帧时间戳, 保证与雷达帧时间匹配
        out.header.frame_id = "radar"
        out.twist.linear.x = float(v_radar[0])
        out.twist.linear.y = float(v_radar[1])
        out.twist.linear.z = float(v_radar[2])
        self.pub.publish(out)

        self.n += 1
        if self.n % 100 == 0:
            rospy.loginfo("[radar_synth] published %d msgs, last |v_radar|=%.3f m/s",
                          self.n, float(np.linalg.norm(v_radar)))


if __name__ == "__main__":
    rospy.init_node("radar_vel_synth")
    RadarVelSynth()
    rospy.spin()
