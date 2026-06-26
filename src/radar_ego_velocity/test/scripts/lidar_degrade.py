#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 注意: 显式用系统 /usr/bin/python3 (ROS Noetic 解释器), 避免被 pyenv/conda 的 python3 截走
"""
lidar_degrade.py —— 人工制造 LiDAR 几何退化 (Livox CustomMsg), 用于验证雷达抗退化

思路:
  管廊/隧道的退化本质 = 沿轴向缺约束。轴向约束来自"前后端墙/特征"(方位角≈0°或180°);
  侧壁(方位角≈±90°)只约束横向。所以按方位角裁掉前后扇区, 只留侧向带,
  就能在普通数据上人工制造"轴向不可观", 让纯LiDAR沿X漂移。

模式 ~mode:
  "corridor"  : 只保留 ±90° 附近的侧向带 (去掉前后) -> 轴向(传感器X)退化  [默认]
  "fov"       : 只保留前向 ±halfwidth 的扇区 (缩小FOV)
  "passthrough": 不裁剪 (用于核对管线, 等价全视场)

参数:
  ~keep_halfwidth_deg : 每个保留扇区的半宽(度), 默认 30
  ~blind              : 近距盲区(m), 默认 0.5 (与FAST-LIO一致, 顺手清掉近点)
"""

import math
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class LidarDegrade(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "corridor")
        self.hw = math.radians(float(rospy.get_param("~keep_halfwidth_deg", 30.0)))
        self.blind = float(rospy.get_param("~blind", 0.5))
        in_topic = rospy.get_param("~input_topic", "/livox/lidar")
        out_topic = rospy.get_param("~output_topic", "/livox/lidar_deg")

        self.pub = rospy.Publisher(out_topic, CustomMsg, queue_size=10)
        rospy.Subscriber(in_topic, CustomMsg, self.cb, queue_size=10)
        self.n = 0
        rospy.loginfo("[lidar_degrade] mode=%s halfwidth=%.1fdeg %s -> %s",
                      self.mode, math.degrees(self.hw), in_topic, out_topic)

    def keep(self, x, y):
        """根据方位角决定是否保留该点。az=atan2(y,x), 前向为0。"""
        if self.mode == "passthrough":
            return True
        r2 = x * x + y * y
        if r2 < self.blind * self.blind:
            return False
        az = math.atan2(y, x)              # (-pi, pi]
        if self.mode == "fov":
            # 只留前向扇区
            return abs(az) <= self.hw
        # corridor: 保留 +90° 和 -90° 两个侧向带, 去掉前后
        d_left = abs(az - math.pi / 2.0)
        d_right = abs(az + math.pi / 2.0)
        return (d_left <= self.hw) or (d_right <= self.hw)

    def cb(self, msg):
        out = CustomMsg()
        out.header = msg.header
        out.timebase = msg.timebase
        out.lidar_id = msg.lidar_id
        out.rsvd = msg.rsvd
        kept = []
        for p in msg.points:
            if self.keep(p.x, p.y):
                kept.append(p)
        out.points = kept
        out.point_num = len(kept)
        self.pub.publish(out)

        self.n += 1
        if self.n % 50 == 0:
            ratio = (len(kept) / max(1, len(msg.points))) * 100.0
            rospy.loginfo("[lidar_degrade] frame %d: kept %d/%d (%.1f%%)",
                          self.n, len(kept), len(msg.points), ratio)


if __name__ == "__main__":
    rospy.init_node("lidar_degrade")
    LidarDegrade()
    rospy.spin()
