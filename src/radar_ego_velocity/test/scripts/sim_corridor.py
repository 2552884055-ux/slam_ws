#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 显式系统解释器, 避免 pyenv/conda 截走
"""
sim_corridor.py —— 生成「直管廊」LiDAR-惯导仿真 rosbag (无需 Gazebo)

场景: 一条沿 X 轴的直管廊, 只有平滑的左右侧壁 + 地面 + 天花板, 没有任何沿轴向(X)特征。
      => 侧壁约束 Y/偏航, 地面天花板约束 Z/俯仰滚转, 但 X 平移无约束 => 经典退化方向。
      纯 LiDAR 会沿 X 漂移; 雷达自速度可补 X 约束。

输出话题:
  /livox/lidar (livox_ros_driver2/CustomMsg)  10 Hz
  /livox/imu   (sensor_msgs/Imu)             200 Hz
  /gt_odom     (nav_msgs/Odometry)            10 Hz  真值(evo参考 + 雷达合成源)

用法:
  /usr/bin/python3 sim_corridor.py -o corridor.bag
"""

import argparse
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint

G = 9.81  # 重力


def velocity_profile(t, t_static, v_max, a):
    """梯形速度剖面: 两端静止, 中间匀速, 平滑加减速。返回 (x, v, acc)。"""
    t_ramp = v_max / a                  # 加/减速时长
    d_ramp = 0.5 * a * t_ramp ** 2      # 加/减速位移
    t1 = t_static                       # 开始加速
    t2 = t1 + t_ramp                    # 到匀速
    # 巡航时长由总时长反推, 这里用外部传入的 cruise 段
    return t1, t2, t_ramp, d_ramp


class CorridorSim(object):
    def __init__(self, args):
        self.args = args
        self.rng = np.random.default_rng(args.seed)
        # 管廊几何
        self.half_w = args.half_width      # 侧壁 y = ±half_w
        self.z_floor = -args.cam_height    # 地面相对传感器
        self.z_ceil = args.ceil_height - args.cam_height
        self.R = args.lidar_range
        self.blind = 0.5
        self.v_max = args.speed
        self.acc = 0.5
        # 传感器误差 (与 mid360.yaml 量级一致)
        self.acc_noise = args.acc_noise
        self.gyr_noise = args.gyr_noise
        self.acc_bias = np.array([args.acc_bias_x, 0.0, 0.0])  # X向小零偏 -> 退化方向可见漂移
        self.lidar_noise = 0.01
        # 行进起点(世界系); 合成几何时为原点, PCD几何时为隧道入口中心
        self.path_origin = np.zeros(3)
        self.travel_len = args.length      # PCD几何会覆盖为实际隧道可行段长
        self.build_geometry()
        # 运动剖面: 静止-加速-匀速-减速-静止 (依据 travel_len)
        self.t_static = 2.0
        self.t_ramp = self.v_max / self.acc
        self.d_ramp = 0.5 * self.acc * self.t_ramp ** 2
        self.cruise_len = max(0.5, self.travel_len - 2 * self.d_ramp)
        self.t_cruise = self.cruise_len / self.v_max
        self.t_move = 2 * self.t_ramp + self.t_cruise
        self.T = 2 * self.t_static + self.t_move
        print("[sim] 行进长 %.1fm, 速度 %.2fm/s, 总时长 %.1fs, 持久点 %d, 起点 %s"
              % (self.travel_len, self.v_max, self.T, len(self.geo), np.round(self.path_origin, 2)))

    # ---- 几何来源: 真实PCD 或 合成管廊 ----
    def build_geometry(self):
        if self.args.pcd:
            self.load_pcd_geometry()
        else:
            self.build_synthetic()

    # ---- 从真实 PCD 加载隧道几何, 自动定轴线/裁剪/抽稀 ----
    def load_pcd_geometry(self):
        with open(self.args.pcd, 'rb') as f:
            head = f.read(512)
        off = head.find(b'DATA binary\n') + len(b'DATA binary\n')
        with open(self.args.pcd, 'rb') as f:
            f.seek(off); data = np.fromfile(f, dtype=np.float32)
        n = data.size // 8                              # FIELDS: x y z intensity nx ny nz curvature
        xyz = data[:n * 8].reshape(n, 8)[:, :3]
        ok = np.isfinite(xyz).all(1) & (np.abs(xyz) < 500).all(1)
        xyz = xyz[ok]
        # 主轴=X(已验证). 用分位裁掉离群, 保留隧道主体
        xlo, xhi = np.percentile(xyz[:, 0], [5, 95])
        ylo, yhi = np.percentile(xyz[:, 1], [2, 98])
        zlo, zhi = np.percentile(xyz[:, 2], [2, 98])
        box = ((xyz[:, 0] > xlo - self.R) & (xyz[:, 0] < xhi + self.R) &
               (xyz[:, 1] > ylo) & (xyz[:, 1] < yhi) &
               (xyz[:, 2] > zlo) & (xyz[:, 2] < zhi))
        xyz = xyz[box]
        # 行进范围(沿X), 默认走主体分位段
        x0 = self.args.x_start if self.args.x_start is not None else xlo
        x1 = self.args.x_end if self.args.x_end is not None else xhi
        self.travel_len = x1 - x0
        # 路径横/纵位置 = 隧道中心(行进段内点的中位Y, 地面之上cam_height)
        near = xyz[(xyz[:, 0] > x0) & (xyz[:, 0] < x1)]
        y0 = float(np.median(near[:, 1]))
        zf = float(np.percentile(near[:, 2], 3))
        self.path_origin = np.array([x0, y0, zf + self.args.cam_height])
        # 抽稀到 ~30万点(密度足够且每帧渲染快)
        if len(xyz) > 300000:
            sel = self.rng.choice(len(xyz), 300000, replace=False)
            xyz = xyz[sel]
        self.geo = xyz.astype(float)

    # ---- 合成「持久」管廊点云: 平滑平面, X零梯度=>干净退化 ----
    def build_synthetic(self):
        step = 0.2
        L0, L1 = -2.0, self.args.length + 2.0
        xs = np.arange(L0, L1, step)
        zs = np.arange(self.z_floor, self.z_ceil, step)
        ys = np.arange(-self.half_w, self.half_w, step)
        g = []
        # 左右壁: (x, ±W, z)
        XX, ZZ = np.meshgrid(xs, zs)
        for sgn in (+1, -1):
            g.append(np.stack([XX.ravel(), np.full(XX.size, sgn * self.half_w), ZZ.ravel()], 1))
        # 地面/天花板: (x, y, z固定)
        XX2, YY = np.meshgrid(xs, ys)
        for zf in (self.z_floor, self.z_ceil):
            g.append(np.stack([XX2.ravel(), YY.ravel(), np.full(XX2.size, zf)], 1))
        geo = np.concatenate(g, 0)
        # 固定微小起伏(持久纹理), 不改变平面法向但让匹配稳定可复现
        jit = self.rng.normal(0, 0.005, geo.shape)
        self.geo = geo + jit

    # ---- 运动学: 给定时间返回 (px, vx, ax) ----
    def kinematics(self, t):
        ts, tr, tc = self.t_static, self.t_ramp, self.t_cruise
        if t < ts:                                  # 静止
            return 0.0, 0.0, 0.0
        td = t - ts
        if td < tr:                                 # 加速
            return 0.5 * self.acc * td ** 2, self.acc * td, self.acc
        td -= tr
        if td < tc:                                 # 匀速
            return self.d_ramp + self.v_max * td, self.v_max, 0.0
        td -= tc
        if td < tr:                                 # 减速
            x = self.d_ramp + self.cruise_len + self.v_max * td - 0.5 * self.acc * td ** 2
            return x, self.v_max - self.acc * td, -self.acc
        # 末段静止
        return self.travel_len, 0.0, 0.0

    # ---- 生成一帧 LiDAR 点 (传感器系): 渲染持久点云中范围内可见部分 ----
    def make_lidar(self, px, t0):
        # 传感器系(姿态恒等): 机器人世界位置 = 起点 + 沿X行进px
        d = self.geo - (self.path_origin + np.array([px, 0.0, 0.0]))
        r2 = (d ** 2).sum(1)
        m = (r2 > self.blind ** 2) & (r2 < self.R ** 2)
        pts = d[m] + self.rng.normal(0, self.lidar_noise, (int(m.sum()), 3))
        # 每帧抽稀到接近真实 LiDAR 的点数(MID360 ~2万/帧)
        if len(pts) > self.args.max_points:
            sel = self.rng.choice(len(pts), self.args.max_points, replace=False)
            pts = pts[sel]
        else:
            idx = np.arange(len(pts)); self.rng.shuffle(idx); pts = pts[idx]

        msg = CustomMsg()
        msg.header.stamp = rospy.Time.from_sec(t0)
        msg.header.frame_id = "livox"
        msg.timebase = int(t0 * 1e9)
        msg.lidar_id = 1
        msg.rsvd = [0, 0, 0]
        n = len(pts)
        dt_ns = int(0.1 * 1e9 / max(1, n))   # 0.1s 帧内均匀分布 offset_time
        cps = []
        for i in range(n):
            cp = CustomPoint()
            cp.offset_time = i * dt_ns
            cp.x, cp.y, cp.z = float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2])
            cp.reflectivity = 120
            cp.tag = 0
            cp.line = i % 4          # N_SCANS=4
            cps.append(cp)
        msg.points = cps
        msg.point_num = n
        return msg

    def make_imu(self, t, ax):
        m = Imu()
        m.header.stamp = rospy.Time.from_sec(t)
        m.header.frame_id = "livox"
        # 姿态恒等: 比力 = a_world - g_world, g_world=(0,0,-G)
        a = np.array([ax, 0.0, G]) + self.acc_bias
        a = a + self.rng.normal(0, self.acc_noise, 3)
        w = self.rng.normal(0, self.gyr_noise, 3)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
        m.orientation.w = 1.0
        return m

    def make_gt(self, t, px, vx):
        o = Odometry()
        o.header.stamp = rospy.Time.from_sec(t)
        o.header.frame_id = "camera_init"
        o.child_frame_id = "body"
        o.pose.pose.position.x = px
        o.pose.pose.orientation.w = 1.0
        o.twist.twist.linear.x = vx
        return o

    def run(self, out_path):
        t_base = 1000.0
        imu_dt = 1.0 / 200.0
        lidar_dt = 1.0 / 10.0
        gt_dt = 1.0 / 50.0          # 真值/雷达源 50Hz: 保证任意 lidar_end_time 附近都有样本可匹配
        events = []                 # (t, kind)
        n_imu = int(self.T / imu_dt)
        for k in range(n_imu):
            events.append((t_base + k * imu_dt, "imu"))
        n_lid = int(self.T / lidar_dt)
        for k in range(n_lid):
            events.append((t_base + k * lidar_dt, "lidar"))
        n_gt = int(self.T / gt_dt)
        for k in range(n_gt):
            events.append((t_base + k * gt_dt, "gt"))
        events.sort(key=lambda e: e[0])

        bag = rosbag.Bag(out_path, "w")
        n_pts_last = 0
        for t, kind in events:
            tt = t - t_base
            px, vx, ax = self.kinematics(tt)
            stamp = rospy.Time.from_sec(t)
            if kind == "imu":
                bag.write("/livox/imu", self.make_imu(t, ax), stamp)
            elif kind == "gt":
                bag.write("/gt_odom", self.make_gt(t, px, vx), stamp)
            else:
                lm = self.make_lidar(px, t)
                n_pts_last = lm.point_num
                bag.write("/livox/lidar", lm, stamp)
        bag.close()
        print("[sim] 写入 %s  (LiDAR帧≈%d, 每帧点≈%d, IMU≈%d, GT≈%d@50Hz)"
              % (out_path, n_lid, n_pts_last, n_imu, n_gt))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="corridor.bag")
    ap.add_argument("--length", type=float, default=15.0)        # 管廊长(m)
    ap.add_argument("--half_width", type=float, default=1.5)     # 半宽(m)
    ap.add_argument("--ceil_height", type=float, default=2.5)    # 净高(m)
    ap.add_argument("--cam_height", type=float, default=1.0)     # 传感器离地(m)
    ap.add_argument("--lidar_range", type=float, default=10.0)   # 量程(m)
    ap.add_argument("--speed", type=float, default=1.0)          # 巡航速度(m/s)
    ap.add_argument("--points_per_face", type=int, default=400)  # 每面采样点
    ap.add_argument("--acc_noise", type=float, default=0.06)     # 加速度计噪声
    ap.add_argument("--gyr_noise", type=float, default=0.003)    # 陀螺噪声
    ap.add_argument("--acc_bias_x", type=float, default=0.02)    # X向加速度零偏(制造退化漂移)
    ap.add_argument("--seed", type=int, default=2024)
    # ---- 用真实PCD作几何(可选): 给定则忽略合成管廊 ----
    ap.add_argument("--pcd", type=str, default=None)             # 真实点云地图路径
    ap.add_argument("--x_start", type=float, default=None)       # 沿X行进起点(默认主体5%分位)
    ap.add_argument("--x_end", type=float, default=None)         # 沿X行进终点(默认主体95%分位)
    ap.add_argument("--max_points", type=int, default=20000)     # 每帧最大点数(贴近真实LiDAR)
    args = ap.parse_args()
    CorridorSim(args).run(args.out)


if __name__ == "__main__":
    main()
