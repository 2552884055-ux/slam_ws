# slam_ws —— 毫米波雷达辅助抗退化激光惯性 SLAM 工作空间

面向 **管廊 / 隧道 / 长走廊** 等激光退化场景的无人机 / 机器人定位工作空间。核心思路：在 FAST-LIO2 激光惯性里程计的基础上，引入 **毫米波雷达多普勒自速度** 作为速度观测，紧耦合补偿激光沿轴向的退化漂移；并提供先验地图全局重定位能力。

---

## 1. 工作空间组成

本工作空间（catkin）包含三个 ROS 包，构成「传感器驱动 → 雷达自速度估计 → 激光惯性融合」的完整链路：

| 包 | 作用 | 详细文档 |
|----|------|----------|
| **[livox_ros_driver2](src/livox_ros_driver2/)** | Livox MID360 等激光雷达 ROS 驱动，发布 `/livox/lidar` 与 `/livox/imu` | [README](src/livox_ros_driver2/README.md) |
| **[radar_ego_velocity](src/radar_ego_velocity/)** | 毫米波雷达(SR71)驱动 + RANSAC 自速度估计，把雷达点云解算为雷达系自速度 `/radar_velocity` | [README](src/radar_ego_velocity/README.md) |
| **[FAST_LIO_GLOBAL](src/FAST_LIO_GLOBAL/)** | FAST-LIO2 激光惯性里程计 + 雷达速度抗退化融合 + 全局重定位 | [README](src/FAST_LIO_GLOBAL/README.md) |

## 2. 整体数据流

```
                         ┌──────────────────────────┐
  Livox MID360 ─────────►│   livox_ros_driver2       │── /livox/lidar (点云) ──┐
   (LiDAR + IMU)         └──────────────────────────┘── /livox/imu   (IMU)  ──┤
                                                                              │
  SR71 毫米波雷达 ──串口──►┌──────────────────┐ /radar_scan ┌──────────────────┐│
   (TTL / 可选CAN)        │ sr71_serial_node │────────────►│ radar_ego_velocity│
                          │  (解析雷达帧)     │  RadarScan  │  (RANSAC估自速度) ││
                          └──────────────────┘             └─────────┬─────────┘│
                                                /radar_velocity      │          │
                                                 (TwistStamped)      ▼          ▼
                                                          ┌────────────────────────────┐
                                                          │   FAST_LIO_GLOBAL           │
                                                          │   (laserMapping)            │
                                                          │  IESKF激光主更新            │
                                                          │  + 雷达速度紧耦合更新(可选) │
                                                          └─────────────┬──────────────┘
                                                                        ▼
                                              /Odometry(位姿+速度), /cloud_registered, /path, TF
```

要点：
- 雷达 **不直接** 进 FAST-LIO，而是先估出 **雷达系自速度（对地速度）**，再作为速度观测融合；
- 融合发生在激光 IESKF 更新 **之后**（`laserMapping.cpp` 的 `fuse_radar_velocity()`），属于状态级紧耦合，连带修正速度、姿态与陀螺零偏；
- 退化方向（沿廊轴前进）由雷达速度约束，从而抑制激光里程计的轴向漂移。

## 3. 依赖

- Ubuntu 20.04 + **ROS Noetic**，C++14、OpenMP
- **PCL ≥ 1.8**、**Eigen ≥ 3.3.4**
- 全局重定位（可选）：`pip3 install open3d ros_numpy`

> 注：`radar_ego_velocity` 的 CAN 节点（`sr71_can_node`，依赖 SocketCAN）只能在 Linux 上编译运行；Windows 开发机上仅用于查看 / 编辑源码。

## 4. 编译

```bash
cd ~/Desktop/slam_ws/slam_ws
catkin_make            # 或 catkin build
source devel/setup.bash
```
> 各包关键可执行：`livox_ros_driver2`、`radar_ego_velocity_node` / `sr71_serial_node`、`fastlio_mapping_global`。

## 5. 运行

### 5.1 一键启动（建图 + 雷达融合，推荐）
```bash
roslaunch radar_ego_velocity mapping_mid360_with_radar.launch
```
该 launch 同时拉起：FAST-LIO 建图（`mapping_mid360.launch`）+ 雷达自速度估计链路（`radar_ego_velocity.launch`）。FAST-LIO 默认订阅 `/radar_velocity`，无需重映射。

### 5.2 分步启动
```bash
# 终端1：激光雷达驱动
roslaunch livox_ros_driver2 msg_MID360.launch
# 终端2：雷达自速度估计（需先接好 SR71）
roslaunch radar_ego_velocity radar_ego_velocity.launch
# 终端3：FAST-LIO（含融合）
roslaunch fast_lio_global mapping_mid360.launch
```

### 5.3 全局重定位（已有先验 PCD 地图时）
```bash
roslaunch fast_lio_global global_localization_mid360.launch
rosrun fast_lio_global publish_initial_pose.py 0 -0.5 0 0 0 0   # 初始位姿先验
```

### 5.4 仅纯激光（关闭雷达）
编辑 `src/FAST_LIO_GLOBAL/config/mid360.yaml`，将 `radar/radar_en` 设为 `false`，系统即等价于原版 FAST-LIO2，便于 A/B 对比。

## 6. 部署前必须完成的标定 / 适配

整条链路能否在退化环境正常工作，关键在代码之外的这几项（详见各包 README）：

1. **SR71 串口协议解析**：按雷达厂商协议补全 `src/radar_ego_velocity/src/sr71_serial_node.cpp` 的帧格式、字段偏移与缩放，以及 launch 里的 `port` / `baudrate`。
2. **雷达 → IMU 外参**：`config/mid360.yaml` 的 `radar/extrinsic_T`（杆臂）与 `extrinsic_R`（含机头 45° 安装角），**必须实测标定**——平移误差在自转/悬停时尤其敏感。
3. **多普勒符号约定**：`radar_ego_velocity.launch` 的 `doppler_sign`，前进时输出速度应指向正前方，反了则改符号。
4. **激光 → IMU 外参 / 时间同步**：`mapping/extrinsic_*` 与时间偏移。

> FAST-LIO 侧已内置「自转/悬停自适应降权」机制（`radar/min_trans_vel`），显著降低了低速工况对杆臂标定误差的敏感性；但外参与符号仍需正确标定。

## 7. 验证建议

- **A/B 对比**：`radar_en` 开/关，对比退化路段 `/Odometry` 漂移；
- **分工况**：加速前进、匀速前进、左右自转、悬停、转弯，重点确认 **原地自转时位置基本不动、速度≈0**；
- **健康检查**：观察卡方残差 `chi2` 应集中在小值，频繁触顶说明外参 / 符号 / 时间同步有问题。

## 8. 目录结构

```
slam_ws/slam_ws/
├── src/
│   ├── livox_ros_driver2/     # Livox 激光雷达驱动
│   ├── radar_ego_velocity/    # SR71 雷达驱动 + 自速度估计
│   │   ├── src/  (radar_ego_velocity_node / sr71_serial_node / sr71_can_node)
│   │   ├── msg/  (RadarScan, RadarTarget)
│   │   └── launch/
│   └── FAST_LIO_GLOBAL/       # FAST-LIO2 + 雷达融合 + 全局重定位
│       ├── src/  (laserMapping.cpp ...)
│       ├── config/ (mid360.yaml ...)
│       ├── scripts/ (global_localization.py ...)
│       └── launch/
└── README.md                  # 本文件
```

## 9. 致谢

- **FAST-LIO / FAST-LIO2**、**ikd-Tree**、**IKFoM**（HKU-MaRS）
- **livox_ros_driver2**（Livox）
- 全局重定位参考 **FAST_LIO_LOCALIZATION**
- 各包许可证见其各自目录（FAST-LIO 为 BSD）。
