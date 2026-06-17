# FAST-LIO-GLOBAL（雷达速度抗退化 + 全局重定位）

> 基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的激光惯性里程计扩展工程。在原版 LIO 的基础上集成了三项能力：
> 1. **毫米波雷达多普勒速度紧耦合融合**——解决长直管廊 / 隧道 / 走廊等场景下激光沿轴向的退化问题；
> 2. **先验点云地图全局重定位**（FAST-LIO-LOCALIZATION 思路，基于 Open3D ICP）；
> 3. **2D 栅格地图构建与机器人位置 Socket 上报**。
>
> 原版 FAST-LIO 的上游说明保留在 [`README.fastlio-upstream.md`](README.fastlio-upstream.md)。

---

## 目录

- [1. 功能概述](#1-功能概述)
- [2. 系统架构与数据流](#2-系统架构与数据流)
- [3. 依赖与编译](#3-依赖与编译)
- [4. 快速开始](#4-快速开始)
- [5. 话题接口](#5-话题接口)
- [6. 配置参数说明](#6-配置参数说明)
- [7. 毫米波雷达速度融合（抗退化）](#7-毫米波雷达速度融合抗退化)
- [8. 全局重定位](#8-全局重定位)
- [9. 调参与故障排查](#9-调参与故障排查)
- [10. 目录结构](#10-目录结构)
- [11. 致谢与许可](#11-致谢与许可)

---

## 1. 功能概述

| 模块 | 说明 | 关键文件 |
|------|------|----------|
| 激光惯性里程计 | FAST-LIO2 的 IESKF + ikd-Tree，输出高频里程计与点云地图 | `src/laserMapping.cpp` |
| **雷达速度融合** | 在 IESKF 之后追加一次雷达多普勒自速度观测更新，约束激光退化的前进方向；含杆臂补偿、卡方门限、自转/悬停自适应降权 | `src/laserMapping.cpp`（`fuse_radar_velocity`） |
| 全局重定位 | 用当前扫描与先验 PCD 地图做 Open3D ICP，估计 `map → camera_init` 变换并实时融合 | `scripts/global_localization.py`、`scripts/transform_fusion.py` |
| 2D 建图 / 上报 | 点云转 LaserScan 构建 2D 栅格地图；机器人位置经 TF 与 Socket 对外发布 | `launch/Pointcloud2Map.launch` 等 |

> 雷达融合通过参数 `radar/radar_en` 完全隔离：设为 `false` 时系统等价于原版 FAST-LIO2。

## 2. 系统架构与数据流

```
 /livox/lidar ─┐
 /livox/imu  ──┤→ [laserMapping] ── IESKF 激光主更新 ──┐
 /radar_velocity ┘     (含点云预处理/去畸变/ikd-Tree)   │
                                                       ├─ 雷达速度更新(可选) ─→ /Odometry, /path
                                                       │                         /cloud_registered ...
                                            ┌──────────┘
                                            ▼
                         [global_localization.py]  当前扫描 vs 先验PCD地图 (ICP)
                                            │  发布 /map_to_odom
                                            ▼
                         [transform_fusion.py]  融合 map→odom→base，输出全局位姿与TF
```

- **建图模式**（`mapping_mid360.launch`）：纯 LIO + 雷达融合，实时建图，可选 2D 栅格图与位置上报。
- **重定位模式**（`global_localization_mid360.launch`）：加载先验 PCD 地图，启动 ICP 重定位三件套，在已知地图中输出全局一致位姿。

## 3. 依赖与编译

### 3.1 系统环境
- Ubuntu 20.04 + **ROS Noetic**
- C++14、CMake ≥ 3.0、OpenMP

### 3.2 C++ 依赖
- **PCL ≥ 1.8**
- **Eigen ≥ 3.3.4**
- **[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)**（必须先编译并 source）

### 3.3 Python 依赖（仅重定位模式需要）
```bash
pip3 install open3d ros_numpy
# 若 ros_numpy 缺失：sudo apt install ros-noetic-ros-numpy
```

### 3.4 外部 ROS 包（launch 中引用，按需提供）
`pointcloud_to_laserscan`、`map_server`、`pcl_ros`，以及私有包 `map_switch` / `pointcloud_tf`（位置 Socket 上报）。若不需要 2D 建图/上报，可在 launch 中关闭对应 `arg`。

### 3.5 编译
```bash
cd ~/slam_ws
catkin_make            # 或 catkin build
source devel/setup.bash
```
> 可执行文件名为 **`fastlio_mapping_global`**（见 `CMakeLists.txt`）。

## 4. 快速开始

### 建图模式
```bash
# 终端1：启动 Livox 驱动（发布 /livox/lidar 与 /livox/imu）
roslaunch livox_ros_driver2 msg_MID360.launch

# 终端2：启动建图
roslaunch fast_lio_global mapping_mid360.launch
```
建图结束若开启 `pcd_save/pcd_save_en`，点云保存在 `PCD/` 下。

### 重定位模式
```bash
# 1) 准备先验地图：将建图得到的 .pcd 放到 PCD/ 并在 launch 中指定 3dmap_path
# 2) 启动
roslaunch fast_lio_global global_localization_mid360.launch
# 3) 给定初始位姿（x y z yaw pitch roll），例：
rosrun fast_lio_global publish_initial_pose.py 0 -0.5 0 0 0 0
```

## 5. 话题接口

### 订阅
| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` 或 `sensor_msgs/PointCloud2` | 雷达点云（按 `lidar_type` 选择回调） |
| `/livox/imu` | `sensor_msgs/Imu` | IMU |
| `/radar_velocity` | `geometry_msgs/TwistStamped` | **毫米波雷达系自速度**（外部节点经 RANSAC 估计后发布，`radar_en=true` 时订阅） |

### 发布
| 话题 | 类型 | 说明 |
|------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | 位姿 + **速度（twist，已含雷达校正）** |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 世界系配准点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | IMU 系点云 |
| `/path` | `nav_msgs/Path` | 轨迹 |
| `/map_to_odom` | `nav_msgs/Odometry` | 重定位输出（Python 节点） |

坐标系：`camera_init`（世界/里程计起点）→ `body`（IMU）；重定位模式增加 `map` / `pcd_map`。

## 6. 配置参数说明

主配置在 `config/mid360.yaml`（另有 `avia/horizon/ouster64/velodyne.yaml`）。

### common / preprocess / mapping（原版 FAST-LIO）
| 参数 | 含义 |
|------|------|
| `common/lid_topic`, `common/imu_topic` | 输入话题名 |
| `preprocess/lidar_type` | 1=Livox, 2=Velodyne, 3=Ouster |
| `preprocess/blind` | 近距盲区(m) |
| `mapping/extrinsic_T`, `mapping/extrinsic_R` | **激光→IMU 外参** |
| `mapping/det_range`, `fov_degree` | 探测范围 / 视场角 |
| `filter_size_surf`, `filter_size_map` | 降采样体素大小 |
| `pcd_save/pcd_save_en`, `interval` | 是否保存点云地图 |

### radar（本工程新增）
| 参数 | 默认 | 含义 |
|------|------|------|
| `radar/radar_en` | `true` | **雷达速度融合总开关**（`false`=纯 FAST-LIO） |
| `radar/radar_topic` | `/radar_velocity` | 雷达自速度话题 |
| `radar/vel_cov` | `0.04` | 水平速度观测噪声 σ²，越小越信任雷达 |
| `radar/vel_cov_z` | `1e6` | 垂直速度噪声；2D 雷达设很大=忽略 z |
| `radar/chi2_thr` | `16.0` | 卡方门限（3 自由度），残差过大拒绝该帧 |
| `radar/time_tol` | `0.05` | 雷达与激光帧时间匹配容差(s) |
| `radar/min_trans_vel` | `0.3` | **平移速度阈值(m/s)**：自转/悬停时平滑降权雷达 |
| `radar/extrinsic_T` | — | **雷达→IMU 平移（杆臂）**，必须实测标定 |
| `radar/extrinsic_R` | — | **雷达→IMU 旋转**（含安装角，如机头 45°朝下），必须实测标定 |

> ⚠️ 配置中的雷达外参示例仅为示意，**务必按实际安装标定后替换**，否则会向退化方向注入错误速度。

## 7. 毫米波雷达速度融合（抗退化）

### 7.1 解决的问题
长直管廊中，两侧墙/地/顶面约束了横向、垂直与三个姿态角，但**沿廊轴的平移在点到面残差中几乎无信息**（退化方向），激光里程计会沿轴向漂移。毫米波雷达基于多普勒可直接测量传感器自速度，**恰好补上这个退化方向的速度约束**。

### 7.2 算法流程（`fuse_radar_velocity`）
在激光 IESKF 收敛后，以收敛状态/协方差为先验，做一次标准 EKF 速度观测更新：

1. **观测模型**（雷达系自速度，含杆臂补偿）：
   `h = R_IS^T · (R_WI^T · v_W + ω × t_IS)`
2. **雅可比 H(3×23)**：仅 `vel(列12)`、`rot(列3)`、`bg(列15)` 三块非零（与 IKFoM 右扰动约定一致）。
3. **自转/悬停自适应降权**：从测量剔除杆臂项得真实平移速度 `v_t`，按 `w = v_t²/(v_t²+min_trans_vel²)` 平滑放大水平噪声——前进时正常信任雷达，原地自转/悬停时雷达被软关闭，**避免杆臂项注入“幽灵速度”**。
4. **卡方门限**：`r^T S⁻¹ r > chi2_thr` 时拒绝（挡动态目标/异常帧）。
5. **EKF 更新**：`K=PH^T S⁻¹`，`s.boxplus(K·r)`，`P=(I−KH)P` 后**对称化**防数值退化。

### 7.3 多工况行为
| 工况 | 平移速度 | 雷达作用 |
|------|----------|----------|
| 加速 / 匀速前进 | 大 | 正常信任，牢固约束退化方向 |
| 转弯（边走边转） | 中 | 按比例平滑过渡 |
| 原地左右自转 / 悬停 | ≈0 | 自动软关闭，交给激光+IMU |

### 7.4 上游雷达节点约定
本工程**只消费** `/radar_velocity`（`TwistStamped`，雷达系自速度）。该话题需由独立节点（如 CAN 解析 + RANSAC 自速度估计）提供，且**速度符号、外参、时间戳**需与本系统约定一致。

## 8. 全局重定位

参考 FAST-LIO-LOCALIZATION：
- `global_localization.py`：用 Open3D 对「当前累积扫描」与「先验 PCD 地图」做 ICP，发布 `map → odom` 变换（默认低频运行）。
- `transform_fusion.py`：融合 `map→odom`（重定位，低频）与 `odom→base`（LIO，高频），输出全局一致位姿与 TF。
- `publish_initial_pose.py x y z yaw pitch roll`：给定初始位姿先验，帮助 ICP 收敛。

先验地图与初值在 `global_localization_mid360.launch` 中通过 `3dmap_path`、`publish_initial_pose` 参数指定。

## 9. 调参与故障排查

| 现象 | 排查方向 |
|------|----------|
| 开雷达后位姿发散 | 先查 `radar/extrinsic_R` 与速度符号；`radar_en=false` 应恢复正常 |
| 原地自转却位置漂 | 杆臂 `extrinsic_T` 标定误差；调大 `min_trans_vel` |
| 雷达几乎不起作用 | `chi2_thr` 太小被频繁拒绝；或 `time_tol` 太小匹配不到帧 |
| 速度输出抖动 | `vel_cov` 太小（过度自信），适当增大 |
| `chi2` 频繁触顶 | 外参/时间同步/速度符号问题，需重新标定 |
| 重定位不收敛 | 检查先验地图、`publish_initial_pose` 初值、Open3D 是否安装 |

**调参建议**：`min_trans_vel` 取「雷达在静止/纯自转时残留平移速度噪声」的 2~3 倍。

## 10. 目录结构

```
fast_lio_global/
├── config/                 # 各雷达型号配置（含 radar 段）
│   └── mid360.yaml
├── launch/
│   ├── mapping_mid360.launch          # 建图模式
│   ├── global_localization_mid360.launch  # 重定位模式
│   └── Pointcloud2Map.launch          # 2D 栅格建图
├── scripts/                # 全局重定位 Python 节点
│   ├── global_localization.py
│   ├── transform_fusion.py
│   └── publish_initial_pose.py
├── src/
│   ├── laserMapping.cpp    # 主节点（LIO + 雷达融合）
│   ├── preprocess.{h,cpp}  # 点云预处理
│   └── IMU_Processing.hpp  # IMU 前向传播/去畸变
├── include/
│   ├── ikd-Tree/           # 增量 KD-Tree
│   ├── IKFoM_toolkit/      # 流形卡尔曼滤波
│   └── use-ikfom.hpp       # 23 维状态定义
├── msg/Pose6D.msg
├── PCD/                    # 点云地图保存目录
└── README.md
```

## 11. 致谢与许可

- 基于香港大学 MaRS 实验室的 **FAST-LIO / FAST-LIO2**、**ikd-Tree**、**IKFoM**。
- 全局重定位参考 **FAST_LIO_LOCALIZATION**。
- 许可证：**BSD**（见 [`LICENSE`](LICENSE)）。

> 本工程的新增内容（雷达速度抗退化融合等）以注释块 `// ====== [雷达速度融合] ======` 标注，便于检索与裁剪。
