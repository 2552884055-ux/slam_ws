# slam_ws —— 毫米波雷达辅助抗退化激光惯性 SLAM 与巡检机器人工作空间

面向 **管廊 / 隧道 / 长走廊 / 多楼层** 等激光退化场景的无人机 / 巡检机器人定位与建图工作空间（catkin / ROS Noetic）。

核心思路：在 FAST-LIO2 激光惯性里程计的基础上，引入 **毫米波雷达多普勒自速度** 作为速度观测，紧耦合补偿激光沿轴向的退化漂移；并在此之上提供 **先验地图全局重定位、多楼层地图切换、跨楼层乘梯（Modbus 电梯控制 + 两阶段地图切换）、2D 栅格建图、位置 Socket 上报、RTSP 远程监看、开机/按键/端口自启动** 等一整套工程化能力。

> 本仓库是一个完整的产品级工作空间，不仅含算法核心（FAST_LIO_GLOBAL），还包含驱动、地图转换、楼层切换、对外通信、自启动等部署所需的全部配套包。

---

## 目录

- [1. 工作空间组成](#1-工作空间组成)
- [2. 整体数据流](#2-整体数据流)
- [3. 依赖与环境](#3-依赖与环境)
- [4. 编译](#4-编译)
- [5. 运行场景](#5-运行场景)
- [6. 自启动机制](#6-自启动机制)
- [7. 根目录文件说明](#7-根目录文件说明)
- [8. 部署前必须完成的标定 / 适配](#8-部署前必须完成的标定--适配)
- [9. 目录结构](#9-目录结构)
- [10. 致谢与许可](#10-致谢与许可)

---

## 1. 工作空间组成

`src/` 下共 **10 个 ROS 包**，按职责可分为「核心算法」「传感器驱动」「地图工具」「对外通信 / 部署」四类：

### 核心算法
| 包 | 作用 | 文档 |
|----|------|------|
| **[FAST_LIO_GLOBAL](src/FAST_LIO_GLOBAL/)** | FAST-LIO2 激光惯性里程计 + 雷达速度抗退化融合 + 全局重定位 + 2D 建图 | [README](src/FAST_LIO_GLOBAL/README.md) |
| **[radar_ego_velocity](src/radar_ego_velocity/)** | 毫米波雷达(SR71)驱动 + RANSAC 自速度估计，解算雷达系自速度 `/radar_velocity` | [README](src/radar_ego_velocity/README.md) |

### 传感器驱动
| 包 | 作用 | 文档 |
|----|------|------|
| **[livox_ros_driver2](src/livox_ros_driver2/)** | Livox MID360 等激光雷达 ROS 驱动，发布 `/livox/lidar` 与 `/livox/imu` | [README](src/livox_ros_driver2/README.md) |

### 地图工具
| 包 | 作用 | 文档 |
|----|------|------|
| **[pointcloud_to_laserscan](src/pointcloud_to_laserscan/)** | 3D 点云投影为 2D `LaserScan`，供 2D 导航 / 建图使用 | [README](src/pointcloud_to_laserscan/README.md) |
| **[octomap_mapping](src/octomap_mapping/)** | 基于 OctoMap 的三维八叉树占据栅格建图与服务 | [README](src/octomap_mapping/README.md) |
| **[pcd2pgm_package](src/pcd2pgm_package/)** | 离线把 `.pcd` 点云地图投影为 2D 栅格地图（`.pgm`） | [README](src/pcd2pgm_package/README.md) |

### 对外通信 / 部署
| 包 | 作用 | 文档 |
|----|------|------|
| **[all_project](src/all_project/)** | 多楼层 / 多区域地图切换（`map_switch`，TCP 控制）+ 位姿/激光 UDP 上报（`tf_publish`），编出 `map_switch`、`tf_publish` 两个可执行 | [README](src/all_project/README.md) |
| **[goalsender](src/goalsender/)** | 缓存里程计，通过 UDP 请求-响应协议把机器人位姿对外发布 | [README](src/goalsender/README.md) |
| **[rtsp_stream](src/rtsp_stream/)** | 将建图画面 / ROS 图像转 RTSP / WebRTC 流，供远程监看 | [README](src/rtsp_stream/README.md) |

### 乘梯 / 电梯控制（上位机程序，独立 CMake，非 ROS 包）

跨楼层巡检的乘梯控制端：通过 Modbus 与电梯通信完成召梯/进梯/乘梯/出梯，并与机器人侧 `all_project/map_switch` 节点配合实现**两阶段地图切换**（乘梯期间后台 LOAD、出梯后 RELOC）。运行在上位机/控制端，不属于 catkin 工作空间。

| 程序 | 作用 | 文档 |
|----|------|------|
| **[ElevatorControl](src/ElevatorControl/)** | Modbus 电梯控制 + 两阶段地图切换；**TCP(网络) / RTU(串口) 二合一**(同一套代码,构造时选传输方式) | [README](src/ElevatorControl/README.md) |
| **[ElevatorSimulator](src/ElevatorSimulator/)** | 无真实电梯/`map_switch` 时的模拟器：`elevator_sim`(模拟电梯,Modbus 从站,TCP/RTU) + `map_switch_sim`(模拟地图切换服务端)，纯本机联调 | [README](src/ElevatorSimulator/README.md) |

---

## 2. 整体数据流

```
  Livox MID360 ──┐  /livox/lidar (点云)
   (LiDAR+IMU)   ├─ livox_ros_driver2 ──┐  /livox/imu (IMU)
                 ┘                       │
  SR71 毫米波雷达 ──串口/CAN──┐           │
   (TTL / SocketCAN)         ▼           ▼
                    ┌──────────────────────────────────┐
                    │ radar_ego_velocity                │
                    │  sr71_serial/can → /radar_scan    │
                    │  → RANSAC 自速度 → /radar_velocity │
                    └─────────────────┬─────────────────┘
                                      │ (TwistStamped 雷达系自速度)
                                      ▼
        ┌──────────────────────────────────────────────────────┐
        │ FAST_LIO_GLOBAL (laserMapping)                         │
        │  IESKF 激光主更新  +  雷达速度紧耦合更新(可选)          │
        │  +  Open3D ICP 全局重定位(可选)                        │
        └──┬──────────────────────────────┬────────────────────┘
           │ /Odometry, /cloud_registered │ /projected_map (2D 栅格)
           │ /path, TF                    │
           ▼                              ▼
  pointcloud_to_laserscan / octomap_mapping / pcd2pgm_package  (地图派生)
           │
           ▼
  map_switch (楼层切换 + UDP 上报) · goalsender (UDP 位姿) · rtsp_stream (远程画面)
```

要点：
- 雷达 **不直接** 进 FAST-LIO，而是先估出 **雷达系自速度（对地速度）**，再作为速度观测融合；
- 融合发生在激光 IESKF 更新 **之后**（`laserMapping.cpp` 的 `fuse_radar_velocity()`），属于状态级紧耦合，连带修正速度、姿态与陀螺零偏；
- 退化方向（沿廊轴前进）由雷达速度约束，从而抑制激光里程计的轴向漂移；
- 下游派生模块（2D 栅格、OctoMap、楼层切换、UDP/RTSP 上报）均消费 LIO 的输出，相互独立、按需启用。

---

## 3. 依赖与环境

### 系统
- **rk3588 平台**（推荐；不建议香橙派 5，资源有限）+ **Ubuntu 20.04** + **ROS Noetic**
- C++14、CMake ≥ 3.0、OpenMP

### C++ / 库
- **PCL ≥ 1.8**、**Eigen ≥ 3.3.4**
- **Livox-SDK2** + **livox_ros_driver2**（须先编译并 source）
- `yaml-cpp`、`OpenCV`、`CURL`（map_switch / rtsp_stream 用）
- `octomap`（octomap_mapping 用）
- `libmodbus`（电梯控制 ElevatorControl / ElevatorSimulator 用：`sudo apt install libmodbus-dev`）

### Python（按需）
```bash
pip3 install open3d ros_numpy      # FAST_LIO_GLOBAL 全局重定位
pip3 install opencv-python numpy mss   # rtsp_stream 抓屏/推流
sudo apt install ffmpeg            # rtsp_stream / tuiliu.py 推流后端
# 另需 mediamtx 二进制（rtsp_stream WebRTC/HLS 网关）
```

### 外部工具
- **Livox Viewer2**：查看 / 修改雷达 IP 与编号
- **GIMP**：修正生成的 `.pgm` 栅格地图
  ```bash
  sudo apt install -y gimp
  ```

> 注：`radar_ego_velocity` 的串口 / CAN 节点（`sr71_serial_node` / `sr71_can_node`）只能在 Linux 上编译运行；其他平台仅用于查看 / 编辑源码。

详细的从零环境搭建（一键装 ROS、装 Livox-SDK、常见报错处理）见仓库根目录的 [`README`](README)（刘海洋整理的部署手册）。

---

## 4. 编译

```bash
cd ~/slam_ws            # 实际部署目录通常为 /home/orangepi/slam_ws
catkin_make             # 或 catkin build
source devel/setup.bash
```
> 香橙派 / rk3588 上若内存吃紧，降低并行线程数：`catkin_make -j4`。
>
> 关键可执行：`livox_ros_driver2`、`radar_ego_velocity_node` / `sr71_serial_node`、`fastlio_mapping_global`、`map_switch` / `tf_publish`、`goal_sender_node`、`pcd2pgm`、`pointcloud_to_laserscan_node`、`octomap_server_node`。

---

## 5. 运行场景

### 5.1 建图（含雷达融合，推荐）
```bash
roslaunch radar_ego_velocity mapping_mid360_with_radar.launch
```
同时拉起 FAST-LIO 建图（`mapping_mid360.launch`）+ 雷达自速度估计链路。FAST-LIO 默认订阅 `/radar_velocity`，无需重映射。

### 5.2 分步启动
```bash
# 终端1：激光雷达驱动
roslaunch livox_ros_driver2 msg_MID360.launch
# 终端2：雷达自速度估计（需先接好 SR71）
roslaunch radar_ego_velocity radar_ego_velocity.launch
# 终端3：FAST-LIO（含融合）
roslaunch fast_lio_global mapping_mid360.launch
```

### 5.3 保存 2D 栅格地图
```bash
rosrun map_server map_saver map:=/projected_map -f ~/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans
```
> 生成的 `.yaml` 中地图参数若出现 `nan`，需手动改为 `0`，否则加载报错。

### 5.4 全局重定位（已有先验 PCD 地图）
```bash
roslaunch fast_lio_global global_localization_mid360.launch
rosrun fast_lio_global publish_initial_pose.py 0 -0.5 0 0 0 0   # 初始位姿先验
```

### 5.5 整体工程（含楼层切换）
```bash
roslaunch all_project project.launch
```
> 包名为 `all_project`（内含 `map_switch`、`tf_publish` 两个可执行），切换服务节点即 `map_switch`。

### 5.6 仅纯激光（关闭雷达，做 A/B 对比）
编辑 `src/FAST_LIO_GLOBAL/config/mid360.yaml`，将 `radar/radar_en` 设为 `false`，系统即等价于原版 FAST-LIO2。

---

## 6. 自启动机制

部署到机器人上后，整套系统通过 `auto_start.sh` 起栈，并提供三种触发方式（按项目选其一）：

| 方式 | 触发器脚本 | 说明 | 典型项目 |
|------|-----------|------|----------|
| **开机自启** | — | 开机直接执行 `auto_start.sh` | 通用 |
| **GPIO 按键触发** | [`gpio_trigger.py`](gpio_trigger.py) | 监听 orangepi5plus GPIO1_A4 引脚，低→高跳变触发一次 | 肇庆巡检机器狗 |
| **网络端口触发** | [`port_trigger.py`](port_trigger.py) | 监听 TCP `8080`，收到 `start`/`stop`/`status` 指令控制 ROS 栈 | 码垛机器人 |

`auto_start.sh` 会依次在新终端启动 Livox 驱动与整体工程（`roslaunch all_project project.launch`）。

> 部署目录默认为 `/home/orangepi/slam_ws`，如实际路径不同需改 `auto_start.sh` 里的 `cd` 行。

---

## 7. 根目录文件说明

| 文件 | 说明 |
|------|------|
| [`README`](README) | 部署手册（环境搭建、装机、常见报错、建图与运行步骤），刘海洋整理 |
| [`auto_start.sh`](auto_start.sh) | 一键启动脚本：拉起 Livox 驱动 + 整体工程 |
| [`gpio_trigger.py`](gpio_trigger.py) | GPIO 按键触发 `auto_start.sh`（带防抖、日志、只触发一次） |
| [`port_trigger.py`](port_trigger.py) | TCP 端口监听，支持 `start` / `stop`（安全杀节点）/ `status`（列节点） |
| [`tuiliu.py`](tuiliu.py) | 把 `/projected_map` 2D 栅格地图渲染为图像并经 FFmpeg 推 RTSP（`rtsp://127.0.0.1:8554/mapping`），用于远程看建图进度 |
| `.catkin_workspace` | catkin 标记文件 |
| `.gitignore` | 忽略 `build/ devel/ install/ Log/ PCD/ *.pcd *.pgm *.bag` 等产物 |

---

## 8. 部署前必须完成的标定 / 适配

整条链路能否在退化环境正常工作，关键在代码之外的这几项（详见各包 README）：

1. **SR71 串口/CAN 协议解析**：按雷达厂商协议补全 `radar_ego_velocity` 的 `sr71_serial_node.cpp` / `sr71_can_node.cpp` 帧格式、字段偏移与缩放（当前为占位 TODO，未实现前 `/radar_scan` 无目标）。
2. **雷达 → IMU 外参**：`config/mid360.yaml` 的 `radar/extrinsic_T`（杆臂）与 `extrinsic_R`（含机头安装角），**必须实测标定**——平移误差在自转/悬停时尤其敏感。
3. **多普勒符号约定**：`radar_ego_velocity.launch` 的 `doppler_sign`，前进时速度应指向正前方，反了则改符号。
4. **激光 → IMU 外参 / 时间同步**：`mapping/extrinsic_*` 与时间偏移。
5. **Livox 雷达 IP / 编号**：雷达默认 `192.168.1.1XX`（XX 为编号后两位），本机网口须与雷达同网段（默认 `192.168.1.50`），在 Livox Viewer2 中查看修改。
6. **楼层切换坐标 / 脚本路径**：`all_project/config/floors.yaml` 的各楼层电梯口锚点 `exit_x/exit_y/exit_yaw` 与 `tx/ty/theta_to_map1`，以及各触发脚本里的绝对路径（默认 `/home/orangepi/slam_ws/...`）。

> FAST-LIO 侧已内置「自转/悬停自适应降权」机制（`radar/min_trans_vel`），显著降低低速工况对杆臂标定误差的敏感性；但外参与符号仍需正确标定。

---

## 9. 目录结构

```
slam_ws/
├── README                  # 部署手册（环境/装机/建图步骤）
├── README.md               # 本文件（工作空间总览）
├── auto_start.sh           # 一键启动脚本
├── gpio_trigger.py         # GPIO 按键触发自启
├── port_trigger.py         # 端口监听触发自启
├── tuiliu.py               # 2D 地图 → RTSP 推流
└── src/
    ├── FAST_LIO_GLOBAL/        # 核心：FAST-LIO2 + 雷达融合 + 全局重定位 + 2D 建图
    ├── radar_ego_velocity/     # SR71 雷达驱动 + RANSAC 自速度估计
    ├── livox_ros_driver2/      # Livox 激光雷达驱动
    ├── pointcloud_to_laserscan/# 3D 点云 → 2D LaserScan
    ├── octomap_mapping/        # OctoMap 三维占据栅格建图
    ├── pcd2pgm_package/        # PCD → 2D 栅格地图离线转换
    ├── all_project/            # 多楼层地图切换(map_switch) + 位姿/激光 UDP 上报(tf_publish)
    ├── goalsender/             # UDP 请求-响应 位姿发布
    ├── rtsp_stream/            # RTSP / WebRTC 远程监看
    ├── ElevatorControl/        # 上位机:Modbus 电梯控制 + 两阶段地图切换,TCP/RTU 二合一(非 ROS 包)
    └── ElevatorSimulator/      # 电梯/地图切换模拟器(无真实设备时本机联调,非 ROS 包)
```

---

## 10. 致谢与许可

- **FAST-LIO / FAST-LIO2**、**ikd-Tree**、**IKFoM**（香港大学 MaRS 实验室）
- **livox_ros_driver2**（Livox）
- **octomap_mapping / octomap_server**（OctoMap 团队）
- **pointcloud_to_laserscan**（ros-perception）
- **libmodbus**（电梯 Modbus 通信，LGPL）
- 全局重定位参考 **FAST_LIO_LOCALIZATION**
- 各包许可证见其各自目录（FAST-LIO 为 BSD）。

> 维护：刘海洋 · 2026
