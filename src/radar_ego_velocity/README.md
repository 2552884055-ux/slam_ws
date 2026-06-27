# radar_ego_velocity

毫米波雷达(SR71)自速度估计 + FAST-LIO 紧耦合融合,用于对抗**退化场景**(地下管廊、长走廊)中沿轴向的定位漂移。

## 链路

```
SR71 (串口/TTL 或 CAN)
  └─ sr71_serial_node ──/radar_scan──► radar_ego_velocity_node ──/radar_velocity──►  FAST-LIO
     (解析雷达帧)       (RadarScan)    (RANSAC+最小二乘估自速度)   (TwistStamped)    (紧耦合EKF更新)
```
> SR71 用串口(TTL)接入,驱动节点为 `sr71_serial_node`;改用 CAN 接口则用 `sr71_can_node`。
>
> ⚠️ **驱动层协议解析当前是占位 TODO**:`sr71_serial_node` / `sr71_can_node` 的串口/CAN 打开与读循环框架已就绪,但帧解析函数(`parseBuffer`/`parseTarget`/`parseHeader`)尚未按厂家协议实现,默认**不会产出有效 `RadarTarget`**。未补全前 `/radar_scan` 无目标、`/radar_velocity` 无输出。可先用 `test/scripts/` 下的仿真脚本(见末节)验证融合链路。
>
> `radar_ego_velocity_node`(RANSAC 自速度估计)本身已完整实现,只要喂入有效 `/radar_scan` 即可工作。

- 雷达**不直接**进 FAST-LIO,先估出**雷达系自速度**(对地速度),再作为速度观测融合;
- 融合点在 `laserMapping.cpp` 激光 IESKF 更新**之后**:`fuse_radar_velocity()`;
- 紧耦合:速度观测进同一状态/协方差,连带修正姿态与陀螺零偏。

## 编译

```bash
cd ~/slam_ws
catkin_make            # 或 catkin build
source devel/setup.bash
```
> `sr71_serial_node`(POSIX termios 串口)与 `sr71_can_node`(SocketCAN)**仅在 Linux 下编译**;非 Linux 开发机只能编译/查看 `radar_ego_velocity_node`,实机请在 Linux 上编译运行。

## 运行

```bash
# 1) 确认串口设备与权限 (实机, Linux)
ls /dev/ttyUSB*                       # 找到 SR71 的串口号
sudo usermod -aG dialout $USER        # 串口权限(需重新登录生效)

# 2) 一键启动 FAST-LIO + 雷达链路
roslaunch radar_ego_velocity mapping_mid360_with_radar.launch
```

## 必须完成的两处标定/适配

1. **串口/CAN 协议解析（当前为占位 TODO，必须先补全）** —— 打开 `src/sr71_serial_node.cpp`(或 `src/sr71_can_node.cpp`),按纳雷《SR71 通信协议文档》补全:
   - 帧格式常量 `FRAME_HEAD0/1`、`TARGET_BYTES`、帧长计算、校验;
   - `parseTarget()`/`parseHeader()` 里方位角/距离/多普勒的字节偏移与缩放系数(现在 `parseTarget()` 直接 `return false`);
   - launch 里 `port`(如 `/dev/ttyUSB0`)和 `baudrate`。
   这是唯一无法预先写死、且**未实现前整条实机链路不通**的部分。

2. **雷达外参 + 多普勒符号**
   - `config/mid360.yaml` 的 `radar/extrinsic_T`、`extrinsic_R`:雷达相对 IMU 的位置和姿态
     (机头 45° 朝下),**必须按实际安装标定**;
   - `radar_ego_velocity.launch` 的 `doppler_sign`:载体朝雷达视线方向前进时,
     输出速度应指向正前方;若方向反了,把 `1.0` 改成 `-1.0`。

## 关键参数(融合侧,位于 `fast_lio_global/config/mid360.yaml` → radar:)

> 下列融合参数属于 **FAST_LIO_GLOBAL** 包(消费 `/radar_velocity` 的一侧),不在本包内;本包 launch 只配 RANSAC 估计相关参数(`doppler_sign`、`ransac_iter`、`inlier_thresh`、`min/max_range`、`max_speed` 等)。


| 参数 | 含义 |
|------|------|
| `radar_en` | 融合总开关 |
| `vel_cov` | 水平速度观测噪声,越小越信任雷达 |
| `vel_cov_z` | 垂直速度噪声;2D雷达设 1e6(忽略z) |
| `chi2_thr` | 卡方门限,挡粗差/动目标 |
| `extrinsic_T/R` | 雷达→IMU 外参(含45°安装) |

## 接 EGO-Planner

FAST-LIO 输出已可作为 EGO 输入,在 EGO 的 launch 里重映射:
```xml
<remap from="<EGO的odom话题>"  to="/Odometry"/>
<remap from="<EGO的cloud话题>" to="/cloud_registered"/>
```
注意坐标系(`camera_init`/`body`)与 EGO 期望的 frame_id 对齐;`/Odometry` 的 twist 已填世界系速度。

## 仿真验证（无实机 / 驱动未补全时）

在 SR71 协议解析补全前,可用 `test/scripts/` 下的脚本离线验证融合链路(脚本分类见 `test/` 子目录):

- `radar_vel_synth.py` —— 合成 `/radar_velocity`,绕过真实雷达驱动直接喂 FAST-LIO;
- `lidar_degrade.py` —— 人为制造激光退化,复现轴向漂移;
- `sim_corridor.py` —— 模拟管廊场景。

## 验证退化效果

- 关融合(`radar_en: false`)在管廊里跑一遍,看 `/Odometry` 沿轴向漂移;
- 开融合再跑,轴向应明显收敛;`rostopic echo /radar_velocity` 确认速度合理、不频繁丢帧。
