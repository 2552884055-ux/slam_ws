# radar_ego_velocity

毫米波雷达(SR71)自速度估计 + FAST-LIO 紧耦合融合,用于对抗**退化场景**(地下管廊、长走廊)中沿轴向的定位漂移。

## 链路

```
SR71 (串口/TTL)
  └─ sr71_serial_node ──/radar_scan──► radar_ego_velocity_node ──/radar_velocity──►  FAST-LIO
     (解析串口帧)       (RadarScan)    (RANSAC+最小二乘估自速度)   (TwistStamped)    (紧耦合EKF更新)
```
> SR71 用串口(TTL)接入,驱动节点为 `sr71_serial_node`。若改用 CAN 接口,另有 `sr71_can_node` 备用。

- 雷达**不直接**进 FAST-LIO,先估出**雷达系自速度**(对地速度),再作为速度观测融合;
- 融合点在 `laserMapping.cpp` 激光 IESKF 更新**之后**:`fuse_radar_velocity()`;
- 紧耦合:速度观测进同一状态/协方差,连带修正姿态与陀螺零偏。

## 编译

```bash
cd ~/Desktop/slam_ws/slam_ws
catkin_make            # 或 catkin build
source devel/setup.bash
```
> Windows 开发机上 `sr71_can_node`(SocketCAN)不会编译,只编译估计节点;实机请在 Linux 上编译运行。

## 运行

```bash
# 1) 确认串口设备与权限 (实机, Linux)
ls /dev/ttyUSB*                       # 找到 SR71 的串口号
sudo usermod -aG dialout $USER        # 串口权限(需重新登录生效)

# 2) 一键启动 FAST-LIO + 雷达链路
roslaunch radar_ego_velocity mapping_mid360_with_radar.launch
```

## 必须完成的两处标定/适配

1. **串口协议解析** —— 打开 `src/sr71_serial_node.cpp`,按纳雷《SR71 串口通信协议文档》补全:
   - 帧格式常量 `FRAME_HEAD0/1`、`TARGET_BYTES`、帧长计算、校验;
   - `parseTarget()` 里方位角/距离/多普勒的字节偏移与缩放系数;
   - launch 里 `port`(如 `/dev/ttyUSB0`)和 `baudrate`。
   这是唯一无法预先写死的部分。

2. **雷达外参 + 多普勒符号**
   - `config/mid360.yaml` 的 `radar/extrinsic_T`、`extrinsic_R`:雷达相对 IMU 的位置和姿态
     (机头 45° 朝下),**必须按实际安装标定**;
   - `radar_ego_velocity.launch` 的 `doppler_sign`:载体朝雷达视线方向前进时,
     输出速度应指向正前方;若方向反了,把 `1.0` 改成 `-1.0`。

## 关键参数(config/mid360.yaml → radar:)

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

## 验证退化效果

- 关融合(`radar_en: false`)在管廊里跑一遍,看 `/Odometry` 沿轴向漂移;
- 开融合再跑,轴向应明显收敛;`rostopic echo /radar_velocity` 确认速度合理、不频繁丢帧。
