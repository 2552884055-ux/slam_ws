# pointcloud_to_laserscan —— 3D 点云转 2D 激光扫描

将 `sensor_msgs/PointCloud2`（3D 点云）按高度切片投影为 `sensor_msgs/LaserScan`（2D 扫描线），使只有 3D 雷达的平台也能对接传统 2D 激光 SLAM / 导航 / 避障算法。

> 来源：ROS 社区开源包 [ros-perception/pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)（版本 1.3.1）。本文档面向其在本工作空间中的用法。

---

## 1. 在本工程中的角色

FAST-LIO 输出的世界系点云 `/cloud_registered` 经本包投影为 `/<laser>/scan`，供 `map_switch/tf_publish`、2D 栅格建图、`pcd2pgm` 等下游使用。

```
/cloud_registered (PointCloud2) → pointcloud_to_laserscan → /mid360/scan (LaserScan)
```

---

## 2. 节点 / nodelet

| 形式 | 名称 | 说明 |
|------|------|------|
| 可执行 | `pointcloud_to_laserscan_node` | 独立节点，内部加载 nodelet |
| nodelet | `pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet` | 实际转换逻辑，可嵌入 nodelet manager |

转换逻辑：按高度/距离过滤点 → 按角度分桶 → 每条射线取最近距离；支持可选 TF 变换到 `target_frame`；按订阅数惰性收发。

---

## 3. 话题

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `cloud_in` | `sensor_msgs/PointCloud2` |
| 发布 | `scan` | `sensor_msgs/LaserScan` |

---

## 4. 主要参数（nodelet 默认值）

| 参数 | 默认 | 说明 |
|------|------|------|
| `target_frame` | `""` | 投影目标坐标系（空=用点云自身 frame，不做 TF） |
| `transform_tolerance` | `0.01` | TF 容差(s) |
| `min_height` / `max_height` | `0.0` / `1.0` | 参与投影的高度切片范围(m) |
| `angle_min` / `angle_max` | `-π/2` / `π/2` | 扫描角范围(rad) |
| `angle_increment` | `π/360` | 角分辨率(rad) |
| `scan_time` | `1/30` | 扫描周期(s) |
| `range_min` / `range_max` | `0.45` / `4.0` | 有效距离范围(m) |
| `use_inf` | `true` | 空射线填 `inf`（否则填 `range_max+1`） |
| `concurrency_level` | `1` | 处理线程数（`0`=与 CPU 核数一致） |

---

## 5. Launch

| 文件 | 说明 |
|------|------|
| `launch/sample_node.launch` | 本工程常用：`cloud_in→/cloud_registered`，`scan→$(arg laser)/scan`（默认 `mid360`），高度 `±0.5`，角度 `±π`，距离 `0.1~10.0` |
| `launch/sample_nodelet.launch` | nodelet 形式示例（含 openni2 深度相机） |

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash
roslaunch pointcloud_to_laserscan sample_node.launch laser:=mid360
```

---

## 6. 依赖

catkin：`roscpp`、`nodelet`、`sensor_msgs`、`message_filters`、`tf2`、`tf2_ros`、`tf2_sensor_msgs`。
