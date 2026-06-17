# octomap_mapping —— 基于 OctoMap 的三维占据栅格建图

把点云增量融合为 **八叉树（octree）三维占据栅格地图**，并对外提供地图话题 / 服务，同时可投影出 2D 占据栅格 `projected_map`。常用于三维环境建图、路径规划与避障。

> 来源：开源 ROS 栈 [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping)（ROS1 分支 `kinetic-devel`，ROS2 分支 `ros2`）。本文档面向其在本工作空间中的用法。

---

## 1. 子包构成

| 子包 | 说明 |
|------|------|
| `octomap_mapping` | 元包（metapackage），仅依赖 `octomap_server` |
| `octomap_server` | 实际建图节点 / nodelet / 工具集 |

`octomap_server` 提供的可执行：`octomap_server_node`、`octomap_color_server_node`、`octomap_server_static`、`octomap_server_multilayer`、`octomap_tracking_server_node`、`octomap_saver`；以及 `octomap_server_nodelet` / `octomap_color_server_nodelet`。

---

## 2. 在本工程中的角色

订阅 FAST-LIO 输出的配准点云（`cloud_in` ← 如 `/cloud_registered`），构建 3D OctoMap，并发布可视化 Marker、`octomap_full/binary`、点云中心、以及 2D 投影地图 `projected_map`。

```
/cloud_registered (PointCloud2) → octomap_server_node → /projected_map (OccupancyGrid)
                                                       → /octomap_full, /octomap_point_cloud_centers
                                                       → occupied/free_cells_vis_array (Marker)
```

---

## 3. 话题与服务

### 发布
| 话题 | 类型 |
|------|------|
| `occupied_cells_vis_array` / `free_cells_vis_array` | `visualization_msgs/MarkerArray` |
| `octomap_binary` / `octomap_full` | `octomap_msgs/Octomap`（同名亦提供服务） |
| `octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` |
| `projected_map` | `nav_msgs/OccupancyGrid` |

### 订阅
| 话题 | 类型 |
|------|------|
| `cloud_in` | `sensor_msgs/PointCloud2`（输入点云，launch 中 remap） |

### 服务
`octomap_binary`、`octomap_full`、`~clear_bbx`、`~reset`。

---

## 4. 主要参数

| 参数 | 默认 / 示例 | 说明 |
|------|------------|------|
| `resolution` | `0.05` | octree 体素分辨率(m) |
| `frame_id` | `odom_combined` / `map` | 全局坐标系 |
| `base_frame_id` | — | 机体坐标系 |
| `sensor_model/max_range` | `5.0` | 射线插入最大距离(m) |
| `sensor_model/hit` / `miss` | `0.7` / `0.4` | 命中/穿透概率 |
| `sensor_model/min` / `max` | `0.12` / `0.97` | 占据概率夹紧范围 |
| `pointcloud_min/max_{x,y,z}` | — | 输入点云裁剪范围 |
| `occupancy_min/max_z` | — | 2D 投影的高度范围 |
| `filter_ground` / `ground_filter/*` | — | 地面分割（距离/角度/平面距离） |
| `filter_speckles` | — | 去孤立体素 |
| `height_map` / `colored_map` | — | 高度着色 / 彩色地图 |
| `latch` | — | 是否锁存发布 |

跟踪模式额外参数（`octomap_tracking_server_node`）：`topic_changes`、`track_changes`、`listen_changes`、`change_id_frame`、`min_change_pub`。

---

## 5. Launch

| 文件 | 说明 |
|------|------|
| `octomap_server/launch/octomap_mapping.launch` | 普通建图（`resolution=0.05`, `max_range=5.0`） |
| `octomap_server/launch/octomap_mapping_nodelet.launch` | nodelet 形式 |
| `octomap_server/launch/octomap_tracking_server.launch` | 跟踪服务端（发布地图变更集） |
| `octomap_server/launch/octomap_tracking_client.launch` | 跟踪客户端（接收并应用变更集） |

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash
roslaunch octomap_server octomap_mapping.launch     # 注意将 cloud_in remap 到实际点云话题

# 保存地图
rosrun octomap_server octomap_saver -f map.bt
```

---

## 6. 依赖

catkin：`roscpp`、`sensor_msgs`、`std_msgs`、`std_srvs`、`nav_msgs`、`visualization_msgs`、`pcl_ros`、`pcl_conversions`、`octomap_ros`、`octomap_msgs`、`dynamic_reconfigure`、`nodelet`；系统库 `octomap`。
