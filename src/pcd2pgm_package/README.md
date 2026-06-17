# pcd2pgm_package —— PCD 点云地图转 2D 栅格地图

将 FAST-LIO 建图保存的 **`.pcd` 点云地图** 离线投影为 **2D 占据栅格地图**（`nav_msgs/OccupancyGrid`），再配合 `map_server map_saver` 导出为 `.pgm` + `.yaml`，供 2D 导航使用。

> 子包：`pcd2pgm`（可执行 `pcd2pgm`，ROS 节点名 `pcl_filters`）。

---

## 1. 处理流程

```
读取 file_directory/file_name.pcd
   │  ① 直通滤波(z 方向)：保留 [thre_z_min, thre_z_max] 内的点  → map_filter.pcd
   │  ② 半径滤波：去离群点(thre_radius 内邻居数 < thres_point_count) → map_radius_filter.pcd
   │  ③ 投影到 XY 平面，按 map_resolution 栅格化，占据格置 100
   ▼
以 1 Hz 持续发布 map_topic_name (nav_msgs/OccupancyGrid, frame_id=map)
```

> 节点**不订阅任何话题**，直接从磁盘读取 PCD；中间结果 `map_filter.pcd` / `map_radius_filter.pcd` 会写回 `file_directory`。

---

## 2. 参数

| 参数 | 默认（代码 / launch 示例） | 说明 |
|------|--------------------------|------|
| `file_directory` | `/home/` / `/home/orangepi/rosbag/pcd/map_3_31/` | PCD 所在目录（含末尾 `/`） |
| `file_name` | `map` / `map_3_31` | PCD 文件名（不含 `.pcd`） |
| `thre_z_min` | `0.2` / `0.1` | 直通滤波 z 下限(m) |
| `thre_z_max` | `2.0` / `1.5` | 直通滤波 z 上限(m) |
| `flag_pass_through` | `0` | `0`=保留范围内点，`1`=保留范围外点 |
| `thre_radius` | `0.5` | 半径滤波搜索半径(m) |
| `thres_point_count` | `10` | 半径内最少邻居数（少于则视为离群） |
| `map_resolution` | `0.05` | 栅格分辨率(m/格) |
| `map_topic_name` | `map` / `map_2d` | 发布的栅格地图话题名 |

---

## 3. 编译与运行

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash

# 修改 launch 里的 file_directory / file_name 指向你的 PCD 后启动
roslaunch pcd2pgm run.launch

# 另开终端，将栅格地图导出为 pgm+yaml
rosrun map_server map_saver map:=/map_2d -f ~/map_2d
```
> 导出的 `.yaml` 中若出现 `nan`，需手动改为 `0`，否则 `map_server` 加载报错。

---

## 4. 依赖

catkin：`roscpp`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`pcl_ros`；系统库 `PCL`。

---

## 5. 目录结构

```
pcd2pgm_package/
└── pcd2pgm/
    ├── src/pcd2pgm.cpp
    ├── launch/run.launch
    ├── CMakeLists.txt
    └── package.xml
```
