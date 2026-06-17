# map_switch —— 多楼层地图切换 + 位姿/激光 UDP 上报

巡检机器人跨楼层 / 多区域时的 **地图切换控制** 与 **机器人状态对外上报** 模块。包含两个独立可执行：

| 可执行 | ROS 节点名 | 作用 |
|--------|-----------|------|
| `map_switch` | `tcp_map_switch_node` | TCP 服务，接收切换指令，停旧图/起新图/重定位/回执 |
| `tf_publish` | `tf_publish` | 读取 TF 与 LaserScan，经 UDP 对外发布机器人位姿与两侧障碍距离 |

---

## 1. map_switch 节点（楼层切换）

### 流程
```
外部 TCP 请求 {目标地图id, seq, x, y, yaw}  (监听 server_addr:map_switch_PORT, 默认 :6050)
        │
        ├─ 执行 stop_script 停止当前地图进程
        ├─ 执行目标地图的 start_script 启动新地图
        ├─ 等待 /waiting_for_initial_pose (std_msgs/Bool)：就绪后调用 initial_pose_script 发布初始位姿
        ├─ 等待 /map_to_odom_flag (std_msgs/Bool)：重定位完成标志
        └─ 通过 TCP 回发成功/失败结果
```

> ⚠️ 现状：`handleTcpData()` 中调用 `launchMap(..., 0, 0, 0, client_fd)`，**忽略请求里的 x/y/yaw**，统一用 `(0,0,0)` 启动（注释说明是肇庆测试为规避电梯漂移所做的硬编码）。需要按坐标重定位时应改回使用 `convertBetweenMaps()` 的转换结果。

### 参数
| 参数 | 默认 | 说明 |
|------|------|------|
| `server_addr` | `192.168.2.100` | TCP 绑定 IP（`0.0.0.0`/`0` 表示 `INADDR_ANY`） |
| `map_switch_PORT` | `6050` | TCP 监听端口 |
| `stop_script` | `.../scripts/stop_current_map.sh` | 停止当前地图脚本 |
| `initial_pose_script` | `.../FAST_LIO_GLOBAL/scripts/publish_initial_pose.py` | 发布初始位姿脚本，调用形式 `script x y 0 yaw 0 0` |
| `maps` | **必填** | 楼层列表，每项含 `id` / `start_script` / `tx_to_map1` / `ty_to_map1` / `theta_to_map1`；缺失则致命退出 |

`maps` 由 `config/floors.yaml` 提供，`tx/ty/theta_to_map1` 描述各楼层坐标系到 map1 的变换，用于 `convertBetweenMaps()` 跨图坐标换算。

### 依赖话题
- 等待 `/waiting_for_initial_pose`（`std_msgs/Bool`）
- 等待 `/map_to_odom_flag`（`std_msgs/Bool`，来自 FAST_LIO_GLOBAL 全局重定位）

---

## 2. tf_publish 节点（状态 UDP 上报）

订阅激光、查询 TF，把机器人位姿 + 两侧障碍距离打包，经 UDP 请求-响应对外发布。

| 参数 | 默认 | 说明 |
|------|------|------|
| `sub_LaserScan_topic` | `/mid360/scan` | 输入激光话题（`sensor_msgs/LaserScan`） |
| `pub_topic` | `/robot_position` | 发布话题（`std_msgs/Float32MultiArray`，当前实现未实际 publish） |
| `source_frame` / `target_frame` | `/robot` / `/robot_body` | TF 查询的源/目标坐标系 |
| `publish_tf_rate` | `10.0` | 主循环频率（Hz） |
| `server_addr` / `server_PORT` | `192.168.110.206` / `6001` | UDP 服务端绑定 |
| `client_addr` / `client_PORT` | `192.168.110.206` / `6002` | UDP 客户端目标 |
| `socket_en` | `true` | 是否启用 UDP 服务线程 |
| `socket_print_en` | `true` | 是否打印 TF 日志 |

---

## 3. 脚本与配置

```
scripts/
├── start_map1.sh ~ start_map5.sh   # 各楼层启动脚本（由 maps.start_script 调用）
└── stop_current_map.sh             # 停止当前地图进程
config/
├── floors.yaml      # maps 列表：各楼层 id / start_script / 到 map1 的坐标变换
└── mid360.yaml      # tf_publish 用：frame/topic/UDP/频率/开关 等
launch/
├── project.launch                  # 主编排 launch（整体工程入口）
└── publish_map1.launch ~ map5.launch   # 各楼层对应 launch（被 start_mapX.sh 拉起）
```

> `src/backup.cpp` 是早期合并版备份代码，**不参与编译**。

---

## 4. 编译与运行

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash

# 整体编排（推荐）
roslaunch map_switch project.launch

# 或单独运行
rosrun map_switch map_switch     # 楼层切换 TCP 服务
rosrun map_switch tf_publish     # 状态 UDP 上报
```

运行前确认：
- `maps` 参数已从 `config/floors.yaml` 加载；
- `scripts/*.sh` 可执行，且参数里的脚本路径在本机有效（默认 `/home/orangepi/slam_ws/...`）；
- 切换依赖的 `/waiting_for_initial_pose`、`/map_to_odom_flag` 话题存在。

---

## 5. 依赖

- catkin：`roscpp`、`rospy`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf`、`pcl_ros`、`eigen_conversions`
- 系统库：`CURL`、`yaml-cpp`、`OpenCV`（已 find_package，部分当前未强链接）
