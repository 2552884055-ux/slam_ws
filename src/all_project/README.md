# all_project —— 多楼层地图切换 + 位姿/激光 UDP 上报

巡检机器人跨楼层 / 多区域时的 **地图切换控制** 与 **机器人状态对外上报** 模块。`all_project` 包内含两个独立节点(各自子目录 `map_switch/`、`tf_publish/`),编出两个可执行：

| 可执行 | 源码目录 | ROS 节点名 | 作用 |
|--------|---------|-----------|------|
| `map_switch` | `map_switch/`(protocol/tcp_server/node_launcher/map_switch_node) | `tcp_map_switch_node` | TCP 服务，接收切换指令，停旧图/起新图/重定位/回执 |
| `tf_publish` | `tf_publish/` | `tf_publish` | 读取 TF 与 LaserScan，经 UDP 对外发布机器人位姿与两侧障碍距离 |

> `map_switch` 节点按职责拆分:`protocol.hpp`(协议帧)、`TcpServer`(收发)、`NodeLauncher`(fork/rosrun 起停进程)、`MapSwitchNode`(两阶段编排)。

---

## 1. map_switch 节点（楼层切换，两阶段）

跨层巡检要坐电梯，而**重定位必须在到达新层、雷达扫到新环境后才能做**（物理约束）；电梯内的几何退化与垂直加速度还会污染 LIO 的 odom。为此把一次切换拆成**两个由上位机分别下发的 TCP 指令**：

> 节点不再用 `.sh` / `.launch` 启动：map_switch 在 C++ 里 `fork()` 后 `execvp("rosrun", pkg, type, ...)` 直接拉起每个节点，并以独立进程组运行；停图时按 PID（进程组）`SIGINT→SIGKILL` 精确杀死，不再用 `rosnode kill`。各节点清单见 `buildLoadSpecs()` / `buildLaserSpec()`。

| 阶段 | `cmd` | 触发时机 | 服务端动作 |
|------|------|----------|-----------|
| **LOAD** | `1` | 进电梯 | 停旧节点 → rosrun 起目标层 5 个预加载节点（**不含 laserMapping**）→ 等 `/waiting_for_initial_pose` 就绪后回执。重的地图加载藏在坐电梯时间内，期间不重定位。 |
| **RELOC** | `2` | 出电梯、到达新层并静止 | rosrun 起 laserMapping（干净 IMU 初始化 + odom≈I）→ 等 `/Odometry` → 直接向 `/initialpose` 发初值（带重试）→ 等 `/map_to_odom_flag` → 回执。 |

LOAD 阶段启动的 5 个节点（等价于原 publish_mapN_load.launch）:
| 节点(`__name`) | rosrun 包/类型 | 说明 |
|---|---|---|
| `map_publishe` | `pcl_ros` / `pcd_to_pointcloud` | 发布 `.pcd`(参数 `pcd`) |
| `map_server` | `map_server` / `map_server` | 发布栅格图(参数 `gridmap`),`map:=grid_map` |
| `global_localization` | `fast_lio_global` / `global_localization.py` | 全局重定位(需 `PYTHONPATH`) |
| `transform_fusion` | `fast_lio_global` / `transform_fusion.py` | map→odom→base TF 融合 |
| `tf_robot2map` | `all_project` / `tf_publish` | 位姿/状态 UDP 上报 |

RELOC 阶段:`laserMapping` = `fast_lio_global` / `fastlio_mapping_global`(全楼层共用,参数沿用 project.launch 已加载到参数服务器者)。

> 默认电梯流程下 RELOC 用各层固定电梯口锚点 `exit_x/exit_y/exit_yaw` 作初值(**前提是每层地图原点≈电梯口出梯位姿**,故多数层填 `(0,0,0)`,map1 单独配)。需要按实时坐标重定位时,把 `use_coord_transform` 置 `true`(详见下文)。

### 请求帧 `req_frame`（二进制，定长）
```
cmd(unsigned long) | frame_type(unsigned long, 目标地图id) | seq(unsigned long) | x,y,yaw(float)
```
> ⚠️ `cmd` 为本次改造**新增的首字段**，上位机须按此布局发送，否则服务端长度校验不通过。

### 流程
```
[进电梯] 上位机发 cmd=1 ──> 停旧栈 ──> 起 mapN 预加载节点 ──> 等就绪 ──> 回执
              ⏳ 坐电梯期间地图加载完成（无感）
[出电梯静止] 上位机发 cmd=2 ──> 起 laserMapping ──> 发初值 ──> 重定位 ──> 回执
```
RELOC 初值来源由 `use_coord_transform` 控制：默认 `false` 用各层固定电梯口锚点（`exit_*`），`true` 则用 `convertBetweenMaps` 把请求坐标从源图换算到目标图。

启动时若无激活地图，节点会对 `initial_map_id` 指定的初始地图（缺省为列表第一张）自动做一次完整起栈（LOAD + RELOC），适用于机器人静止在初始楼层的开机场景。

### 切换逻辑详解

**入口 `handleTcpData`**：`accept` → **循环 `recv` 凑满 40 字节定长帧**（抗 TCP 分片）→ **并发互斥**（`busy_.exchange(true)`，已有切换在进行则回失败、拒绝插队）→ 按 `cmd` 分发到 `loadMap` / `relocalize`，各起 detach 线程异步处理，线程结束由 `BusyGuard` 释放 `busy_`，回执后关连接。

**阶段一 `doLoad`（进电梯）**
```
① 校验目标层存在、pcd/gridmap 已配
② 记录 prev_map_id_ = 当前层（供坐标变换 src 用）
③ stopCurrentNodes()：按 PID 杀掉旧层全部节点（含上次 laserMapping），sleep(1s)
④ launchNode() 逐个 rosrun 起 5 个预加载节点；任一失败则回滚已起的
⑤ 记录 5 个 PID，current_map_id_ = 目标层
⑥ 等 /waiting_for_initial_pose（超时 load_ready_timeout）= global_localization 已加载完地图、挂起待命
⑦ 回执 result
```
此阶段**不起 laserMapping、不发初值，绝不重定位**；慢的地图加载藏在坐电梯途中。

**阶段二 `doReloc`（出电梯、新层静止）**
```
① 检查已 LOAD 过（current_map_id_!=0）
② launchNode(laserMapping)：新层干净启动，IMU 初始化好、odom≈I
③ 等 /Odometry（超时 laser_alive_timeout）→ 确认 FAST-LIO 活了，sleep(1s) 等首帧 scan/暂态过去
④ 重试循环（总超时 reloc_total_timeout）：发 /initialpose（初值）→ 等 /map_to_odom_flag 2s
       收到 true → 成功；否则重发（覆盖竞争/首帧未到/ICP fitness 未达标）
⑤ 回执 result
```
重定位由 `global_localization` 一次性 ICP 完成（`fitness>0.95`），成功后发 `/map_to_odom` + `/map_to_odom_flag`，`transform_fusion` 把 `/localization` 变为对齐地图的全局位姿。

**重定位初值的三种来源**
| 场景 | 初值 | 配置项 |
|---|---|---|
| 开机起栈（`init`） | `initial_x/y/yaw`（机器人开机所在位姿） | floors.yaml 顶层 |
| 切层 RELOC，`use_coord_transform=false`（默认） | 目标层固定电梯口锚点 `exit_x/y/yaw` | 每层 `maps` 项 |
| 切层 RELOC，`use_coord_transform=true` | `convertBetweenMaps` 把请求坐标从源图 `prev_map_id_` 换算到目标图 | 各层 `tx/ty/theta_to_map1` |

**节点启停（全 C++，按 PID 精确管理）**
- 启动 `launchNode`：`fork()` → 子进程 `setpgid` 独立进程组 → `execvp("rosrun", pkg type args... __name:=name)`；`PYTHONPATH` 在构造函数进程级设好由子进程继承（不在 fork 后 setenv）。
- 停止 `killNode`：按进程组 `SIGINT`（让 FAST-LIO 正常 shutdown）→ 5s 超时 → `SIGKILL`，`waitpid` 回收。下一次 LOAD 先杀上一层全部节点，保证全程单图。

**顺序巡检一轮（1→2→3→4→5→1）**
```
N层巡检完 → 进电梯发 LOAD(N+1) → 坐电梯（地图后台加载完） →
出电梯静止发 RELOC(N+1) → 起 laser+发该层锚点+重定位 → N+1层巡检 → ...
```
每次切换用户可见耗时 ≈ 出梯后 laserMapping 启动 + 一次 ICP（几秒），而非旧方案一条龙的 ~20s。

### 参数
| 参数 | 默认 | 说明 |
|------|------|------|
| `server_addr` | `192.168.2.100` | TCP 绑定 IP（`0.0.0.0`/`0` 表示 `INADDR_ANY`） |
| `map_switch_PORT` | `6050` | TCP 监听端口 |
| `python_path` | `/home/orangepi/.local/lib/python3.8/site-packages` | 启动 python 节点时注入的 `PYTHONPATH` |
| `use_coord_transform` | `false` | RELOC 初值来源：`false`=各层固定电梯口锚点（`exit_*`）；`true`=用 `convertBetweenMaps` 把请求坐标从源图换算到目标图 |
| `initial_map_id` | `0` | 开机自动起栈的初始地图 ID；`<=0` 表示用 `maps` 列表里的第一张 |
| `initial_x` / `initial_y` / `initial_yaw` | `0` / `0` / `0` | 开机起栈时机器人所在的初始位姿（map 系，yaw 弧度）；用于初始地图的重定位初值，与电梯口锚点 `exit_*` 相互独立 |
| `load_ready_timeout` | `60.0` | LOAD 等待 `/waiting_for_initial_pose` 的超时（秒） |
| `laser_alive_timeout` | `30.0` | RELOC 等待 `/Odometry` 的超时（秒） |
| `reloc_total_timeout` | `30.0` | RELOC 重定位重试总超时（秒） |
| `maps` | **必填** | 楼层列表，每项含 `id` / `pcd` / `gridmap` / `exit_x` / `exit_y` / `exit_yaw` / `tx_to_map1` / `ty_to_map1` / `theta_to_map1` |

`maps`、`use_coord_transform`、`initial_map_id`、`initial_x/y/yaw` 由 `config/floors.yaml` 提供；`pcd` / `gridmap` 为 `fast_lio_global/PCD/` 下的文件名，由 `ros::package::getPath("fast_lio_global")` 拼出绝对路径。节点的可执行文件路径由 `rosrun` 解析，无需配置。

> `use_coord_transform`:
> - `false`（默认，电梯流程推荐）：用各层 `exit_x/exit_y/exit_yaw` 固定锚点重定位，忽略请求里的 `x/y/yaw`，规避电梯漂移。
> - `true`：上位机 RELOC 指令的 `x/y/yaw` 须为机器人在**源图（进电梯前那张图）坐标系**下的位姿，由 `convertBetweenMaps` 用各层 `tx/ty/theta_to_map1` 换算到目标图作初值；依赖标定准确,适合非电梯、坐标可信场景。

### 依赖话题
- 等待 `/waiting_for_initial_pose`（`std_msgs/Bool`，LOAD 就绪信号）
- 等待 `/Odometry`（`nav_msgs/Odometry`，确认 laserMapping 活起来）
- 发布 `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`，触发重定位）
- 等待 `/map_to_odom_flag`（`std_msgs/Bool`，来自 FAST_LIO_GLOBAL 全局重定位完成）

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

## 3. 目录结构与配置

```
all_project/
├── CMakeLists.txt          # project(all_project),编译 map_switch / tf_publish 两个可执行
├── package.xml
├── config/
│   ├── floors.yaml         # maps 列表:各楼层 id / pcd / gridmap / exit_* 锚点 / 到 map1 变换 + 全局开关
│   └── mid360.yaml         # tf_publish 用:frame/topic/UDP/频率/开关 等
├── launch/
│   └── project.launch      # 整体工程入口:加载全局参数 + 静态 TF + map_switch_node + p2l + rviz
│                           #   各层定位节点由 map_switch_node 运行时 rosrun 拉起(不再用 .sh/.launch)
├── example/                # 上位机地图切换示例(不依赖 ROS,纯 socket)
│   ├── map_switch_client.hpp   # header-only 接口:SendLoad(非阻塞)+SendReloc(阻塞)
│   └── map_switch_client.cpp   # 单次完整切换演示:发 LOAD→延时→发 RELOC
├── map_switch/             # 节点1:楼层切换(按职责拆分)
│   ├── include/{protocol.hpp, tcp_server.hpp, node_launcher.hpp, map_switch_node.hpp}
│   └── src/{map_switch.cpp(main), map_switch_node.cpp, tcp_server.cpp, node_launcher.cpp}
└── tf_publish/             # 节点2:位姿/激光 UDP 上报
    ├── include/tf_publish_node.hpp
    └── src/{tf_publish.cpp(main), tf_publish_node.cpp}
```

---

## 4. 编译与运行

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash

# 整体编排（推荐）
roslaunch all_project project.launch

# 或单独运行
rosrun all_project map_switch     # 楼层切换 TCP 服务(监听 :6050)
rosrun all_project tf_publish     # 状态 UDP 上报
```

运行前确认：
- `maps` 参数已从 `config/floors.yaml` 加载，`pcd`/`gridmap` 文件在 `fast_lio_global/PCD/` 下存在；
- `rosrun` 在 PATH 中（已 source ROS）；`global_localization.py`/`transform_fusion.py` 具可执行权限；
- 各被拉起节点所属包（`pcl_ros`、`map_server`、`fast_lio_global`、`all_project`）已编译/可被 rosrun 找到；
- 切换依赖的 `/waiting_for_initial_pose`、`/map_to_odom_flag` 话题存在。

### 上位机调用

上位机通过 TCP 向 `map_switch` 节点发送两阶段切换指令：

- **C++ 接口**：`example/map_switch_client.hpp`（header-only，无需 ROS）：
  - `SendLoad(addr, port, map_id, cb)`：非阻塞发 CMD_LOAD，地图加载完成时回调 `cb(ok)`
  - `SendReloc(addr, port, map_id, x, y, yaw)`：阻塞发 CMD_RELOC，等重定位完成返回 bool
  - `example/map_switch_client.cpp`：单次完整切换演示——发 LOAD → `sleep` 模拟乘梯 → 等 LOAD 成功 → 发 RELOC
  - 编译：`g++ -std=c++11 map_switch_client.cpp -o run -pthread`，运行一次即完成一次地图切换
- **电梯控制集成**：见 `ElevatorControl`（Modbus TCP/RTU 二合一），已封装完整乘梯 + 换层流程（三步函数）：`callElevatorAndOpenDoor` → `closeDoorLoadMapRideAndOpenDoor` → `closeDoorAndWaitMapThenReloc`

---

## 5. 依赖

- catkin：`roscpp`、`roslib`、`rospy`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf`、`pcl_ros`、`eigen_conversions`
- 系统库：`CURL`、`yaml-cpp`、`OpenCV`（已 find_package，部分当前未强链接）
- 上位机电梯控制：`libmodbus`（`ElevatorControl` 依赖，`sudo apt install libmodbus-dev`）
