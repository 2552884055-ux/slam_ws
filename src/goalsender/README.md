# goalsender（goal_sender）—— 里程计位姿 UDP 上报节点

将 FAST-LIO 输出的里程计位姿缓存下来，通过 **UDP 请求-响应**（自定义二进制帧协议）对外发布机器人当前位置 `(x, y, z, yaw)`。常用于上位机 / 调度系统按需拉取机器人实时位姿。

> 包名 `goal_sender`，可执行 `goal_sender_node`（ROS 节点名 `lidar_position_sender_node`）。
> 数据**只走 UDP**，本包不发布任何 ROS 话题。

---

## 1. 工作原理

```
/Odometry (nav_msgs/Odometry)
        │  odometryCallback: 缓存最新 x,y,z 与 yaw(由四元数解算)
        ▼
  goal_sender_node ──── UDP 接收线程(recvfrom, 绑定 local_port)
        │
        │  收到请求帧 (type==REQUEST 且 subObj==10)
        ▼
  向 target_ip:target_port 回发两帧 RESPONSE：
    ① XY   帧 (subObj=10): data3=x, data4=y
    ② ZYaw 帧 (subObj=11): data3=z, data4=yaw(rad)
```

- 节点订阅里程计，持续更新「最新位姿」；
- 外部客户端发来请求帧后，节点立即回发两帧应答（XY 与 Z+Yaw）；
- 若配置了地图参数（`map_width/height > 0`），还会按 `map_resolution`/`map_origin_*` 把世界坐标换算为像素坐标用于日志。

---

## 2. 节点

| 可执行 | 源文件 | 说明 |
|--------|--------|------|
| `goal_sender_node` | `ros_goal_sender.cpp` + `src/network/{socket,networkBase,tcpUdp}.cpp` | 订阅里程计、UDP 服务、请求-响应应答 |

---

## 3. 话题

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `~odom_topic`（默认 `/Odometry`） | `nav_msgs/Odometry` |
| 发布 | 无（数据经 UDP 发出） | — |

---

## 4. 参数

| 参数 | 默认（代码 / launch） | 说明 |
|------|----------------------|------|
| `~target_ip` | `192.168.110.180` | 应答帧目标 IP |
| `~target_port` | 代码 `8111` / launch `8115` | 应答帧目标 UDP 端口 |
| `~local_port` | `8112` | 本地监听端口（接收请求） |
| `~odom_topic` | `/Odometry` | 订阅的里程计话题 |
| `~map_resolution` | `0.05` | 米/像素（世界→像素换算） |
| `~map_origin_x` | `0.0` / launch `-10.0` | 地图原点 X（米） |
| `~map_origin_y` | `0.0` / launch `-10.0` | 地图原点 Y（米） |
| `~map_width` | `0` / launch `400` | 地图宽（像素），`>0` 才启用像素换算 |
| `~map_height` | `0` / launch `400` | 地图高（像素） |

---

## 5. UDP 协议

- 传输：`AF_INET / SOCK_DGRAM`，本地绑定 `INADDR_ANY:local_port`。
- 帧头结构（`include/network/frame.hpp`、`include/data_type.h`）：
  ```cpp
  struct frame {
      unsigned int source, dest, type, len_data, checksum, reserve;
      char data[];   // 内含打包的 WirePayload
  };
  ```
- 触发条件：收到帧 `size >= sizeof(frame)` 且 `type == REQUEST(0)` 且 `subObj == 10`(SUBOBJ_ROBOT_NAV_XY)。
- 应答两帧（`type = RESPONSE`）：
  - **subObj=10（XY）**：`hasData=1, data1=position_id, data2=0(status), data3=x, data4=y`
  - **subObj=11（ZYaw）**：`hasData=1, data1=同上, data2=0, data3=z, data4=yaw(rad)`

---

## 6. 编译与运行

```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash

# 启动（推荐用 launch 配齐参数）
roslaunch goal_sender goal_sender.launch \
    target_ip:=192.168.110.180 target_port:=8115 local_port:=8112 odom_topic:=/Odometry

# 或直接运行
rosrun goal_sender goal_sender_node
```

验证流程：节点订阅到里程计 → 外部客户端向 `local_port` 发送请求帧（`type=REQUEST, subObj=10`）→ 节点回发 subObj 10/11 两帧到 `target_ip:target_port`。

---

## 7. 依赖

- catkin、`roscpp`、`geometry_msgs`、`tf`、`Threads`（C++17）
- 代码使用 `nav_msgs/Odometry`，但 `package.xml` 未显式声明 `nav_msgs`，依赖其传递可用；规范起见建议补充。

---

## 8. 目录结构

```
goalsender/
├── ros_goal_sender.cpp        # 主节点
├── src/network/               # socket / tcpUdp / networkBase 网络封装
├── include/                   # frame.hpp / data_type.h / socket.h ...
├── launch/goal_sender.launch
├── CMakeLists.txt
└── package.xml
```
