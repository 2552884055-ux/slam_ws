# ElevatorControl_RTU —— Modbus RTU 电梯控制 + 两阶段地图切换(上位机)

巡检机器人跨楼层乘梯的**上位机控制端**，通过 Modbus RTU(串口)与电梯通信，完成召梯/进梯/乘梯/出梯全流程，并与机器人侧的 `all_project/map_switch` 节点配合，实现**两阶段地图切换**（LOAD + RELOC）。

---

## 1. 乘梯 + 换层完整流程

```
步骤1  导航到候梯点
       callElevatorAndOpenDoor(from_floor)
       → 等电梯上线/激活 → 召梯 → 等到达 → 开门
       → 机器狗进梯

步骤2  机器狗进梯后(停稳)
       closeDoorLoadMapRideAndOpenDoor(to_floor, target_map, addr, port)
       → 关门(确认进梯)
       → 非阻塞发 LOAD(目标层地图在乘梯期间后台加载,m_load_ok 置位)
       → 发乘梯指令 → 等到达目标层(每20s重发) → 开门
       → 机器狗出梯

步骤3  机器狗出梯后(停稳)
       closeDoorAndWaitMapThenReloc(target_map, addr, port, x, y, yaw)
       → 关门 → 断开电梯连接
       → 等 m_load_ok(地图加载完成,乘梯够长则无需等待)
       → 阻塞发 RELOC(重定位) → 切换完成
```

**设计要点**：地图加载(通常 5~15s)被藏在乘梯时间(10~30s)内后台完成，出梯后只剩重定位（几秒），用户感知切换时间大幅缩短。

---

## 2. 关键接口

### `RobotElevatorClient`

| 方法 | 说明 |
|------|------|
| `callElevatorAndOpenDoor(FromFloor)` | 召梯到出发层并开门；返回 true 表示门已开，机器狗可进梯 |
| `closeDoorLoadMapRideAndOpenDoor(ToFloor, target_map, addr, port)` | 关门 + 非阻塞发 LOAD + 发乘梯指令 + 等到达 + 开门 |
| `closeDoorAndWaitMapThenReloc(target_map, addr, port, x, y, yaw)` | 关门 + 等地图加载完成 + 阻塞发 RELOC 重定位 |

### 地图切换接口（`map_switch.hpp`）

| 函数 | 阻塞性 | 说明 |
|------|--------|------|
| `SendLoad(addr, port, map_id, cb)` | **非阻塞** | 发 CMD_LOAD，完成时回调 `cb(ok)` |
| `SendReloc(addr, port, map_id, x, y, yaw)` | **阻塞** | 发 CMD_RELOC，返回是否成功 |

`x/y/yaw`：机器人在源图坐标系下的位姿，始终需要传入；
- 服务端 `use_coord_transform=false`（默认）：不使用，改用 `floors.yaml` 各层锚点；
- 服务端 `use_coord_transform=true`：换算到目标图作为重定位初值。

---

## 3. 目录结构

```
ElevatorControl_RTU/
├── main.cpp                    # 乘梯 + 换层调用示例
├── include/
│   ├── elevator_controller.hpp # Modbus RTU 电梯控制器(寄存器读写/状态/控制指令)
│   ├── robot_elevator_client.hpp # 封装完整乘梯 + 两阶段切换流程
│   └── map_switch.hpp          # 两阶段地图切换协议(req_frame / SendLoad / SendReloc)
└── src/
    ├── elevator_controller.cpp
    ├── robot_elevator_client.cpp
    └── map_switch.cpp
```

---

## 4. 参数配置（`main.cpp` 顶部）

| 变量 | 说明 |
|------|------|
| `server_addr` | 机器人侧 `map_switch` 节点 IP（默认 `192.168.2.100`） |
| `map_switch_PORT` | map_switch 节点端口（默认 `6050`） |
| `from_floor` | 出发楼层 |
| `to_floor` | 目标楼层 |
| `target_map` | 目标地图 ID（`map1`~`map5`，对应 `floors.yaml` 的 `id`） |
| `reloc_x/y/yaw` | 重定位初始位姿（服务端 `use_coord_transform=false` 时不使用） |
| 串口参数 | `/dev/ttyUSB0`、`9600 N 8 1`、从机 ID `1` |

---

## 5. 依赖

- **硬件**：Modbus RTU 串口电梯控制器
- **库**：`libmodbus`（`sudo apt install libmodbus-dev`）
- **配合**：机器人侧运行 `roslaunch all_project project.launch`（`map_switch` 节点监听 `:6050`）

---

## 6. 编译与运行

```bash
mkdir build && cd build
cmake ..
make
./ElevatorControl_RTU   # 或直接 g++ -std=c++11 main.cpp src/*.cpp -I include -lmodbus -pthread -o run
```
