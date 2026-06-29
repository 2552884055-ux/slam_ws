# ElevatorControl —— Modbus 电梯控制 + 两阶段地图切换（TCP / RTU 二合一）

巡检机器人跨楼层乘梯的**上位机控制端**：通过 Modbus 与电梯通信完成召梯/进梯/乘梯/出梯全流程，
并与机器人侧 `all_project/map_switch` 节点配合实现**两阶段地图切换**（LOAD + RELOC）。

> **TCP 与 RTU 二合一**：libmodbus 创建上下文后读写接口完全一致，本包用同一套代码、
> 在构造时选择传输方式——TCP(网络 IP+端口) 或 RTU(串口)。无需维护两份代码。

---

## 1. 乘梯 + 换层完整流程

```
步骤0  dumpAllRegisters()                       打印全部寄存器,确认与电梯通信正常
步骤1  callElevatorAndOpenDoor(from_floor)
       → 等电梯上线/激活 → 召梯 → 等到达 → 开门 → 机器狗进梯
步骤2  closeDoorLoadMapRideAndOpenDoor(to_floor, target_map, addr, port)
       → 关门(确认进梯) → 非阻塞发 LOAD(乘梯期间后台加载目标层地图)
       → 发乘梯指令 → 等到达目标层(每20s重发) → 开门 → 机器狗出梯
步骤3  closeDoorAndWaitMapThenReloc(target_map, addr, port, x, y, yaw)
       → 关门 → 断开电梯连接 → 等 LOAD 完成 → 阻塞发 RELOC(重定位) → 切换完成
```

**设计要点**：地图加载(5~15s)被藏在乘梯时间(10~30s)内后台完成，出梯后只剩重定位(几秒)，用户感知切换时间大幅缩短。

---

## 2. 关键接口

### `RobotElevatorClient`

构造函数二选一（其余调用完全相同）：

```cpp
RobotElevatorClient client("127.0.0.1", 8000, 1);              // TCP: ip, port, slave_id
RobotElevatorClient client("/dev/ttyUSB0", 9600,'N',8,1,1);    // RTU: device, baud, parity, data, stop, slave_id
```

| 方法 | 说明 |
|------|------|
| `dumpAllRegisters()` | 读取并打印全部寄存器(输入+保持, 地址 0~14)及状态解析，调试用 |
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

## 3. TCP / RTU 选择

在 `main.cpp` 里改 client 的构造方式即可（已用注释标好两行，二选一）：

```cpp
// 【TCP】网络:IP、端口、从机ID
RobotElevatorClient client("127.0.0.1", 8000, 1);
// 【RTU】串口:设备、波特率、校验位、数据位、停止位、从机ID
// RobotElevatorClient client("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);
```

| 项目 | TCP | RTU |
|------|-----|-----|
| 通信 | 网络 Modbus TCP | 串口 Modbus RTU |
| 构造 | `(ip, port, slave_id)` | `(device, baud, parity, data, stop, slave_id)` |
| 重试间隔 | 1000ms（网络往返） | 100ms（低延迟串口） |

---

## 4. 目录结构

```
ElevatorControl/
├── main.cpp                       # 乘梯 + 换层调用示例(TCP 默认, RTU 注释备选)
├── include/
│   ├── elevator_controller.hpp    # 电梯控制器(TCP/RTU 两个构造函数,寄存器读写/状态/控制)
│   ├── robot_elevator_client.hpp  # 封装完整乘梯 + 两阶段切换流程
│   └── map_switch.hpp             # 两阶段地图切换协议(SendLoad / SendReloc)
└── src/
    ├── elevator_controller.cpp
    ├── robot_elevator_client.cpp
    └── map_switch.cpp
```

---

## 5. 参数配置（`main.cpp` 顶部）

| 变量 | 说明 |
|------|------|
| client 构造参数 | TCP：电梯 IP/端口；RTU：串口设备/波特率等 |
| `server_addr` | 机器人侧 `map_switch` 节点 IP（默认本机联调 `127.0.0.1`，实际为机器人 IP） |
| `map_switch_PORT` | map_switch 节点端口（默认 `6050`） |
| `from_floor` / `to_floor` | 出发 / 目标楼层 |
| `target_map` | 目标地图 ID（`map1`~`map5`，对应 `floors.yaml` 的 `id`） |
| `reloc_x/y/yaw` | 重定位初始位姿（服务端 `use_coord_transform=false` 时不使用） |

---

## 6. 依赖

- **硬件**：支持 Modbus TCP(网络可达) 或 Modbus RTU(串口) 的电梯控制器
- **库**：`libmodbus`（`sudo apt install libmodbus-dev`）
- **配合**：机器人侧运行 `roslaunch all_project project.launch`（`map_switch` 节点监听 `:6050`）

---

## 7. 编译与运行

```bash
mkdir build && cd build
cmake ..
make
./ElevatorControl       # 可执行名为 ElevatorControl
# 或直接 g++ -std=c++11 main.cpp src/*.cpp -I include -lmodbus -pthread -o run
```

`main.cpp` 流程：**步骤0 打印全部寄存器** → 步骤1 召梯+开门 → 步骤2 关门+LOAD+乘梯+开门 → 步骤3 关门+RELOC。

---

## 8. 本机联调（无真实电梯 / map_switch 时）

用 `src/ElevatorSimulator` 的两个模拟器（`elevator_sim` + `map_switch_sim`）纯本机跑通：

- **TCP**：`main.cpp` 用 TCP 构造 → `elevator_sim --tcp 127.0.0.1 8000` + `map_switch_sim 6050`
- **RTU**：`main.cpp` 用 RTU 构造 → 两个 USB-TTL 交叉对接 → `elevator_sim --rtu /dev/ttyUSB0 ...` + `map_switch_sim 6050`

详见 `src/ElevatorSimulator/README.md`（§6.1 TCP / §6.2 RTU）。接真实设备时把 IP / 串口设备 / `server_addr` 改回实际值。
