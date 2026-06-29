# ElevatorSimulator —— 模拟电梯控制器 + 模拟地图切换服务

没有真实电梯控制器、也没有机器人侧 `all_project/map_switch` 节点时，用本目录的
两个模拟程序给 `ElevatorControl`（TCP/RTU 二合一上位机）做**纯本机联调**：

| 程序 | 角色 | 对接上位机的 | 协议 |
|------|------|-------------|------|
| `elevator_sim` | 模拟电梯（Modbus **从站/server**） | `ElevatorController` 召梯/开关门/乘梯 | Modbus TCP / RTU |
| `map_switch_sim` | 模拟地图切换服务（TCP **server**） | `SendLoad` / `SendReloc` | 自定义 40B 请求 / 16B 回执 |

`ElevatorControl/main.cpp` 用 TCP 还是 RTU，由其构造函数注释切换；模拟器对应用 `--tcp` 或 `--rtu`：
- TCP（`main.cpp` 用 TCP 构造）→ 跑 `elevator_sim --tcp` + `map_switch_sim`（见 §6.1）。
- RTU（`main.cpp` 用 RTU 构造）→ 跑 `elevator_sim --rtu` + `map_switch_sim`（见 §6.2）。

上位机是 Modbus **主站(client)**，`elevator_sim` 是 Modbus **从站(server)**，
寄存器布局与上位机 `elevator_controller.cpp` 的读写/解码方式严格一致。

---

## 1. 模拟的电梯行为

- 上电即「在线 + 激活 + 正常」，停在起始楼层（默认 1 层），门关闭。
- 收到**乘梯指令**（保持寄存器 `8=0x8004`, `9/10`=目标层 ASCII）后：
  - 立即把目标层回显到「召梯楼层」输入寄存器 → 上位机 `sendRideCommandWithRetry` 确认成功；
  - 开始上行/下行，每层耗时 `FLOOR_TRAVEL_MS`(默认 5s)，逐层更新「当前楼层」；
  - 到达后停车，清除运行/上行/下行标志。
- 收到**门控制**（保持寄存器 `7`：`1`=开主门 / `2`=开副门 / `0`=关门）后：
  - 开/关门各耗时 `DOOR_MOVE_MS`(默认 3s)，完成后置位/清除「主门已开」标志；
  - 运行途中忽略开门请求（符合真实电梯逻辑）。
- 通信心跳寄存器 `2` 正常接收（上位机 `startCommFlagThread` 每秒写一次）。

所有状态变化都会打印中文日志，方便观察上位机的每一步指令。

---

## 2. 寄存器映射（依据《机器人乘梯通信协议》）

### 电梯公共数据表 —— 输入寄存器（FC04 只读，电梯 → 机器人）

| 地址 | 含义 |
|------|------|
| `IR[3]` 低字节 bit0 | 电梯投入状态（0退出/1投入）`isActive` |
| `IR[4]` | 电梯通讯计数器，约 1s 自增 1（机器人判电梯在线） |
| `IR[5]` 低字节 | bit0 上行 / bit1 下行 / bit2 运行中 / bit3 正常 / bit5 主门开到位 / bit7 副门开到位 |
| `IR[5]` 高字节 bit1 | 机器人使用中（`0x0200`） |
| `IR[6]/IR[7]` | 电梯轿厢楼层（3 字符 ASCII） |
| `IR[9]` | 门控制指令回显（高字节 bit0 开主门 `0x0100` / bit1 开副门 `0x0200`）+ 乘梯指令确认（低字节 bit2 内呼 `0x0004`） |
| `IR[10]/IR[11]` | 外呼/内呼楼层（3 字符 ASCII） |

### 机器人乘梯数据表 —— 保持寄存器（FC03/06/10 读写，机器人 → 电梯）

| 地址 | 含义 |
|------|------|
| `HR[2]` | 机器人通讯计数器，约 1s 自增 1（`startCommFlagThread`） |
| `HR[7]` | 开门请求：0=关门 / 1=开主门 / 2=开副门 |
| `HR[8]` | 乘梯请求：高字节 bit7 乘梯 + 低字节 bit2 内呼 → `0x8004` |
| `HR[9]/HR[10]` | 用梯楼层（3 字符 ASCII）；`HR[9]` 高字节 bit0：0=主门/1=副门 |

> `IR[4]`、`IR[5]` 高位机器人使用中、`IR[9]` 回显均为协议要求、真实电梯会提供的字段；
> 当前上位机 `getElevatorStatus` 未读取它们，模拟器仍按协议填充以保持行为一致。

---

## 3. 依赖与编译

```bash
sudo apt install libmodbus-dev      # 与上位机共用同一个库

cd src/ElevatorSimulator
mkdir build && cd build
cmake .. && make
# 或直接：
# g++ -std=c++11 elevator_sim.cpp -lmodbus -pthread -o elevator_sim
```

---

## 4. 运行

### 4.1 TCP（对接 ElevatorControl，main.cpp 用 TCP 构造）

```bash
./elevator_sim --tcp 0.0.0.0 8000        # [ip] [port] [起始楼层]
```

上位机 `ElevatorControl/main.cpp` 里把电梯 IP 改成模拟器所在机器的地址
（本机联调用 `127.0.0.1`），端口保持 `8000`，即可跑通召梯/进梯/乘梯/出梯全流程。

> 注意：`--tcp` 第一个参数是模拟器**监听**地址，用 `0.0.0.0` 监听所有网卡；
> 上位机连接时填模拟器的实际 IP。

### 4.2 RTU（对接 ElevatorControl，main.cpp 用 RTU 构造）

RTU 走串口，主站(控制器)与从站(模拟器)各需一个串口。本机用**两个 USB 转 TTL 模块交叉对接**：

```
USB-TTL A (模拟器/从站, /dev/ttyUSB0)      USB-TTL B (控制器/主站, /dev/ttyUSB1)
        TX  ────────────────────────────────►  RX
        RX  ◄────────────────────────────────  TX
        GND ────────────────────────────────  GND
```

```bash
# 确认设备并赋权(首次或权限不足时)
ls /dev/ttyUSB*                       # 通常是 /dev/ttyUSB0 /dev/ttyUSB1
sudo usermod -aG dialout $USER        # 加入 dialout 组(重新登录生效); 或临时 sudo chmod 666 /dev/ttyUSB*

# 终端1：模拟器接 A 端
./elevator_sim --rtu /dev/ttyUSB0 9600 N 8 1 1   # device baud parity data stop slave [起始楼层]

# 终端2：上位机 ElevatorControl/main.cpp 用 RTU 构造,串口设备改成 B 端 /dev/ttyUSB1
#         波特率/校验/数据/停止/从机ID 两端必须一致(9600 N 8 1，从机ID 1)
```

> 两端的波特率、校验位、数据位、停止位、从机 ID **必须完全一致**，否则收发错乱。
> RS232 电平的 USB-TTL 直连即可；若是 RS485 模块，A-A、B-B 对接(无需交叉)。

---

## 5. 可调参数（`elevator_sim.cpp` 顶部）

| 常量 | 含义 | 默认 |
|------|------|------|
| `SIM_TICK_MS` | 仿真步进周期 | 100ms |
| `FLOOR_TRAVEL_MS` | 每层运行耗时 | 5000ms |
| `DOOR_MOVE_MS` | 开/关门耗时 | 3000ms |

把 `FLOOR_TRAVEL_MS` 调大可模拟乘梯时间较长的场景，用来验证
「地图在乘梯期间后台 LOAD、出梯后只剩 RELOC」的两阶段切换设计。

---

## 6. 联调场景

> 编译时 `cmake .. && make` 会同时生成 `elevator_sim` 和 `map_switch_sim` 两个可执行文件。

> 上位机 `main.cpp` 的流程：**步骤0 打印全部寄存器** → 步骤1 召梯+开门
> → 步骤2 关门+LOAD+乘梯+开门 → 步骤3 关门+RELOC。步骤0 先打印寄存器快照，
> 可单独用来确认与电梯通信正常。

### 6.1 TCP 完整 main（`ElevatorControl/main.cpp` 用 TCP 构造）

`main.cpp` 步骤2/3 会调用 `map_switch`，需同时启动两个模拟器。
`main.cpp` 里电梯 IP 与 `server_addr` 本机联调已指向 `127.0.0.1`。

```bash
# 终端1：电梯模拟器(Modbus TCP 从站)
./elevator_sim --tcp 127.0.0.1 8000

# 终端2：地图切换模拟服务端  [port] [load耗时s] [reloc耗时s]
./map_switch_sim 6050 15 5

# 终端3：运行 main
cd ../../ElevatorControl && mkdir -p build && cd build && cmake .. && make
./ElevatorControl               # 可执行名是 ElevatorControl
```

跑通后终端3 打印「乘梯 + 地图切换全部成功」。

### 6.2 RTU 完整 main（`ElevatorControl/main.cpp` 用 RTU 构造）

把 `main.cpp` 的 client 构造改成 RTU 那一行；电梯走串口(两个 USB-TTL 交叉对接，见 §4.2 接线)，
地图切换仍用 TCP 的 `map_switch_sim`：

```bash
# 终端1：电梯模拟器接 A 端
./elevator_sim --rtu /dev/ttyUSB0 9600 N 8 1 1

# 终端2：地图切换模拟服务端
./map_switch_sim 6050 15 5

# 终端3：main.cpp 改用 RTU 构造、串口设备填 B 端 /dev/ttyUSB1，编译运行
cd ../../ElevatorControl && mkdir -p build && cd build && cmake .. && make
./ElevatorControl
```

**map_switch_sim 协议**（与 `map_switch.hpp` 一致）：客户端连上后发 40B `req_frame`
（`cmd` 1=LOAD/2=RELOC、目标图ID、seq、x/y/yaw），服务端延时模拟处理后回 16B
`replay_frame`（`result`+`seq`，seq 必须与请求一致）。后两个参数可调加载/重定位耗时——
把 `load` 设大（如 `./map_switch_sim 6050 20 3`）即可观察「乘梯时间不足、出梯后仍需等待地图加载」的过程。
