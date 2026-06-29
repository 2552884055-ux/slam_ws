// 电梯控制客户端 (Modbus TCP / RTU 二合一)
// 封装完整的乘梯 + 两阶段地图切换流程：召梯/进梯/乘梯/出梯/重定位
#pragma once

#include "elevator_controller.hpp"
#include "map_switch.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <bitset>
#include <atomic>

class RobotElevatorClient {
public:
    // TCP 构造:IP、Modbus TCP 端口、从机ID
    RobotElevatorClient(const std::string& ip, int port = 8000, int slave_id = 1);
    // RTU 构造:串口设备、波特率、校验位、数据位、停止位、从机ID
    RobotElevatorClient(const std::string& device, int baudrate, char parity,
                        int data_bits, int stop_bits, int slave_id);

    // 读取并打印全部寄存器(输入寄存器+保持寄存器, 地址 0~14),含状态解析,便于调试观察
    void dumpAllRegisters();

    // ===== 单元测试/调试用 =====
    void testCallElevator(int floor); // 单测:召梯到指定层(发指令 + 等到达)
    void testOpenMainDoor();        // 单测:开主门(发指令 + 等开到位)
    void testCloseDoor();           // 单测:关门(发指令 + 等关到位)

    // 召梯到出发层并开门,等机器狗进入电梯后由上层调用下一步
    bool callElevatorAndOpenDoor(int FromFloor);

    // 机器狗已进梯:关门 → 非阻塞发 LOAD(预加载目标层地图) → 发乘梯指令
    //              → 等到达目标层 → 开门(机器狗出梯)
    // 地图加载在乘梯期间后台进行,完成后 m_load_ok 置位
    bool closeDoorLoadMapRideAndOpenDoor(int ToFloor, unsigned long target_map,
                                      const char* server_addr, int map_switch_PORT);

    // 机器狗已出梯:关门 → 等地图加载完成(m_load_ok) → 阻塞发 RELOC 重定位
    // x/y/yaw:机器人在源图坐标系下的位姿,始终需要传入;
    //   服务端 use_coord_transform=false 时不使用(改用 floors.yaml 各层锚点),
    //   服务端 use_coord_transform=true  时将其换算到目标图作为重定位初值。
    bool closeDoorAndWaitMapThenReloc(unsigned long target_map,
                               const char* server_addr, int map_switch_PORT,
                               float x, float y, float yaw);

private:
    ElevatorController m_controller;
    std::atomic<bool>  m_load_ok{false};  // LOAD 成功信号:由 SendLoad 回调(后台线程)置位

    // ===== 带重试的控制指令(每次重试间隔 intervalMs 毫秒) =====
    bool sendRideCommandWithRetry(const std::string& floor, int retryLimit = 10, int intervalMs = 10);
    bool requestOpenMainDoorWithRetry(int retryLimit = 100, int intervalMs = 1000);
    bool requestCloseDoorWithRetry(int retryLimit = 100, int intervalMs = 1000);

    // ===== 等待逻辑 =====
    bool waitElevatorOnlineAndActive(int timeout_sec = 100);   // 等电梯上线并激活
    bool waitElevatorArrives(const std::string& floor, int timeout_sec = 180);  // 等电梯到达指定层
};
