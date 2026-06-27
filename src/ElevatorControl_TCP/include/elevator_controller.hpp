
// TCP方式
#pragma once

#include <modbus/modbus.h>
#include <string>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <atomic>
#include <fcntl.h>

constexpr int MAX_RETRY = 100;              // 最大重试次数
constexpr int RETRY_INTERVAL_MS = 1000;    // 重试间隔（毫秒）

class ElevatorController {
private:
    modbus_t *m_ctx;                  // Modbus 上下文
    std::string m_ip;                 // 电梯控制器 IP
    int m_port;                       // Modbus TCP 端口（默认 8000）
    int m_slave_id;                   // 从站地址（部分控制器需要设置）
    
    std::string m_device;             // 串口设备路径
    int m_baudrate;                   // 波特率
    char m_parity;                    // 奇偶校验
    int m_data_bits;                  // 数据位
    int m_stop_bits;                  // 停止位

    std::mutex commFlagMutex;
    std::mutex modbusMutex;
    bool commFlagRunning = false;
    std::thread commFlagThread;
    int commCounter = 0;

public:
    ElevatorController(const std::string& ip, int port = 8000, int slave_id = 1);
    ~ElevatorController();
    
    bool connect();
    void disconnect();

    struct ElevatorStatus {
        bool isOnline = false;          // 电梯设备通讯标志（通信正常）
        bool isActive = false;          // 电梯是否可用（无故障）
        bool mainDoorOpen = false;      // 主门是否完全打开
        bool viceDoorOpen = false;      // 副门是否完全打开
        bool isDownward = false;        // 电梯是否在下行
        bool isUpward = false;          // 电梯是否在上行
        bool isNormal = false;          // 电梯是否处于正常状态
        bool isRuning = false;          // 电梯是否处于运行状态
        std::string currentFloor;       // 当前电梯所在楼层
        std::string callFloor;          // 召梯的目标楼层
    };

    // 获取电梯状态（读取寄存器）
    ElevatorStatus getElevatorStatus();

    void startCommFlagThread();
    void stopCommFlagThread();

    // 电梯控制命令
    void requestOpenMainDoor();
    void requestOpenViceDoor();
    void requestCloseDoor();
    void sendRideCommand(const std::string& floor);

    // === Modbus 读写接口 ===
    // 输入寄存器 (30001~)
    int readInputRegisters(int addr, int nb, uint16_t *dest);
    // 保持寄存器 (40001~)
    int readHoldingRegisters(int addr, int nb, uint16_t *dest);
    // 写单寄存器
    int writeRegister(int addr, uint16_t value);
    // 写多个寄存器
    int writeRegisters(int addr, int nb, const uint16_t *data);
};



