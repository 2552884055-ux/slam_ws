// 电梯控制器 (Modbus) —— TCP / RTU 二合一
//
// libmodbus 创建上下文后(modbus_new_tcp / modbus_new_rtu)，读写接口完全一致，
// 因此本类用两个构造函数分别支持 TCP(网络) 与 RTU(串口)，其余逻辑共用。
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
#include <glob.h>

constexpr int MAX_RETRY = 100;                  // 最大重试次数
constexpr int RETRY_INTERVAL_MS_TCP = 1000;     // TCP 重试间隔(网络往返,较大)
constexpr int RETRY_INTERVAL_MS_RTU = 100;      // RTU 重试间隔(低延迟串口)

class ElevatorController {
public:
    enum class Transport { TCP, RTU };

private:
    modbus_t   *m_ctx = nullptr;      // Modbus 上下文
    Transport   m_transport;          // 传输方式
    int         m_retryIntervalMs;    // 重试间隔(按传输方式取值)
    int         m_slave_id;           // 从站地址

    // TCP 参数
    std::string m_ip;
    int         m_port = 0;

    // RTU 参数
    std::string m_device;
    int         m_baudrate = 0;
    char        m_parity = 'N';
    int         m_data_bits = 8;
    int         m_stop_bits = 1;

    std::mutex commFlagMutex;
    std::mutex modbusMutex;
    std::mutex m_reconnMutex;   // 序列化重连，防止多线程同时重建 m_ctx
    bool commFlagRunning = false;
    std::thread commFlagThread;
    int commCounter = 0;

    // 创建上下文后的共用初始化(设置从机ID + 带重试连接)
    void setupAndConnect();

public:
    // TCP 构造:IP、Modbus TCP 端口(默认 8000)、从机ID
    ElevatorController(const std::string& ip, int port = 8000, int slave_id = 1);
    // RTU 构造:串口设备、波特率、校验位、数据位、停止位、从机ID
    ElevatorController(const std::string& device, int baudrate = 9600, char parity = 'N',
                       int data_bits = 8, int stop_bits = 1, int slave_id = 1);
    ~ElevatorController();

    bool connect();
    void disconnect();

    int  retryIntervalMs() const { return m_retryIntervalMs; }
    Transport transport() const { return m_transport; }

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
    int readInputRegisters(int addr, int nb, uint16_t *dest);   // 输入寄存器 (30001~)
    int readHoldingRegisters(int addr, int nb, uint16_t *dest); // 保持寄存器 (40001~)
    int writeRegister(int addr, uint16_t value);                // 写单寄存器
    int writeRegisters(int addr, int nb, const uint16_t *data); // 写多个寄存器

    // 自动探测：扫描 /dev/ttyUSB*，对每个设备发一次 Modbus 读探测，
    // 返回第一个能正常响应的设备路径；未找到则返回空字符串。
    static std::string detectRtuDevice(int baudrate = 9600, char parity = 'N',
                                       int data_bits = 8, int stop_bits = 1,
                                       int slave_id = 1);

    // RTU 模式下探测当前连接是否正常；无响应时自动重新扫描串口并重连。
    // TCP 模式直接返回 true。返回 false 表示重探测也失败。
    bool ensureRtuConnected();
};
