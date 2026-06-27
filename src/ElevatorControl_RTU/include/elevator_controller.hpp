//RTU方式
#ifndef ELEVATOR_CONTROLLER_H
#define ELEVATOR_CONTROLLER_H

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
constexpr int RETRY_INTERVAL_MS = 100;    // 重试间隔（毫秒）

class ElevatorController {
private:
    modbus_t *m_ctx;
    std::string m_device;
    int m_baudrate;
    char m_parity;
    int m_data_bits;
    int m_stop_bits;
    int m_slave_id;

    std::mutex commFlagMutex;
    std::mutex modbusMutex;
    bool commFlagRunning = false;
    std::thread commFlagThread;
    int commCounter = 0;

public:
    ElevatorController(const std::string& device, int baudrate = 9600, char parity = 'N', int data_bits = 8, int stop_bits = 1, int slave_id = 1);
    ~ElevatorController();
    
    bool connect();
    void disconnect();

    struct ElevatorStatus {
        bool isOnline = false;          //电梯设备通讯标志（通信正常）
        bool isActive = false;          //电梯是否可用（无故障）
        bool mainDoorOpen = false;      //主门是否完全打开
        bool viceDoorOpen = false;      //副门是否完全打开
        bool isDownward = false;        //电梯是否在下行
        bool isUpward = false;          //电梯是否在上行      
        std::string currentFloor;       //当前电梯所在楼层
        std::string callFloor;          //召梯的目标楼层
    };

    // 读取电梯状态（共13个寄存器）
    ElevatorStatus getElevatorStatus();

    void startCommFlagThread();
    void stopCommFlagThread();
    void requestOpenMainDoor();
    void requestOpenViceDoor();
    void requestCloseDoor();
    void sendRideCommand(const std::string& floor);

    // 读取输入寄存器0x04(30001~30013)
    int readInputRegisters(int addr, int nb, uint16_t *dest);
    // 读取保持寄存器0x03(40001~40013)
    int readHoldingRegisters(int addr, int nb, uint16_t *dest);
    // 写入单个寄存器0x06(40001~40013)
    int writeRegister(int addr, uint16_t value);
    // 写入多个寄存器0x10(40001~40013)
    int writeRegisters(int addr, int nb, const uint16_t *data);
};


#endif      