//RTU方式
#include "elevator_controller.hpp"

// 构造函数
ElevatorController::ElevatorController(const std::string& device, int baudrate, char parity, int data_bits, int stop_bits, int slave_id) 
    : m_device(device), m_baudrate(baudrate), m_parity(parity), m_data_bits(data_bits), m_stop_bits(stop_bits), m_slave_id(slave_id) {

    // 创建Modbus RTU上下文
    m_ctx = modbus_new_rtu(device.c_str(), baudrate, parity, data_bits, stop_bits);
    if (m_ctx == nullptr) {
        throw std::runtime_error("Unable to create Modbus RTU context");
    }

    // 设置从机ID
    if (modbus_set_slave(m_ctx, slave_id) != 0) {
        throw std::runtime_error("设置从机ID失败");
    }

    // 设置响应超时
    modbus_set_response_timeout(m_ctx, 1, 0); // 1秒超时

    // 打开串口连接
    if (modbus_connect(m_ctx) == -1) {
        modbus_free(m_ctx);
        throw std::runtime_error(std::string("连接串口失败: ") + modbus_strerror(errno));
    }
}

// 析构函数
ElevatorController::~ElevatorController() {
    if (m_ctx!= nullptr) {
        modbus_close(m_ctx);
        modbus_free(m_ctx);
    }
}

// 连接函数
bool ElevatorController::connect() {
    int fd = open(m_device.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return false;
    }
    modbus_set_socket(m_ctx, fd);
    return modbus_connect(m_ctx) == 0;
}

// 断开连接函数
void ElevatorController::disconnect() {
    modbus_close(m_ctx);
    close(modbus_get_socket(m_ctx));
}


// ===================== 通用重试模板 ===================== //
template<typename Func>
int modbusWithRetry(Func f, ElevatorController* self) {
    for (int attempt = 0; attempt < MAX_RETRY; ++attempt) {
        try {
            return f();
        } catch (const std::runtime_error& e) {
            std::cerr << "[Modbus] 第 " << attempt+1 << " 次操作失败: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_INTERVAL_MS));

        }
    }
    throw std::runtime_error("Modbus 操作重试失败超过最大次数");
}

// ===================== 读写函数（带重试） ===================== //
int ElevatorController::readInputRegisters(int addr, int nb, uint16_t *dest) {
    return modbusWithRetry([&]() {
        std::lock_guard<std::mutex> lock(modbusMutex);
        int rc = modbus_read_input_registers(m_ctx, addr, nb, dest);
        if (rc == -1) throw std::runtime_error(modbus_strerror(errno));
        return rc;
    }, this);
}

int ElevatorController::readHoldingRegisters(int addr, int nb, uint16_t *dest) {
    return modbusWithRetry([&]() {
        std::lock_guard<std::mutex> lock(modbusMutex);
        int rc = modbus_read_registers(m_ctx, addr, nb, dest);
        if (rc == -1) throw std::runtime_error(modbus_strerror(errno));
        return rc;
    }, this);
}

int ElevatorController::writeRegister(int addr, uint16_t value) {
    return modbusWithRetry([&]() {
        std::lock_guard<std::mutex> lock(modbusMutex);
        int rc = modbus_write_register(m_ctx, addr, value);
        if (rc == -1) throw std::runtime_error(modbus_strerror(errno));
        return rc;
    }, this);
}

int ElevatorController::writeRegisters(int addr, int nb, const uint16_t *data) {
    return modbusWithRetry([&]() {
        std::lock_guard<std::mutex> lock(modbusMutex);
        int rc = modbus_write_registers(m_ctx, addr, nb, data);
        if (rc == -1) throw std::runtime_error(modbus_strerror(errno));
        return rc;
    }, this);
}

// ==========================信息读取函数============================ //
// 读取电梯状态（共13个寄存器）
ElevatorController::ElevatorStatus ElevatorController::getElevatorStatus() {
    // // ==========================方案一============================ //
    // uint16_t data_3000[5];
    // uint16_t data_4000[6];
    // readInputRegisters(3, 5, data_3000);   // 30001~30013
    // readHoldingRegisters(5, 6, data_4000); // 40001~40013

    // ElevatorStatus status;

    // // 投入状态：地址3，低字节 bit0
    // status.isActive = data_3000[0] & 0x0001;
    // // 主副门开门状态：地址5，低字节 bit5/bit7
    // status.mainDoorOpen = data_3000[2] & 0x0020;
    // status.viceDoorOpen = data_3000[2] & 0x0080;
    // // 低字节 bit1：下行状态（1为下行）
    // status.isDownward = data_3000[2] & 0x0002;
    // // 低字节 bit0：上行状态（1为上行）
    // status.isUpward = data_3000[2] & 0x0001;
    // // 地址5，高字节 bit8：电梯在线
    // status.isOnline = data_4000[0] & 0x0100; 

    // // 当前楼层 ASCII：地址6（低）+7（高+低）
    // char currentfloorStr[4] = {
    //     static_cast<char>(data_3000[3] & 0x00FF),
    //     static_cast<char>(data_3000[4] >> 8),
    //     static_cast<char>(data_3000[4] & 0x00FF),
    //     '\0'
    // };
    // status.currentFloor = std::string(currentfloorStr);

    // // 目标楼层 ASCII：地址9（低）+10（高+低）
    // char callfloorStr[4] = {
    //     static_cast<char>(data_4000[4] & 0x00FF),
    //     static_cast<char>(data_4000[5] >> 8),
    //     static_cast<char>(data_4000[5] & 0x00FF),
    //     '\0'
    // };
    // status.callFloor = std::string(callfloorStr);


    // ==========================方案二============================ //
    uint16_t data[13];
    readInputRegisters(0, 13, data); // 30001~30013

    ElevatorStatus status;

    // 投入状态：地址3，低字节 bit0
    status.isActive = data[3] & 0x0001;
    // 主副门开门状态：地址5，低字节 bit5/bit7
    status.mainDoorOpen = data[5] & 0x0020;
    status.viceDoorOpen = data[5] & 0x0080;
    // 低字节 bit1：下行状态（1为下行）
    status.isDownward = data[5] & 0x0002;
    // 低字节 bit0：上行状态（1为上行）
    status.isUpward = data[5] & 0x0001;
    // 地址5，高字节 bit8：电梯在线
    status.isOnline = 1; 

    // 当前楼层 ASCII：地址6（低）+7（高+低）
    char currentfloorStr[4] = {
        static_cast<char>(data[6] & 0x00FF),
        static_cast<char>(data[7] >> 8),
        static_cast<char>(data[7] & 0x00FF),
        '\0'
    };
    status.currentFloor = std::string(currentfloorStr);
    
    // 目标楼层 ASCII：地址10（低）+11（高+低）
    char callfloorStr[4] = {
        static_cast<char>(data[10] & 0x00FF),
        static_cast<char>(data[11] >> 8),
        static_cast<char>(data[11] & 0x00FF),
        '\0'
    };
    status.callFloor = std::string(callfloorStr);

    return status;
}

// ============================控制函数================================== //
void ElevatorController::startCommFlagThread() {
    std::lock_guard<std::mutex> lock(commFlagMutex);
    if (commFlagRunning) return; // 已在运行

    commFlagRunning = true;
    commFlagThread = std::thread([this]() {
        try {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(commFlagMutex);
                    if (!commFlagRunning) break;

                    commCounter++;
                    try {
                        writeRegister(2, commCounter);  // 写入通信标志寄存器
                    } catch (const std::exception& e) {
                        std::cerr << "[commFlagThread] writeRegister 异常: " << e.what() << std::endl;
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 循环间隔
            }
        } catch (const std::exception& e) {
            std::cerr << "[commFlagThread] 捕获线程异常: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[commFlagThread] 捕获未知线程异常。" << std::endl;
        }
    });
}

void ElevatorController::stopCommFlagThread() {
    {
        std::lock_guard<std::mutex> lock(commFlagMutex);
        commFlagRunning = false;
    }
    if (commFlagThread.joinable()) {
        commFlagThread.join();
    }
}

// 打开主门操作
void ElevatorController::requestOpenMainDoor() {
    writeRegister(7, 0x0001); // bit0 = 1，开主门
}

// 打开副门操作
void ElevatorController::requestOpenViceDoor() {
    writeRegister(7, 0x0002); // bit1 = 1，开副门
}

// 关门操作
void ElevatorController::requestCloseDoor() {
    writeRegister(7, 0x0000); // bit0 = 0，bit1 = 0 关门
}

// 发送乘梯命令
void ElevatorController::sendRideCommand(const std::string& floor) {
    if (floor.length()!= 3) {
        throw std::invalid_argument("楼层必须是3字符ASCII,比如 ' 10'");
    }
    uint16_t data[3];
    data[0] = 0x8004;   //乘梯信号+内呼
    // data[1] = 0x0001 | static_cast<uint8_t>(floor[0]);//呼副梯
    data[1] = static_cast<uint8_t>(floor[0]);//呼主梯
    data[2] = (static_cast<uint8_t>(floor[1]) << 8) | static_cast<uint8_t>(floor[2]);
    writeRegisters(8, 3, data); // 写入地址8~10，共3个寄存器（40009~40011）
}

