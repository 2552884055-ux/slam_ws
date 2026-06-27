
#include "elevator_controller.hpp"

// ===================== 构造与析构 ===================== //

//MODBUS TCP 构造函数
ElevatorController::ElevatorController(const std::string& ip, int port, int slave_id)
    : m_ip(ip), m_port(port), m_slave_id(slave_id)
{
    m_ctx = modbus_new_tcp(m_ip.c_str(), m_port);
    if (m_ctx == nullptr) {
        throw std::runtime_error("无法创建 Modbus TCP 上下文");
    }

    if (modbus_set_slave(m_ctx, m_slave_id) == -1) {
        modbus_free(m_ctx);
        throw std::runtime_error("设置 Modbus 从站 ID 失败");
    }

    bool connected = false;
    for (int attempt = 0; attempt < MAX_RETRY; ++attempt) {
        if (modbus_connect(m_ctx) == 0) {
            connected = true;
            break;
        }
        std::cerr << "[TCP] 第 " << attempt+1 << " 次连接失败: "
                  << modbus_strerror(errno) << "，将在 " 
                  << RETRY_INTERVAL_MS << "ms 后重试。" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_INTERVAL_MS));
    }

    if (!connected) {
        std::cerr << "[TCP] 无法连接至电梯控制器 " << m_ip << ":" << m_port
                  << " (slave_id=" << m_slave_id << ")。" << std::endl;
    } else {
        std::cout << "[TCP] 已连接至电梯控制器 " << m_ip << ":" << m_port
                  << " (slave_id=" << m_slave_id << ")" << std::endl;
    }
}


ElevatorController::~ElevatorController() {
    if (m_ctx != nullptr) {
        modbus_close(m_ctx);
        modbus_free(m_ctx);
    }
}

// ===================== 连接与断开 ===================== //
bool ElevatorController::connect() {
    if (modbus_connect(m_ctx) == -1) {
        std::cerr << "[Modbus] 重连失败: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void ElevatorController::disconnect() {
    modbus_close(m_ctx);
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
            if (!self->connect()) {
                std::cerr << "[Modbus] 重连失败" << std::endl;
            } else {
                std::cerr << "[Modbus] 重连成功" << std::endl;
            }
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

// ===================== 信息读取 ===================== //
ElevatorController::ElevatorStatus ElevatorController::getElevatorStatus() {
    uint16_t data[13];
    readInputRegisters(0, 13, data);
    ElevatorStatus status;
    status.isActive = data[3] & 0x0001; //投入状态
    status.mainDoorOpen = data[5] & 0x0020;//主门状态
    status.viceDoorOpen = data[5] & 0x0080;//副门状态
    status.isNormal = data[5] & 0x0008; //正常状态
    status.isRuning = data[5] & 0x0004; //运行状态
    status.isDownward = data[5] & 0x0002; //下行状态
    status.isUpward = data[5] & 0x0001; //上行状态
    
    status.isOnline = 1; 

    char currentfloorStr[4] = {
        static_cast<char>(data[6] & 0x00FF),
        static_cast<char>(data[7] >> 8),
        static_cast<char>(data[7] & 0x00FF),
        '\0'
    };
    status.currentFloor = std::string(currentfloorStr);

    char callfloorStr[4] = {
        static_cast<char>(data[10] & 0x00FF),
        static_cast<char>(data[11] >> 8),
        static_cast<char>(data[11] & 0x00FF),
        '\0'
    };
    status.callFloor = std::string(callfloorStr);
    return status;
}

// ===================== 控制函数 ===================== //
void ElevatorController::startCommFlagThread() {
    std::lock_guard<std::mutex> lock(commFlagMutex);
    if (commFlagRunning) return;

    commFlagRunning = true;
    commFlagThread = std::thread([this]() {
        try {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(commFlagMutex);
                    if (!commFlagRunning) break;
                    commCounter++;
                    try {
                        writeRegister(2, commCounter);
                    } catch (const std::exception& e) {
                        std::cerr << "[commFlagThread] writeRegister 异常: " << e.what() << std::endl;
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } catch (const std::exception& e) {
            std::cerr << "[commFlagThread] 捕获线程异常: " << e.what() << std::endl;
        }
    });
}

void ElevatorController::stopCommFlagThread() {
    {
        std::lock_guard<std::mutex> lock(commFlagMutex);
        commFlagRunning = false;
    }
    if (commFlagThread.joinable()) commFlagThread.join();
}

void ElevatorController::requestOpenMainDoor() { writeRegister(7, 0x0001); }
void ElevatorController::requestOpenViceDoor() { writeRegister(7, 0x0002); }
void ElevatorController::requestCloseDoor()    { writeRegister(7, 0x0000); }

void ElevatorController::sendRideCommand(const std::string& floor) {
    if (floor.length() != 3)
        throw std::invalid_argument("楼层必须是3字符ASCII,比如 '001'");
    uint16_t data[3];
    data[0] = 0x8004;
    data[1] = static_cast<uint8_t>(floor[0]);
    data[2] = (static_cast<uint8_t>(floor[1]) << 8) | static_cast<uint8_t>(floor[2]);
    writeRegisters(8, 3, data);
}


