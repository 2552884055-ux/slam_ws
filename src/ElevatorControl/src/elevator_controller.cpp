// 电梯控制器实现 (Modbus TCP / RTU 二合一)
#include "elevator_controller.hpp"
#include <vector>

// ===================== 构造与析构 ===================== //

// TCP 构造
ElevatorController::ElevatorController(const std::string& ip, int port, int slave_id)
    : m_transport(Transport::TCP), m_retryIntervalMs(RETRY_INTERVAL_MS_TCP),
      m_slave_id(slave_id), m_ip(ip), m_port(port)
{
    m_ctx = modbus_new_tcp(m_ip.c_str(), m_port);
    if (m_ctx == nullptr) {
        throw std::runtime_error("无法创建 Modbus TCP 上下文");
    }
    setupAndConnect();
}

// RTU 构造：device 为空时自动扫描 /dev/ttyUSB* 找梯控
ElevatorController::ElevatorController(const std::string& device, int baudrate, char parity,
                                       int data_bits, int stop_bits, int slave_id)
    : m_transport(Transport::RTU), m_retryIntervalMs(RETRY_INTERVAL_MS_RTU),
      m_slave_id(slave_id), m_baudrate(baudrate),
      m_parity(parity), m_data_bits(data_bits), m_stop_bits(stop_bits)
{
    if (device.empty()) {
        if (access("/dev/elevator_rs485", F_OK) == 0) {
            m_device = "/dev/elevator_rs485";
            std::cout << "[RTU] 使用 udev 别名: " << m_device << std::endl;
        } else {
            m_device = detectRtuDevice(baudrate, parity, data_bits, stop_bits, slave_id);
            if (m_device.empty())
                throw std::runtime_error("未能自动识别梯控串口设备");
        }
    } else {
        // 显式指定路径：先验证设备文件存在，不存在立即报错
        if (access(device.c_str(), F_OK) != 0)
            throw std::runtime_error("串口设备不存在: " + device);
        m_device = device;
    }

    m_ctx = modbus_new_rtu(m_device.c_str(), m_baudrate, m_parity, m_data_bits, m_stop_bits);
    if (m_ctx == nullptr) {
        throw std::runtime_error("无法创建 Modbus RTU 上下文");
    }
    modbus_set_response_timeout(m_ctx, 1, 0);  // 串口响应超时 1s
    setupAndConnect();
}

// 设置从机ID + 带重试连接(两种传输共用)
void ElevatorController::setupAndConnect() {
    if (modbus_set_slave(m_ctx, m_slave_id) == -1) {
        modbus_free(m_ctx);
        m_ctx = nullptr;
        throw std::runtime_error("设置 Modbus 从站 ID 失败");
    }

    const char* tag = (m_transport == Transport::TCP) ? "TCP" : "RTU";
    bool connected = false;
    for (int attempt = 0; attempt < MAX_RETRY; ++attempt) {
        if (connect()) { connected = true; break; }
        std::cerr << "[" << tag << "] 第 " << attempt + 1 << " 次连接失败: "
                  << modbus_strerror(errno) << "，将在 "
                  << m_retryIntervalMs << "ms 后重试。" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(m_retryIntervalMs));
    }

    if (m_transport == Transport::TCP) {
        if (connected)
            std::cout << "[TCP] 已连接至电梯控制器 " << m_ip << ":" << m_port
                      << " (slave_id=" << m_slave_id << ")" << std::endl;
        else
            std::cerr << "[TCP] 无法连接至电梯控制器 " << m_ip << ":" << m_port
                      << " (slave_id=" << m_slave_id << ")。" << std::endl;
    } else {
        if (connected)
            std::cout << "[RTU] 已打开串口 " << m_device << " " << m_baudrate
                      << " " << m_parity << " " << m_data_bits << " " << m_stop_bits
                      << " (slave_id=" << m_slave_id << ")" << std::endl;
        else
            std::cerr << "[RTU] 无法打开串口 " << m_device << "。" << std::endl;
    }
}

ElevatorController::~ElevatorController() {
    stopCommFlagThread();  // 确保通信线程已停止,否则析构时 std::thread 仍 joinable 会触发 terminate
    if (m_ctx != nullptr) {
        modbus_close(m_ctx);
        modbus_free(m_ctx);
    }
}

// ===================== 连接与断开 ===================== //
bool ElevatorController::connect() {
    if (m_transport == Transport::RTU) {
        // 先关掉已有 fd，避免重连时泄漏
        int old_fd = modbus_get_socket(m_ctx);
        if (old_fd != -1) { close(old_fd); modbus_set_socket(m_ctx, -1); }

        int fd = open(m_device.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "[RTU] 打开串口失败: " << m_device << std::endl;
            return false;
        }
        modbus_set_socket(m_ctx, fd);
    }
    if (modbus_connect(m_ctx) == -1) {
        std::cerr << "[Modbus] 连接失败: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void ElevatorController::disconnect() {
    modbus_close(m_ctx);
    if (m_transport == Transport::RTU) {
        int fd = modbus_get_socket(m_ctx);
        if (fd != -1) { close(fd); modbus_set_socket(m_ctx, -1); }
    }
}

bool ElevatorController::ensureRtuConnected() {
    if (m_transport != Transport::RTU) return true;

    // 快速探测：短超时发一次读，成功直接返回，避免每次调用都加重连锁
    {
        std::lock_guard<std::mutex> lock(modbusMutex);
        modbus_set_response_timeout(m_ctx, 0, 200000);
        uint16_t buf[1];
        bool ok = (modbus_read_input_registers(m_ctx, 0, 1, buf) == 1);
        modbus_set_response_timeout(m_ctx, 1, 0);
        if (ok) return true;
    }

    // 探测失败，进入重连流程，序列化防止多线程并发重建 m_ctx
    std::lock_guard<std::mutex> reconnLock(m_reconnMutex);

    // 再探测一次：可能另一个线程已经重连好了
    {
        std::lock_guard<std::mutex> lock(modbusMutex);
        modbus_set_response_timeout(m_ctx, 0, 200000);
        uint16_t buf[1];
        bool ok = (modbus_read_input_registers(m_ctx, 0, 1, buf) == 1);
        modbus_set_response_timeout(m_ctx, 1, 0);
        if (ok) return true;
    }

    std::cerr << "[RTU] 串口无响应，扫描所有 /dev/ttyUSB* ..." << std::endl;

    // 枚举候选设备：原路径优先，再追加其余
    std::vector<std::string> candidates;
    candidates.push_back(m_device);
    glob_t gl;
    if (glob("/dev/ttyUSB*", 0, nullptr, &gl) == 0) {
        for (size_t i = 0; i < gl.gl_pathc; ++i)
            if (std::string(gl.gl_pathv[i]) != m_device)
                candidates.push_back(gl.gl_pathv[i]);
        globfree(&gl);
    }

    // 用临时独立上下文探测每个候选，不修改 m_ctx / m_device；每个设备尝试两次
    auto probeDevice = [&](const std::string& dev) -> bool {
        modbus_t* tmp = modbus_new_rtu(dev.c_str(), m_baudrate, m_parity,
                                       m_data_bits, m_stop_bits);
        if (!tmp) return false;
        modbus_set_slave(tmp, m_slave_id);
        modbus_set_response_timeout(tmp, 0, 200000);
        modbus_set_error_recovery(tmp, MODBUS_ERROR_RECOVERY_NONE);

        int fd = open(dev.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) { modbus_free(tmp); return false; }
        modbus_set_socket(tmp, fd);

        bool ok = false;
        if (modbus_connect(tmp) == 0) {
            uint16_t probe[1];
            for (int i = 0; i < 2 && !ok; ++i)
                ok = (modbus_read_input_registers(tmp, 0, 1, probe) == 1);
        }
        modbus_close(tmp);
        close(fd);
        modbus_free(tmp);
        return ok;
    };

    for (const auto& dev : candidates) {
        std::cout << "[RTU] 探测 " << dev << " ..." << std::flush;
        if (!probeDevice(dev)) { std::cout << " 无响应" << std::endl; continue; }
        std::cout << " 梯控响应" << std::endl;

        // 找到目标设备，持 modbusMutex 重建正式上下文
        std::lock_guard<std::mutex> lock(modbusMutex);
        modbus_close(m_ctx);
        modbus_free(m_ctx);

        m_ctx = modbus_new_rtu(dev.c_str(), m_baudrate, m_parity,
                               m_data_bits, m_stop_bits);
        modbus_set_slave(m_ctx, m_slave_id);
        modbus_set_response_timeout(m_ctx, 1, 0);
        m_device = dev;

        if (connect()) {
            std::cout << "[RTU] 重连成功，设备: " << dev << std::endl;
            return true;
        }
        std::cerr << "[RTU] 重建上下文后 connect() 失败" << std::endl;
        return false;
    }

    std::cerr << "[RTU] 所有设备均无梯控响应，重连失败" << std::endl;
    return false;
}

// ===================== 通用重试模板 ===================== //
template<typename Func>
int modbusWithRetry(Func f, ElevatorController* self) {
    for (int attempt = 0; attempt < MAX_RETRY; ++attempt) {
        try {
            return f();
        } catch (const std::runtime_error& e) {
            std::cerr << "[Modbus] 第 " << attempt+1 << " 次操作失败: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(self->retryIntervalMs()));
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
    status.isActive = data[3] & 0x0001;      // 投入状态
    status.mainDoorOpen = data[5] & 0x0020;  // 主门状态
    status.viceDoorOpen = data[5] & 0x0080;  // 副门状态
    status.isNormal = data[5] & 0x0008;      // 正常状态
    status.isRuning = data[5] & 0x0004;      // 运行状态
    status.isDownward = data[5] & 0x0002;    // 下行状态
    status.isUpward = data[5] & 0x0001;      // 上行状态

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
        while (true) {
            {
                std::lock_guard<std::mutex> lock(commFlagMutex);
                if (!commFlagRunning) break;
            }
            commCounter++;
            // 直接写，不走长重试；失败立即扫口重连
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(modbusMutex);
                ok = (modbus_write_register(m_ctx, 2, commCounter) != -1);
            }
            if (!ok) {
                std::cerr << "[commFlagThread] 心跳失败，触发重连..." << std::endl;
                ensureRtuConnected();
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
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

// ===================== 自动设备探测 ===================== //
std::string ElevatorController::detectRtuDevice(int baudrate, char parity,
                                                 int data_bits, int stop_bits,
                                                 int slave_id)
{
    // 枚举所有 /dev/ttyUSB* 设备
    glob_t gl;
    if (glob("/dev/ttyUSB*", 0, nullptr, &gl) != 0) {
        std::cerr << "[探测] 未找到任何 /dev/ttyUSB* 设备" << std::endl;
        return "";
    }

    std::string found;
    for (size_t i = 0; i < gl.gl_pathc; ++i) {
        const std::string dev = gl.gl_pathv[i];
        std::cout << "[探测] 尝试 " << dev << " ..." << std::flush;

        modbus_t* ctx = modbus_new_rtu(dev.c_str(), baudrate, parity, data_bits, stop_bits);
        if (!ctx) { std::cout << " 创建上下文失败，跳过" << std::endl; continue; }

        modbus_set_response_timeout(ctx, 0, 200000);
        modbus_set_byte_timeout(ctx, 0, 50000);
        modbus_set_slave(ctx, slave_id);
        modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_NONE);

        int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1) {
            std::cout << " 无法打开设备，跳过" << std::endl;
            modbus_free(ctx);
            continue;
        }
        // 切回阻塞模式供 modbus 使用
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        modbus_set_socket(ctx, fd);

        bool ok = false;
        if (modbus_connect(ctx) == 0) {
            uint16_t buf[1];
            for (int i = 0; i < 2 && !ok; ++i)
                ok = (modbus_read_input_registers(ctx, 0, 1, buf) == 1);
        }
        modbus_close(ctx);
        modbus_free(ctx);
        close(fd);

        if (ok) {
            std::cout << " ✓ 梯控响应，使用此设备" << std::endl;
            found = dev;
            break;
        }
        std::cout << " 无响应" << std::endl;
    }

    globfree(&gl);
    if (found.empty())
        std::cerr << "[探测] 所有 /dev/ttyUSB* 均无梯控响应" << std::endl;
    return found;
}

void ElevatorController::sendRideCommand(const std::string& floor) {
    if (floor.length() != 3)
        throw std::invalid_argument("楼层必须是3字符ASCII,比如 '001'");
    uint16_t data[3];
    data[0] = 0x8004;  // 乘梯信号 + 内呼
    data[1] = static_cast<uint8_t>(floor[0]);
    data[2] = (static_cast<uint8_t>(floor[1]) << 8) | static_cast<uint8_t>(floor[2]);
    writeRegisters(8, 3, data);
}
