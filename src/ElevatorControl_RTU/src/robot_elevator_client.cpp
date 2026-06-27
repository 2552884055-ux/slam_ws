// Modbus RTU 方式电梯控制客户端实现
#include "robot_elevator_client.hpp"

RobotElevatorClient::RobotElevatorClient(const std::string& device, int baudrate, char parity,
                                         int data_bits, int stop_bits, int slave_id)
    : m_controller(device, baudrate, parity, data_bits, stop_bits, slave_id)
{
}

// 将楼层数字格式化为3字符字符串(如 1→"001"),与电梯控制器协议一致
std::string formatFloor(int floor) {
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << floor;
    return ss.str();
}

// 召梯到出发层并开门:等电梯上线/激活 → 若电梯不在目标层则召梯 → 等到达 → 开门
// 返回 true 表示电梯已到达出发层且门已打开,机器狗可进梯
bool RobotElevatorClient::callElevatorAndOpenDoor(int FromFloor) {
    std::string FromFloorStr = formatFloor(FromFloor);

    if (!waitElevatorOnlineAndActive()) {
        std::cerr << "[失败] 电梯激活超时" << std::endl; 
        return false;
    }
    
    auto status = m_controller.getElevatorStatus();
    std::string currentFloor = status.currentFloor;  
    if (currentFloor == FromFloorStr) {
        std::cout << "电梯已在目标楼层，尝试开门..." << std::endl;
        if (!requestOpenMainDoorWithRetry()) {
            std::cerr << "[失败] 开门失败" << std::endl;
            return false;
        }
        return true;
    }

    if (!sendRideCommandWithRetry(FromFloorStr)) {
        std::cerr << "[失败] 召梯失败，指令多次未被确认" << std::endl;
        return false;
    }

    std::cout << "等待电梯到达..." << std::endl;
    if (!waitElevatorArrives(FromFloorStr)) {
        std::cerr << "[失败] 等待超时" << std::endl;
        return false;
    }

    std::cout << "尝试开门..." << std::endl;
    if (!requestOpenMainDoorWithRetry()) {
        std::cerr << "[失败] 开门失败" << std::endl;
        return false;
    }

    return true;
}

// 机器狗已进梯:关门 → 非阻塞发 LOAD(目标层地图在乘梯期间后台加载) → 发乘梯指令
//              → 等到达目标层(每20s重发乘梯指令,超时180s返回失败) → 开门
// m_load_ok 由 SendLoad 回调在后台置位,供后续 closeDoorAndWaitMapThenReloc 使用
bool RobotElevatorClient::closeDoorLoadMapRideAndOpenDoor(int ToFloor,
                                                       unsigned long target_map,
                                                       const char* server_addr,
                                                       int map_switch_PORT) {
    std::string TromFloorStr = formatFloor(ToFloor);

    // 先关门确认机器狗已在梯内,再发 LOAD(避免在进梯过程中就开始预加载)
    std::cout << "关门..." << std::endl;
    if (!requestCloseDoorWithRetry()) {
        std::cerr << "[失败] 关门失败" << std::endl;
        return false;
    }

    // 关门成功(机器狗已确认在梯内):非阻塞预加载目标层地图,乘梯期间后台加载
    m_load_ok = false;
    SendLoad(server_addr, map_switch_PORT, target_map,
        [this](bool ok) { m_load_ok = ok; });

    if (!sendRideCommandWithRetry(TromFloorStr)) {
        std::cerr << "[失败] 楼层指令发送失败" << std::endl;
        return false;
    }

    // 等待电梯到达目标楼层(每20s重新发送乘梯指令,最长等待 timeout_sec)
    std::cout << "等待电梯到达目标楼层..." << std::endl;
    const int retryIntervalSec = 20;
    const int timeoutSec       = 180;
    auto lastRetryTime = std::chrono::steady_clock::now();
    auto startTime     = std::chrono::steady_clock::now();

    while (true) {
        auto status = m_controller.getElevatorStatus();
        if (status.currentFloor == TromFloorStr) {
            std::cout << "电梯已到达目标楼层" << std::endl;
            break;
        }

        std::cout << "目标楼层: [" << TromFloorStr << "]，轿厢当前楼层: [" << status.currentFloor << "]" << std::endl;

        auto now = std::chrono::steady_clock::now();

        // 超时保护
        if (std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count() >= timeoutSec) {
            std::cerr << "[失败] 等待电梯到达超时" << std::endl;
            return false;
        }

        // 每隔 retryIntervalSec 重新发一次楼层指令
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastRetryTime).count() >= retryIntervalSec) {
            std::cout << "[重发] 发送楼层指令：" << TromFloorStr << std::endl;
            sendRideCommandWithRetry(TromFloorStr);
            lastRetryTime = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "尝试开门..." << std::endl;
    if (!requestOpenMainDoorWithRetry()) {
        std::cerr << "[失败] 开门失败" << std::endl;
        return false;
    }

    return true;
}

// 机器狗已出梯:关门 → 断开电梯连接 → 等 m_load_ok(地图加载完成) → 阻塞发 RELOC 重定位
// 若乘梯时间足够长,地图在出梯时已加载完毕,此处无需等待;
// 若乘梯过短,最多等 60s(600×100ms),超时则返回失败。
bool RobotElevatorClient::closeDoorAndWaitMapThenReloc(unsigned long target_map,
                                                const char* server_addr,
                                                int map_switch_PORT,
                                                float x, float y, float yaw) {
    std::cout << "[电梯任务完成阶段] 开始关门..." << std::endl;

    if (!requestCloseDoorWithRetry()) {
        throw std::runtime_error("关门失败");
        return false;
    }

    std::cout << "[乘梯完成]" << std::endl;

    m_controller.stopCommFlagThread();  // 关闭通信标志线程
    m_controller.disconnect();          // 断开连接

    // 等地图加载完成(进梯时 SendLoad 已非阻塞发出,乘梯期间后台加载)
    // 若乘梯较快,此处可能短暂等待;若加载失败则直接报错
    if (!m_load_ok) {
        std::cout << "[等待地图加载完成...]" << std::endl;
        for (int i = 0; i < 600 && !m_load_ok; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    if (!m_load_ok) {
        std::cerr << "[地图加载失败或超时,无法重定位]" << std::endl;
        return false;
    }

    // 地图已加载完成,发 RELOC(阻塞,返回是否成功)
    return SendReloc(server_addr, map_switch_PORT, target_map, x, y, yaw);
}


// =========================== 带重试的控制指令 ===============================

// 发乘梯指令,等控制器回显 callFloor 确认后返回 true;超过重试次数返回 false
bool RobotElevatorClient::sendRideCommandWithRetry(const std::string& floor, int retryLimit, int intervalMs) {
    for (int i = 0; i < retryLimit; ++i) {
        std::cout << "[第 " << (i + 1) << " 次发送乘梯指令：前往 " << floor << "]" << std::endl;
        m_controller.sendRideCommand(floor);
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));

        auto status = m_controller.getElevatorStatus();
        std::cout << "控制器反馈 callFloor = " << status.callFloor
            << "（目标 = " << floor << "）" << std::endl;
        if (status.callFloor == floor) {
            std::cout << "[乘梯指令确认成功] 目标楼层：" << status.callFloor << std::endl;
            return true;
        }
    }
    std::cerr << "[错误] 多次发送乘梯指令失败，目标楼层未被控制器确认！！！" << std::endl;
    return false;
}

// 发开主门指令,等 mainDoorOpen 确认后返回 true;超过重试次数返回 false
bool RobotElevatorClient::requestOpenMainDoorWithRetry(int retryLimit, int intervalMs) {
    for (int i = 0; i < retryLimit; ++i) {
        std::cout << "[第 " << (i + 1) << " 次发送开门指令]" << std::endl;
        m_controller.requestOpenMainDoor();
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));

        auto status = m_controller.getElevatorStatus();
        if (status.mainDoorOpen){
            std::cerr << "电梯主门已完全打开" << std::endl;
            return true;
        }
    }
    std::cerr << "[错误] 多次开门指令失败！！！" << std::endl;
    return false;
}

// 发关门指令,等 mainDoorOpen 为 false 确认后返回 true;超过重试次数返回 false
bool RobotElevatorClient::requestCloseDoorWithRetry(int retryLimit, int intervalMs) {
    for (int i = 0; i < retryLimit; ++i) {
        std::cout << "[第 " << (i + 1) << " 次发送关门指令]" << std::endl;
        m_controller.requestCloseDoor();
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));

        auto status = m_controller.getElevatorStatus();
        if (!status.mainDoorOpen ) {
            std::cerr << "电梯主门已完全关闭" << std::endl;
            return true;
        } 
    }
    std::cerr << "[错误] 多次关门指令失败" << std::endl;
    return false;
}

// =========================== 等待逻辑 ===============================

// 连接电梯控制器并启动通信保持线程,等电梯上线(isOnline)且激活(isActive),超时返回 false
bool RobotElevatorClient::waitElevatorOnlineAndActive(int timeout_sec) {
    if (!m_controller.connect()) {
        throw std::runtime_error("无法连接电梯控制器");
    }
    m_controller.startCommFlagThread();

    auto start = std::chrono::steady_clock::now();

    while (true) {
        auto status = m_controller.getElevatorStatus();
        if (status.isOnline && status.isActive) {
            std::cout << "电梯已上线并激活" << std::endl;
            return true;
        }

        std::cout << "[等待电梯上线并激活...]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeout_sec)) {
            std::cerr << "[超时] 等待电梯上线激活超时" << std::endl;
            return false;
        }
    }
}


// 轮询 currentFloor 直到电梯到达指定层,超时返回 false
bool RobotElevatorClient::waitElevatorArrives(const std::string& floor, int timeout_sec) {
    auto start = std::chrono::steady_clock::now();

    while (true) {
        auto status = m_controller.getElevatorStatus();
        if (status.currentFloor == floor) {
            std::cout << "电梯已到达" << std::endl;
            return true;
        }

        std::cout << "目标楼层: [" << floor << "]，轿厢当前楼层: [" << status.currentFloor << "]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeout_sec)) {
            std::cerr << "[超时] 等待电梯到达目标楼层超时" << std::endl;
            return false;
        }
    }
}

