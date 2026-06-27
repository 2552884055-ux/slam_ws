// TCP方式
#include "robot_elevator_client.hpp"

// MODBUS RTU 构造函数
RobotElevatorClient::RobotElevatorClient(const std::string& serial_port, int baudrate, int slave_id)
    : m_controller(serial_port, baudrate, slave_id)  
{
    if (!m_controller.connect()) {
        throw std::runtime_error("无法连接到电梯控制器 (RTU)");
    }
}

// 转换数字转字符串函数
std::string formatFloor(int floor) {
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << floor;
    return ss.str();
}

// // ==========================使用的函数============================== //
// //召唤电梯并开门
// bool RobotElevatorClient::callElevatorAndOpenDoor(int FromFloor) {
//     std::string FromFloorStr = formatFloor(FromFloor); // 将int转为str

//     if (!waitElevatorOnlineAndActive()) {
//         std::cerr << "[失败] 电梯激活超时" << std::endl;
//         return false;
//     }

//     if (!sendRideCommandWithRetry(FromFloorStr)) {
//         std::cerr << "[失败] 多次召梯后电梯仍未到达目标楼层：" << FromFloorStr << std::endl;
//         return false;
//     }

//     std::cout << "尝试开门..." << std::endl;
//     if (!requestOpenMainDoorWithRetry()) {
//         std::cerr << "[失败] 开门失败" << std::endl;
//         return false;
//     }

//     return true;
// }
// //乘梯到目标楼层并开门
// bool RobotElevatorClient::rideToTargetFloorAndOpenDoor(int ToFloor) {
//     std::string ToFloorStr = formatFloor(ToFloor);// 将int转为str

//     std::cout << "关门..." << std::endl;
//     if (!requestCloseDoorWithRetry()) {
//         std::cerr << "[失败] 关门失败" << std::endl;
//         return false;
//     }

//     if (!sendRideCommandWithRetry(ToFloorStr)) {
//         std::cerr << "[失败] 多次召梯后电梯仍未到达目标楼层：" << ToFloorStr << std::endl;
//         return false;
//     }

//     std::cout << "尝试开门..." << std::endl;
//     if (!requestOpenMainDoorWithRetry()) {
//         std::cerr << "[失败] 开门失败" << std::endl;
//         return false;
//     }

//     return true;
// }
// // 关门并切换地图 
// bool RobotElevatorClient::closeDoorAndSwitchMap(const req_frame& request, const char* server_addr, int map_switch_PORT) {
//     std::cout << "[电梯任务完成阶段] 开始关门..." << std::endl;

//     if (!requestCloseDoorWithRetry()) {
//         throw std::runtime_error("关门失败");
//         return false;
//     }

//     std::cout << "[乘梯完成]" << std::endl;

//     m_controller.stopCommFlagThread();  // 关闭通信标志线程
//     m_controller.disconnect();          // 断开连接

//     std::cout << "[开始切换地图]" << std::endl;
//     // 服务端地址和端口（地图切换的地址、端口）
//     replay_frame reply = SendMapSwitchRequest(request, server_addr, map_switch_PORT);
//     if (reply.seq == request.seq && reply.result) {
//         std::cout << "[地图切换成功]" << std::endl;
//         return true;
//     }else {
//         std::cout << "[地图切换失败]" << std::endl;
//         return false;
//     }
// }

// // 发送乘梯指令直到轿厢到达目标楼层
// bool RobotElevatorClient::sendRideCommandWithRetry(const std::string& floor, int retryLimit, int intervalSec) {
//     const int pollIntervalMs = 1000;  // 每次轮询间隔 1 秒
//     const int commandIntervalMs = intervalSec * 1000; // 每次发指令间隔（默认10秒）
//     int elapsedMs = 0;

//     for (int attempt = 0; attempt < retryLimit; ++attempt) {
//         std::cout << "[第 " << (attempt + 1) << " 次发送乘梯指令：前往 " << floor << "]" << std::endl;
//         m_controller.sendRideCommand(floor);

//         elapsedMs = 0;
//         while (elapsedMs < commandIntervalMs) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
//             elapsedMs += pollIntervalMs;

//             auto status = m_controller.getElevatorStatus();
//             std::cout << "当前楼层 = " << status.currentFloor << "（目标 = " << floor << "）" << std::endl;

//             if (status.currentFloor == floor) {
//                 std::cout << "[成功] 电梯已到达目标楼层：" << status.currentFloor << std::endl;
//                 return true;
//             }
//         }
//     }
//     return false;
// }

// // 开门指令重试
// bool RobotElevatorClient::requestOpenMainDoorWithRetry(int retryLimit, int intervalMs) {
//     for (int i = 0; i < retryLimit; ++i) {
//         std::cout << "[第 " << (i + 1) << " 次发送开门指令]" << std::endl;
//         m_controller.requestOpenMainDoor();
//         std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));

//         auto status = m_controller.getElevatorStatus();
//         if (status.mainDoorOpen){
//             std::cerr << "电梯主门已完全打开" << std::endl;
//             return true;
//         }
//     }
//     std::cerr << "[错误] 多次开门指令失败！！！" << std::endl;
//     return false;
// }
// // 重试关门指令
// bool RobotElevatorClient::requestCloseDoorWithRetry(int retryLimit, int intervalMs) {
//     for (int i = 0; i < retryLimit; ++i) {
//         std::cout << "[第 " << (i + 1) << " 次发送关门指令]" << std::endl;
//         m_controller.requestCloseDoor();
//         std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));

//         auto status = m_controller.getElevatorStatus();
//         if (!status.mainDoorOpen) {
//             std::cerr << "电梯主门已完全关闭" << std::endl;
//             return true;
//         } 
//     }
//     std::cerr << "[错误] 多次关门指令失败" << std::endl;
//     return false;
// }

// // =========================== 等待逻辑 ===============================
// // 等待电梯上线并激活
// bool RobotElevatorClient::waitElevatorOnlineAndActive(int timeout_sec) {
//     m_controller.startCommFlagThread();

//     auto start = std::chrono::steady_clock::now();

//     while (true) {
//         auto status = m_controller.getElevatorStatus();
//         if (status.isOnline && status.isActive ) {
//             std::cout << "电梯已上线并激活" << std::endl;
//             return true;
//         }

//         std::cout << "[等待电梯上线并激活...原因: ]" ;
//         if (!status.isOnline)  std::cout << "未在线 ";
//         if (!status.isActive)  std::cout << "未激活 ";
//         if (!status.isNormal)  std::cout << "状态异常 ";
//         if (!status.isRuning)  std::cout << "未运行 "<< std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//         if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeout_sec)) {
//             std::cerr << "[超时] 等待电梯上线激活超时" << std::endl;
//             return false;
//         }
//     }
// }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ==========================使用的函数============================== //
bool RobotElevatorClient::callElevatorAndOpenDoor(int FromFloor) {
    std::string FromFloorStr = formatFloor(FromFloor); // 将int转为str

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

bool RobotElevatorClient::rideToTargetFloorAndOpenDoor(int ToFloor) {
    std::string TromFloorStr = formatFloor(ToFloor);// 将int转为str

    std::cout << "关门..." << std::endl;
    if (!requestCloseDoorWithRetry()) {
        std::cerr << "[失败] 关门失败" << std::endl;
        return false;
    }

    if (!sendRideCommandWithRetry(TromFloorStr)) {
        std::cerr << "[失败] 楼层指令发送失败" << std::endl;
        return false;
    }

    std::cout << "等待电梯到达目标楼层..." << std::endl;
    // waitElevatorArrives(TromFloorStr);
    const int retryIntervalSec = 20;
    auto lastRetryTime = std::chrono::steady_clock::now();

    // 等待电梯到达目标楼层，每20s重新发送乘梯指令
    while (true) {
        auto status = m_controller.getElevatorStatus();
        if (status.currentFloor == TromFloorStr) {
            std::cout << "电梯已到达" << std::endl;
            break;
        }

        std::cout << "目标楼层: [" << TromFloorStr << "]，轿厢当前楼层: [" << status.currentFloor << "]" << std::endl;

        // 当前时间
        auto now = std::chrono::steady_clock::now();

        // 超过重发周期则重新发楼层指令
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

bool RobotElevatorClient::closeDoorAndSwitchMap(const req_frame& request, const char* server_addr, int map_switch_PORT) {
    std::cout << "[电梯任务完成阶段] 开始关门..." << std::endl;

    if (!requestCloseDoorWithRetry()) {
        throw std::runtime_error("关门失败");
        return false;
    }

    std::cout << "[乘梯完成]" << std::endl;

    m_controller.stopCommFlagThread();  // 关闭通信标志线程
    m_controller.disconnect();          // 断开连接

    std::cout << "[开始切换地图]" << std::endl;
    // 服务端地址和端口（地图切换的地址、端口）
    replay_frame reply = SendMapSwitchRequest(request, server_addr, map_switch_PORT);
    if (reply.seq == request.seq && reply.result) {
        std::cout << "[地图切换成功]" << std::endl;
        return true;
    }else {
        std::cout << "[地图切换失败]" << std::endl;
        return false;
    }
}


// =========================== 重试逻辑 ===============================

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
