// // RTU方式
// #include "robot_elevator_client.hpp" 

// // 乘梯步骤：
// // 1.导航到达候梯点，发送第一次召梯指令来接机器狗
// // 2.电梯到达后，发送开门指令
// // 3.机器狗完全进入电梯后，发送关门指令
// // 4.关门后，再次发送召梯指令，前往目标楼层
// // 5.等待电梯运行，电梯到达目标楼层后发送开门指令
// // 6.开门后走出电梯，完全走出电梯后，发送电梯关门指令
// // 7.执行地图切换指令，等待切换完成后继续巡检任务

// // int main() {
// //     try {
// //         RobotElevatorClient client("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

// //         client.callElevatorAndOpenDoor(1);          // 设置机器狗出发楼层，成功开门返回Ture，然后进梯
    
// //         // 延时10秒模拟机器狗进电梯动作
// //         std::this_thread::sleep_for(std::chrono::seconds(5));
    
// //         client.closeDoorLoadMapRideAndOpenDoor(9);     // 设置机器狗目标楼层，成功开门返回Ture，然后出梯
    
// //         // 延时10秒模拟机器狗进电梯动作
// //         std::this_thread::sleep_for(std::chrono::seconds(5));
    
// //         req_frame request;
// //         request.frame_type = map1; 
// //         request.seq = 1;  // 序列号
// //         request.x = 0;
// //         request.y = 0;
// //         request.yaw = 0;
// //         client.closeDoorAndWaitMapThenReloc(request, "192.168.110.206", 7001);  // 乘梯完成并切换点图
// //     } catch (const std::exception& e) {
// //         std::cerr << "[异常] " << e.what() << std::endl;
// //     }

// //     return 0;
// // }

// //  test
// int main() {
//     try {

//         // 初始化控制器（根据实际串口和参数修改）
//         ElevatorController elevator("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

//         // 测试读取输入寄存器 0~12 (共13个寄存器)
//         uint16_t data1[13] = {0};
//         uint16_t data2[13] = {0};

//         elevator.readInputRegisters(0, 13, data1);
//         std::cout << "读取输入寄存器 30001 ~ 30013：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (30001 + i) << "] = 0x" 
//                         << std::hex << std::setw(4) << std::setfill('0') << data1[i] 
//                         << " (" << std::dec << data1[i] << ")" << std::endl;
//         }

//         // 测试读取保持寄存器 0~12 (共13个寄存器)
//         elevator.readHoldingRegisters(0, 13, data2);
//         std::cout << "\n读取保持寄存器 40001 ~ 40013：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (40001 + i) << "] = 0x" 
//                         << std::hex << std::setw(4) << std::setfill('0') << data2[i] 
//                         << " (" << std::dec << data2[i] << ")" << std::endl;
//         }

//         // 获取并显示电梯状态
//         auto status = elevator.getElevatorStatus();
//         std::cout << "\n--- 电梯状态 ---" << std::endl;
//         std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
//         std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
//         std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
//         std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
//         std::cout << "当前楼层: " << status.currentFloor << std::endl;
//         std::cout << "当前召梯楼层: " << status.callFloor << std::endl;


//         // //可选：打开主门
//         // elevator.requestOpenMainDoor();

//         // //可选：发送乘梯命令至“ 3”楼
//         elevator.sendRideCommand("1");

//         // // 可选：关门
//         // elevator.requestCloseDoor();

//         std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

//         elevator.readInputRegisters(0, 13, data1);
//         std::cout << "读取输入寄存器 30001 ~ 30013：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (30001 + i) << "] = 0x" 
//                         << std::hex << std::setw(4) << std::setfill('0') << data1[i] 
//                         << " (" << std::dec << data1[i] << ")" << std::endl;
//         }
//         elevator.readHoldingRegisters(0, 13, data2);
//         std::cout << "\n读取保持寄存器 40001 ~ 40013：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (40001 + i) << "] = 0x" 
//                         << std::hex << std::setw(4) << std::setfill('0') << data2[i] 
//                         << " (" << std::dec << data2[i] << ")" << std::endl;
//         }
//         // 获取并显示电梯状态
//         status = elevator.getElevatorStatus();
//         std::cout << "\n--- 电梯状态 ---" << std::endl;
//         std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
//         std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
//         std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
//         std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
//         std::cout << "当前楼层: " << status.currentFloor << std::endl;
//         std::cout << "当前召梯楼层: " << status.callFloor << std::endl;

//     } catch (const std::exception& ex) {
//         std::cerr << "发生异常: " << ex.what() << std::endl;
//         return 1;
//     }

//     return 0;
// }

// // 每隔一秒读取一次电梯的两个寄存器状态，测试连接状态
// // int main() {
// //     try {
// //         // 初始化控制器（根据实际串口与通信参数修改）
// //         ElevatorController elevator("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

// //         std::cout << "电梯 Modbus 监控程序启动..." << std::endl;

// //         // 持续检测循环
// //         while (true) {
// //             uint16_t data1[13] = {0};
// //             uint16_t data2[13] = {0};

// //             // === 读取输入寄存器 ===
// //             if (elevator.readInputRegisters(0, 13, data1)) {
// //                 std::cout << "\n[输入寄存器 30000~30012]" << std::endl;
// //                 for (int i = 0; i < 13; ++i) {
// //                     std::cout << "寄存器 [" << std::setw(5) << (30000 + i)
// //                               << "] = 0x" << std::hex << std::setw(4) << std::setfill('0') << data1[i]
// //                               << " (" << std::dec << data1[i] << ")" << std::endl;
// //                 }
// //             } else {
// //                 std::cerr << "读取输入寄存器失败！" << std::endl;
// //             }

// //             // === 读取保持寄存器 ===
// //             if (elevator.readHoldingRegisters(0, 13, data2)) {
// //                 std::cout << "\n[保持寄存器 40000~40012]" << std::endl;
// //                 for (int i = 0; i < 13; ++i) {
// //                     std::cout << "寄存器 [" << std::setw(5) << (40000 + i)
// //                               << "] = 0x" << std::hex << std::setw(4) << std::setfill('0') << data2[i]
// //                               << " (" << std::dec << data2[i] << ")" << std::endl;
// //                 }
// //             } else {
// //                 std::cerr << "读取保持寄存器失败！" << std::endl;
// //             }

// //             // === 获取电梯状态 ===
// //             auto status = elevator.getElevatorStatus();
// //             std::cout << "\n--- 电梯状态 ---" << std::endl;
// //             std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
// //             std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
// //             std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
// //             std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
// //             std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
// //             std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
// //             std::cout << "当前楼层: " << status.currentFloor << std::endl;
// //             std::cout << "当前召梯楼层: " << status.callFloor << std::endl;

// //             // === 每隔 1 秒刷新一次 ===
// //             std::this_thread::sleep_for(std::chrono::seconds(1));
// //         }

// //     } catch (const std::exception& ex) {
// //         std::cerr << "发生异常: " << ex.what() << std::endl;
// //         return 1;
// //     }

// //     return 0;
// // }


// Modbus TCP 方式电梯控制 + 两阶段地图切换
//
// 乘梯 + 换层完整流程：
//   步骤1  导航到候梯点 → callElevatorAndOpenDoor    召梯、等到达、开门，机器狗进梯
//   步骤2  机器狗进梯后 → closeDoorLoadMapRideAndOpenDoor
//              关门 → 非阻塞发 LOAD(目标层地图在乘梯期间后台加载)
//              → 发乘梯指令 → 等到达目标层 → 开门，机器狗出梯
//   步骤3  机器狗出梯后 → closeDoorAndWaitMapThenReloc
//              关门 → 等地图加载完成(若乘梯够长则无需等待)
//              → 发 RELOC(重定位) → 切换完成，可继续巡检

#include "robot_elevator_client.hpp"

// int main() {
//     try {
//         ElevatorController elevator("192.168.110.221", 8000, 1);
//         // 建立 TCP 连接
//         if (!elevator.connect()) {
//             throw std::runtime_error("无法连接到电梯控制器 (Modbus TCP)");
//         }
//         std::cout << "[成功] 已连接 Modbus TCP 电梯控制器" << std::endl;

//         // ===================== 读取输入寄存器 =====================
//         uint16_t data1[13];
//         elevator.readInputRegisters(0, 13, data1);
//         std::cout << "\n读取输入寄存器 30000 ~ 30012：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (30000 + i) << "] = 0x"
//                       << std::hex << std::setw(4) << std::setfill('0') << data1[i]
//                       << " (" << std::dec << data1[i] << ")" << std::endl;
//         }

//         // ===================== 读取保持寄存器 =====================
//         uint16_t data2[13];
//         elevator.readHoldingRegisters(0, 13, data2);
//         std::cout << "\n读取保持寄存器 40000 ~ 40012：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (40000 + i) << "] = 0x"
//                       << std::hex << std::setw(4) << std::setfill('0') << data2[i]
//                       << " (" << std::dec << data2[i] << ")" << std::endl;
//         }

//         // ===================== 获取并显示电梯状态 =====================
//         auto status = elevator.getElevatorStatus();
//         std::cout << "\n--- 电梯状态 ---" << std::endl;
//         std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
//         std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
//         std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
//         std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
//         std::cout << "当前楼层: " << status.currentFloor << std::endl;
//         std::cout << "当前召梯楼层: " << status.callFloor << std::endl;

//         // ===================== 控制命令测试 =====================
//         // elevator.requestOpenMainDoor();      // 打开主门
//         elevator.sendRideCommand("005");     // 乘梯至“5”楼
//         // elevator.requestCloseDoor();         // 关门

//         std::this_thread::sleep_for(std::chrono::milliseconds(10000));
//         std::cout << "\n\n-----召梯后寄存器数据-----\n "<< std::endl;
//         // ===================== 读取输入寄存器 =====================
//         elevator.readInputRegisters(0, 13, data1);
//         std::cout << "\n读取输入寄存器 30000 ~ 30012：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (30000 + i) << "] = 0x"
//                       << std::hex << std::setw(4) << std::setfill('0') << data1[i]
//                       << " (" << std::dec << data1[i] << ")" << std::endl;
//         }
//         // ===================== 读取保持寄存器 =====================
//         elevator.readHoldingRegisters(0, 13, data2);
//         std::cout << "\n读取保持寄存器 40000 ~ 40012：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (40000 + i) << "] = 0x"
//                       << std::hex << std::setw(4) << std::setfill('0') << data2[i]
//                       << " (" << std::dec << data2[i] << ")" << std::endl;
//         }
//         // ===================== 获取并显示电梯状态 =====================
//         status = elevator.getElevatorStatus();
//         std::cout << "\n--- 电梯状态 ---" << std::endl;
//         std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
//         std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
//         std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
//         std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
//         std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
//         std::cout << "当前楼层: " << status.currentFloor << std::endl;
//         std::cout << "当前召梯楼层: " << status.callFloor << std::endl;

//         // 断开 TCP 连接
//         elevator.disconnect();
//         std::cout << "\n[完成] Modbus TCP 电梯测试结束" << std::endl;

//     } catch (const std::exception& ex) {
//         std::cerr << "发生异常: " << ex.what() << std::endl;
//         return 1;
//     }

//     return 0;
// }
// // 数据写入测试
// int main() {
//     try {
//         ElevatorController elevator("192.168.110.221", 8000, 1);
//         // 建立 TCP 连接
//         if (!elevator.connect()) {
//             throw std::runtime_error("无法连接到电梯控制器 (Modbus TCP)");
//         }
//         std::cout << "[成功] 已连接 Modbus TCP 电梯控制器" << std::endl;

//         // ===================== 控制命令测试 =====================
//         // elevator.requestOpenMainDoor();      // 打开主门
//         // elevator.requestOpenViceDoor();      // 打开副门
//         // elevator.sendRideCommand("005");     // 乘梯至“5”楼
//         // elevator.requestCloseDoor();         // 关门
//         while(true){
//             elevator.startCommFlagThread();  // 启动通信保持线程
//         }
//         // 断开 TCP 连接
//         elevator.disconnect();
//         std::cout << "\n[完成] Modbus TCP 电梯测试结束" << std::endl;
//     } catch (const std::exception& ex) {
//         std::cerr << "发生异常: " << ex.what() << std::endl;
//         return 1;
//     }
//     return 0;
// }

// // 每隔一秒读取一次电梯的两个寄存器状态，测试连接状态
// int main() {
//     try {
//         ElevatorController elevator("192.168.110.221", 8000, 1);

//         if (!elevator.connect()) {
//             throw std::runtime_error("无法连接到电梯控制器 (Modbus TCP)");
//         }
//         std::cout << "[成功] 已连接到 Modbus TCP 电梯控制器\n" << std::endl;

//         while (true) {
//             try {
//                 // ----------- 读取输入寄存器 -----------
//                 uint16_t inputRegs[13];
//                 elevator.readInputRegisters(0, 13, inputRegs);
//                 std::cout << "\n[输入寄存器] 30000 ~ 30012：" << std::endl;

//                 for (int i = 0; i < 13; ++i) {
//                     std::string bin = std::bitset<16>(inputRegs[i]).to_string();
//                     bin.insert(4,  " ");
//                     bin.insert(9,  " ");
//                     bin.insert(14, " ");
//                     std::cout << "寄存器 [" << std::setw(5) << (30000 + i) << "] = 0x"
//                             << std::hex << std::setw(4) << std::setfill('0') << inputRegs[i]
//                             << " (" << bin << ")"<< std::dec << std::setfill(' ')<< std::endl;
//                 }

//                 // ----------- 读取保持寄存器 -----------
//                 uint16_t holdingRegs[13];
//                 elevator.readHoldingRegisters(0, 13, holdingRegs);
//                 std::cout << "\n[保持寄存器] 40000 ~ 40012：" << std::endl;
//                 for (int i = 0; i < 13; ++i) {
//                     std::string bin = std::bitset<16>(holdingRegs[i]).to_string();
//                     bin.insert(4,  " ");
//                     bin.insert(9,  " ");
//                     bin.insert(14, " ");
//                     std::cout << "寄存器 [" << std::setw(5) << (40000 + i) << "] = 0x"
//                             << std::hex << std::setw(4) << std::setfill('0') << holdingRegs[i]
//                             << " (" << bin << ")"<< std::dec << std::setfill(' ')<< std::endl;
//                 }

//                 // ----------- 获取电梯状态 -----------
//                 auto status = elevator.getElevatorStatus();
//                 std::cout << "\n--- 电梯状态 ---" << std::endl;
//                 std::cout << "是否投用: " << (status.isActive ? "是" : "否") << std::endl;
//                 std::cout << "是否在线: " << (status.isOnline ? "是" : "否") << std::endl;
//                 std::cout << "主门是否打开: " << (status.mainDoorOpen ? "是" : "否") << std::endl;
//                 std::cout << "副门是否打开: " << (status.viceDoorOpen ? "是" : "否") << std::endl;
//                 std::cout << "电梯是否下行: " << (status.isDownward ? "是" : "否") << std::endl;
//                 std::cout << "电梯是否上行: " << (status.isUpward ? "是" : "否") << std::endl;
//                 std::cout << "当前楼层: " << status.currentFloor << std::endl;
//                 std::cout << "当前召梯楼层: " << status.callFloor << std::endl;

//             } catch (const std::exception& innerEx) {
//                 std::cerr << "[错误] 读取失败: " << innerEx.what() << std::endl;
//             }

//             // ===================== 每秒轮询一次 =====================
//             std::this_thread::sleep_for(std::chrono::seconds(1));
//         }

//     } catch (const std::exception& ex) {
//         std::cerr << "发生异常: " << ex.what() << std::endl;
//         return 1;
//     }

//     return 0;
// }


int main() {
    try {
        // TCP 参数:电梯控制器 IP、Modbus TCP 端口、从机ID
        RobotElevatorClient client("127.0.0.1", 8000, 1);   // 本机联调:对接 ElevatorSimulator (./elevator_sim --tcp 127.0.0.1 8000)

        const char* server_addr     = "127.0.0.1";       // 地图切换服务(all_project map_switch 节点)IP；本机联调对接 map_switch_sim
        const int   map_switch_PORT = 6050;               // 地图切换服务端口
        const int   from_floor      = 1;                  // 出发楼层
        const int   to_floor        = 2;                  // 目标楼层
        const unsigned long target_map = map2;            // 目标地图ID(对应 floors.yaml 的 id)

        // 步骤0:读取并打印全部寄存器(调试观察:确认与电梯通信正常、各寄存器取值)
        client.dumpAllRegisters();

        // 步骤1:召梯到出发层并开门,等机器狗完全进入电梯
        if (!client.callElevatorAndOpenDoor(from_floor)) {
            std::cerr << "[失败] 召梯或开门失败" << std::endl;
            return 1;
        }
        // 等机器狗完全进入电梯(实际由导航到位事件驱动,此处用延时占位)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 步骤2:关门 + 发乘梯指令 + 同步非阻塞预加载目标层地图(利用乘梯时间后台加载)
        if (!client.closeDoorLoadMapRideAndOpenDoor(to_floor, target_map, server_addr, map_switch_PORT)) {
            std::cerr << "[失败] 乘梯或到达目标层失败" << std::endl;
            return 1;
        }
        // 等机器狗完全走出电梯(实际由导航到位事件驱动,此处用延时占位)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 步骤3:关门 + 等地图加载完成 + 发 RELOC 重定位
        // x/y/yaw:机器人在源图坐标系下的位姿,始终需要传入;
        //   服务端 use_coord_transform=false 时不使用(改用 floors.yaml 各层锚点),
        //   服务端 use_coord_transform=true  时将其换算到目标图作为重定位初值。
        float reloc_x   = 0.0f;
        float reloc_y   = 0.0f;
        float reloc_yaw = 0.0f;
        if (!client.closeDoorAndWaitMapThenReloc(target_map, server_addr, map_switch_PORT,
                                          reloc_x, reloc_y, reloc_yaw)) {
            std::cerr << "[失败] 地图切换失败" << std::endl;
            return 1;
        }

        std::cout << "[完成] 乘梯 + 地图切换全部成功" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[异常] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

