// Modbus RTU 方式电梯控制 + 两阶段地图切换
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

int main() {
    try {
        // 串口参数:设备、波特率、校验位、数据位、停止位、从机ID
        RobotElevatorClient client("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

        const char* server_addr     = "192.168.2.100";   // 地图切换服务(all_project map_switch 节点)IP
        const int   map_switch_PORT = 6050;               // 地图切换服务端口
        const int   from_floor      = 1;                  // 出发楼层
        const int   to_floor        = 2;                  // 目标楼层
        const unsigned long target_map = map2;            // 目标地图ID(对应 floors.yaml 的 id)

        // 步骤1:召梯到出发层并开门,等机器狗完全进入电梯
        if (!client.callElevatorAndOpenDoor(from_floor)) {
            std::cerr << "[失败] 召梯或开门失败" << std::endl;
            return 1;
        }
        // 等机器狗完全进入电梯(实际由导航到位事件驱动,此处用延时占位)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 步骤2:关门 + 发乘梯指令 + 同步非阻塞预加载目标层地图(利用乘梯时间后台加载)
        if (!client.closeDoorLoadMapRideAndOpenDoor(
            to_floor, target_map, server_addr, map_switch_PORT)) {
            std::cerr << "[失败] 乘梯或到达目标层失败" << std::endl;
            return 1;
        }
        // 等机器狗完全走出电梯(实际由导航到位事件驱动,此处用延时占位)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 步骤3:关门 + 等地图加载完成 + 发 RELOC 重定位
        // x/y/yaw:机器人在源图坐标系下的位姿,始终需要传入;
        //   服务端 use_coord_transform=false 时不使用该值(改用 floors.yaml 各层锚点),
        //   服务端 use_coord_transform=true  时将其换算到目标图作为重定位初值。
        float reloc_x   = 1.0f;
        float reloc_y   = 2.0f;
        float reloc_yaw = 3.0f;
        if (!client.closeDoorAndWaitMapThenReloc(
            target_map, server_addr, map_switch_PORT,               
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



// //  测试基础通信功能，寄存器读写功能
// int main() {
//     try {

//         // 初始化控制器（根据实际串口和参数修改）
//         ElevatorController elevator("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

//         uint16_t data1[13];
//         elevator.readInputRegisters(0, 13, data1);
//         std::cout << "读取输入寄存器 30001 ~ 30013：" << std::endl;
//         for (int i = 0; i < 13; ++i) {
//             std::cout << "寄存器 [" << std::setw(5) << (30001 + i) << "] = 0x" 
//                         << std::hex << std::setw(4) << std::setfill('0') << data1[i] 
//                         << " (" << std::dec << data1[i] << ")" << std::endl;
//         }

//         // 测试读取保持寄存器 0~12 (共13个寄存器)
//         uint16_t data2[13];
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
//         elevator.sendRideCommand("  3");

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
