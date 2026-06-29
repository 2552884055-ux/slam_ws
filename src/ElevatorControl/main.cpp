// 电梯控制 + 两阶段地图切换 —— 调用示例 (TCP / RTU 二合一)
//
// 乘梯 + 换层完整流程：
//   步骤0  读取并打印全部寄存器(确认与电梯通信正常)
//   步骤1  导航到候梯点 → callElevatorAndOpenDoor       召梯、等到达、开门，机器狗进梯
//   步骤2  机器狗进梯后 → closeDoorLoadMapRideAndOpenDoor
//              关门 → 非阻塞发 LOAD(目标层地图在乘梯期间后台加载)
//              → 发乘梯指令 → 等到达目标层 → 开门，机器狗出梯
//   步骤3  机器狗出梯后 → closeDoorAndWaitMapThenReloc
//              关门 → 等地图加载完成(若乘梯够长则无需等待)
//              → 发 RELOC(重定位) → 切换完成，可继续巡检
//
// TCP / RTU 切换：只需改下面 client 的构造方式(两者后续调用完全相同)。

#include "robot_elevator_client.hpp"

int main() {
    try {
        // ===== 电梯连接方式：二选一 =====
        // 【TCP】电梯控制器 IP、Modbus TCP 端口、从机ID
        RobotElevatorClient client("127.0.0.1", 8000, 1);
        // 【RTU】串口设备、波特率、校验位、数据位、停止位、从机ID
        // RobotElevatorClient client("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);

        const char* server_addr     = "127.0.0.1";       // 地图切换服务(all_project map_switch 节点)IP；本机联调对接 map_switch_sim
        const int   map_switch_PORT = 6050;               // 地图切换服务端口
        const int   from_floor      = 1;                  // 出发楼层
        const int   to_floor        = 2;                  // 目标楼层
        const unsigned long target_map = map2;            // 目标地图ID(对应 floors.yaml 的 id)

        // 步骤0:读取并打印全部寄存器(调试观察:确认与电梯通信正常、各寄存器取值)
        client.dumpAllRegisters();

        // ===== 单元测试(按需注释/取消注释,单独验证某个动作) =====
        // client.testCallElevator(2);   // 召梯到 2 层
        // client.testOpenMainDoor();    // 开主门
        // client.testCloseDoor();       // 关门

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
        float reloc_y   = 1.0f;
        float reloc_yaw = 2.0f;
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
