// map_switch_client.hpp 的调用示例
//
// 编译:g++ -std=c++11 map_switch_client.cpp -o map_switch_client

#include "map_switch_client.hpp"

int main() {
    const char* server_addr = "192.168.2.100";
    const int   port        = 6050;
    const unsigned long target_map = 3;

    // 进电梯到达电梯内部点执行:预加载目标层地图
    if (!SendLoad(server_addr, port, target_map)) {
        std::cerr << "LOAD failed" << std::endl;
        return 1;
    }

    // (此处由上位机等待机器人乘电梯到达目标层并停稳)

    // 出电梯停稳:重定位,并传入初始位姿坐标 (x, y, yaw)
    //   仅当服务端 use_coord_transform=true 时这些坐标才会被使用(作为换算到目标图的源位姿);
    //   出电梯到达地图切换点执行
    float x = 1.50f;     // 机器人在源图坐标系下的 X
    float y = -0.80f;    // Y
    float yaw = 1.57f;   // 航向角(弧度)
    if (!SendReloc(server_addr, port, target_map, x, y, yaw)) {
        std::cerr << "RELOC failed" << std::endl;
        return 1;
    }

    std::cout << "map switch done." << std::endl;
    return 0;
}
