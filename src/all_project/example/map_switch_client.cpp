// map_switch_client.hpp 调用示例 —— 一次完整的地图切换
//
// 模拟两阶段切换流程:
//   1) 进电梯:发 LOAD(非阻塞),然后延时模拟乘梯时间(此时地图在后台加载)
//   2) 出电梯:发 RELOC(阻塞,等重定位完成)
//
// 真实工程中延时替换为"机器人实际到位"的事件触发。

#include "map_switch_client.hpp"
#include <atomic>
#include <unistd.h>

int main()
{
    const char*         server_addr = "192.168.2.100";   // 机器人(地图切换服务)IP
    const int           port        = 6050;              // 端口
    const unsigned long target_map  = 2;                 // 目标地图ID(对应 floors.yaml 的 id)

    // ===== 1) 进电梯:非阻塞发 LOAD,不等结果,乘梯期间后台加载地图 =====
    std::atomic<bool> load_ok{false};
    SendLoad(server_addr, port, target_map,
        [&load_ok](bool ok) {
            load_ok = ok;   // 后台线程:地图加载完成时置位
        });

    // 模拟乘梯时间(真实工程替换为"等机器人出电梯并停稳"的事件)
    sleep(15);

    // ===== 2) 出电梯:等 LOAD 成功后发 RELOC,阻塞直到重定位完成 =====
    // 若乘梯够长地图已加载完(load_ok=true),此处不等直接发;
    // 若乘梯极短地图还没好,最多再等 60s。
    for (int i = 0; i < 600 && !load_ok; ++i) {
        usleep(100 * 1000);
    }

    if (!load_ok) {
        std::cerr << "[失败] 地图加载超时" << std::endl;
        return 1;
    }

    // x/y/yaw:机器人在源图坐标系下的位姿;
    //   服务端 use_coord_transform=false 时不使用(由 floors.yaml 各层锚点决定),
    //   服务端 use_coord_transform=true  时换算到目标图作为重定位初值。
    float x   = 0.0f;
    float y   = 0.0f;
    float yaw = 0.0f;

    if (!SendReloc(server_addr, port, target_map, x, y, yaw)) {
        std::cerr << "[失败] 重定位失败" << std::endl;
        return 1;
    }

    std::cout << "[完成] 地图切换成功" << std::endl;
    return 0;
}
