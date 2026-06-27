// map_switch_client.hpp 的调用示例(巡检:按点位类型分发任务)
//
// 巡检主循环:到达一个点位 → 先判断点位类型 → 执行该类型对应的任务。
// 地图切换分摊到两类点位:
//   电梯内部点(PT_ELEVATOR_IN):非阻塞发 LOAD,预加载目标层地图;
//   重定位点  (PT_RELOC)       :地图加载完成(g_load_ok)则发 RELOC。
// 因为每到一个点都会重新分发,所以可随巡检反复切换,不是一次性。
//
// 编译:g++ -std=c++11 map_switch_client.cpp -o map_switch_client -pthread

#include "map_switch_client.hpp"
#include <atomic>
#include <unistd.h>

// LOAD 成功信号:由 SendLoad 的回调(后台线程)置位
std::atomic<bool> g_load_ok{false};

// 点位类型(按你的实际巡检点位类型扩展)
enum PointType {
    PT_INSPECT,       // 巡检点:执行巡检任务
    PT_CHARGE,        // 充电点:回充
    PT_ELEVATOR_IN,   // 电梯内部点:进电梯后发 LOAD(预加载目标层地图)
    PT_RELOC,         // 重定位点:出电梯后,地图加载完成则发 RELOC
};

// 到达一个点位 → 判类型 → 执行对应任务
void onArriveWaypoint(PointType type, unsigned long target_map,
                      const char* server_addr, int port)
{
    switch (type) {
        case PT_INSPECT:
            // doInspectTask();              // 你的巡检任务
            break;

        case PT_CHARGE:
            // goCharge();                   // 你的回充任务
            break;

        case PT_ELEVATOR_IN:
            // 进电梯:非阻塞发 LOAD,加载完成由回调置位 g_load_ok
            g_load_ok = false;
            SendLoad(server_addr, port, target_map,
                     [](bool ok) { if (ok) g_load_ok = true; });
            break;

        case PT_RELOC:
            // 出电梯重定位点:地图加载完成才发 RELOC(阻塞,返回是否成功)
            if (g_load_ok) {
                SendReloc(server_addr, port, target_map);
            } else {
                // 还没加载完(极少见,如乘梯过快):可在此等待 g_load_ok 或下个周期再来
            }
            break;
    }
}

int main()
{
    const char* server_addr = "192.168.2.100";   // 机器人(地图切换服务)IP
    const int   port        = 6050;               // 端口

    // ===== 巡检主循环 =====
    while (true) {
        // 从导航/巡检取"刚到达的点位"(类型 + 关联的目标层id),例如:
        //   Waypoint wp = nav.getArrivedWaypoint();
        //   if (wp.valid())
        //       onArriveWaypoint(wp.type, wp.target_map, server_addr, port);

        usleep(20 * 1000);   // 主循环节拍(20ms)
    }

    return 0;
}
