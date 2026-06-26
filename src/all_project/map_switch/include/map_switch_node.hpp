// map_switch_node.hpp —— 两阶段楼层切换编排(MapSwitchNode)
//
// 职责:加载配置;接收 TCP 指令(经 TcpServer)并按 cmd 分发;
//       LOAD(进电梯预加载)/ RELOC(出电梯重定位)两阶段编排;
//       通过 NodeLauncher 起停各定位节点。收发与进程管理已下放到 TcpServer / NodeLauncher。

#pragma once

#include "protocol.hpp"
#include "tcp_server.hpp"
#include "node_launcher.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>

class MapSwitchNode
{
public:
    MapSwitchNode();
    ~MapSwitchNode();

    /** 加载配置、绑定监听、起 accept、开机起栈。成功返回 true。 */
    bool init();
    /** 停止服务并杀掉所有由本节点拉起的地图节点。 */
    void stop();

private:
    // ===== TCP 请求入口(TcpServer 回调) =====
    void onRequest(const req_frame& request, int client_fd);   ///< busy 互斥 + 按 cmd 分发

    // ===== 两阶段核心 =====
    bool doLoad(unsigned long target_map_id);                  ///< 停旧 → 起预加载节点 → 等就绪
    bool doReloc(float x, float y, float yaw);                 ///< 起 laserMapping → 等就绪 → 发初值(重试)→ 等重定位
    void loadMap(unsigned long target_map_id, unsigned long req_seq, int client_fd);     ///< LOAD 异步入口
    void relocalize(unsigned long target_map_id, unsigned long req_seq,
                    float x, float y, float yaw, int client_fd);                          ///< RELOC 异步入口

    // ===== 辅助 =====
    std::vector<NodeSpec> buildLoadSpecs(const MapInfo& m) const;   ///< LOAD 阶段 5 个节点
    NodeSpec buildLaserSpec() const;                               ///< laserMapping 节点
    void publishInitialPoseMsg(float x, float y, float yaw);       ///< 直接发 /initialpose
    req_frame convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id);
    void stopCurrentNodes();                                       ///< 杀掉当前所有 load 节点与 laserMapping

    // ===== 组件 =====
    ros::NodeHandle nh;
    ros::Publisher  initialpose_pub_;
    TcpServer       server_;
    NodeLauncher    launcher_;

    // ===== 状态/进程 =====
    std::mutex          pidMutex_;
    std::atomic<bool>   busy_{false};       ///< 切换进行中:为 true 时拒绝新的 LOAD/RELOC
    std::vector<pid_t>  loadPids_;          ///< 当前 LOAD 阶段 5 个节点的 PID
    pid_t               laserPid_;          ///< 当前 laserMapping 的 PID
    unsigned long       current_map_id_;    ///< 当前已加载/激活的地图ID
    unsigned long       prev_map_id_;       ///< 本次切换前的源地图ID(供坐标变换 src 用)
    std::map<unsigned long, MapInfo> maps_; ///< 地图信息表

    // ===== 配置 =====
    std::string server_addr_;
    int         map_switch_PORT_;
    std::string pkg_pcd_dir_;               ///< fast_lio_global 包内 PCD 目录绝对路径
    std::string python_path_;               ///< python 节点的 PYTHONPATH
    bool        use_coord_transform_;       ///< RELOC 初值来源:true=坐标变换 / false=固定锚点
    int         initial_map_id_;            ///< 开机起栈的初始地图ID(<=0 用列表第一张)
    float       initial_x_, initial_y_, initial_yaw_;   ///< 开机起栈初始位姿
    double      load_ready_timeout_, laser_alive_timeout_, reloc_total_timeout_;
};
