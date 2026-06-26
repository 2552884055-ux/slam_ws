/**
 * @file map_switch_node.hpp
 * @brief TCP地图切换节点(两阶段;节点由 C++ 直接 fork/execvp(rosrun) 启动,不再依赖 .sh / .launch)
 *
 * 两阶段切换(由上位机两条TCP指令触发):
 *   - LOAD  (cmd=1):进电梯——停旧节点、起目标层"预加载"节点(map_publishe/map_server/
 *                    global_localization/transform_fusion/tf_robot2map,不含 laserMapping),
 *                    等 global_localization 加载好地图并挂起等初值。期间不重定位。
 *   - RELOC (cmd=2):出电梯静止——启动 laserMapping(干净 IMU 初始化、odom≈I),
 *                    发布初始位姿触发一次重定位,等待完成后回执。
 *
 * 启动方式:每个节点由本进程 fork() 后在子进程 execvp("rosrun", pkg, type, args...) 启动,
 *           并以独立进程组运行;停图时按 PID(进程组)精确 kill,不再用 rosnode kill / 脚本。
 *           laserMapping 等 FAST-LIO 参数仍由 project.launch 预先加载到参数服务器。
 */

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <map>
#include <utility>
#include <cmath>
#include <errno.h>

/** TCP 请求命令字(req_frame.cmd) */
enum MapSwitchCmd {
    CMD_LOAD  = 1,   ///< 进电梯:预加载目标地图(不含 laserMapping,不重定位)
    CMD_RELOC = 2    ///< 出电梯:启动 laserMapping 并发布初始位姿触发重定位
};

/**
 * @struct req_frame
 * @brief TCP客户端请求数据帧(定长,二进制)
 * 注意:cmd 为首字段,上位机需按此布局发送。
 */
struct req_frame
{
    unsigned long cmd;          ///< 1=LOAD 2=RELOC
    unsigned long frame_type;   ///< 目标地图ID
    unsigned long seq;          ///< 请求序列号
    float x;                    ///< 初始位姿X(RELOC;电梯流程填0)
    float y;                    ///< 初始位姿Y(RELOC;电梯流程填0)
    float yaw;                  ///< 初始位姿yaw(弧度;RELOC;电梯流程填0)
};

/**
 * @struct replay_frame
 * @brief TCP服务端应答数据帧
 */
struct replay_frame
{
    bool result;                ///< true成功 / false失败
    unsigned long seq;          ///< 对应请求序列号
};

/**
 * @struct MapInfo
 * @brief 地图信息(由 config/floors.yaml 提供)
 */
struct MapInfo {
    unsigned long id;           ///< 地图唯一ID
    std::string pcd;            ///< 三维点云地图文件名(位于 fast_lio_global/PCD/)
    std::string gridmap;        ///< 二维栅格地图 yaml 文件名(位于 fast_lio_global/PCD/)
    float exit_x;               ///< RELOC 重定位锚点(出梯位姿,map系)X,默认0
    float exit_y;               ///< RELOC 重定位锚点 Y,默认0
    float exit_yaw;             ///< RELOC 重定位锚点 yaw(弧度),默认0
    float tx_to_map1;           ///< 相对map1的X平移量(convertBetweenMaps 可选用)
    float ty_to_map1;           ///< 相对map1的Y平移量
    float theta_to_map1;        ///< 相对map1的旋转角(弧度)
};

/**
 * @struct NodeSpec
 * @brief 一个 ROS 节点的启动描述(供 rosrun 启动)
 */
struct NodeSpec {
    std::string name;                                          ///< 节点名(__name:=)
    std::string pkg;                                           ///< rosrun 包名
    std::string type;                                          ///< rosrun 可执行/脚本名
    std::vector<std::string> args;                             ///< 透传给节点的位置参数/remap/私有参数
    // 注:固定环境变量(PYTHONPATH)在构造函数里进程级 setenv 一次,子进程继承,
    //     不在 fork 后设置(fork+多线程下 setenv 非 async-signal-safe,可能死锁)。
};

/**
 * @class TcpMapSwitchNode
 * @brief TCP地图切换核心类(两阶段,C++ 直接拉起/杀死节点)
 */
class TcpMapSwitchNode
{
public:
    TcpMapSwitchNode();
    ~TcpMapSwitchNode();

    bool init();
    void start();
    void stop();

private:
    // ===== ROS =====
    ros::NodeHandle nh;
    ros::Publisher initialpose_pub_;        ///< /initialpose 发布器
    bool isRunning_;

    // ===== 网络 =====
    int sockfd_;
    int listenfd_;
    int map_switch_PORT_;
    std::string server_addr_;

    std::thread tcpThread_;
    std::mutex mutex_;
    std::mutex pidMutex_;
    std::atomic<bool> busy_{false};         ///< 切换进行中标志:为 true 时拒绝新的 LOAD/RELOC 指令

    // ===== 地图/进程管理 =====
    std::vector<pid_t> loadPids_;           ///< 当前 LOAD 阶段 5 个节点的 PID
    pid_t laserPid_;                        ///< 当前 laserMapping 的 PID
    unsigned long current_map_id_;          ///< 当前已加载/激活的地图ID
    unsigned long prev_map_id_;             ///< 本次切换前的源地图ID(供坐标变换 src 用)
    std::map<unsigned long, MapInfo> maps_; ///< 地图信息表

    // ===== 配置 =====
    std::string pkg_pcd_dir_;               ///< fast_lio_global 包内 PCD 目录绝对路径
    std::string python_path_;               ///< python 节点的 PYTHONPATH
    bool use_coord_transform_;              ///< RELOC 初值来源:true=坐标变换换算 / false=固定电梯口锚点
    int initial_map_id_;                    ///< 开机自动起栈的初始地图ID(<=0 表示用列表中第一张)
    float initial_x_;                       ///< 开机起栈时的初始位姿X(机器人开机所在位置,不一定是0)
    float initial_y_;                       ///< 开机起栈时的初始位姿Y
    float initial_yaw_;                     ///< 开机起栈时的初始位姿yaw(弧度)

    // ===== 超时(秒) =====
    double load_ready_timeout_;
    double laser_alive_timeout_;
    double reloc_total_timeout_;

    int createAndBindTcpSocket();

    // ===== 节点启停(C++ 直接管理进程) =====
    pid_t launchNode(const NodeSpec& spec);              ///< fork + execvp(rosrun ...),返回 PID
    void  killNode(pid_t pid);                           ///< 按进程组优雅(SIGINT)→强制(SIGKILL)杀死并回收
    void  stopCurrentNodes();                            ///< 杀掉当前所有 load 节点与 laserMapping
    std::vector<NodeSpec> buildLoadSpecs(const MapInfo& m) const;  ///< 构造 LOAD 阶段 5 个节点
    NodeSpec buildLaserSpec() const;                     ///< 构造 laserMapping 节点

    void publishInitialPoseMsg(float x, float y, float yaw);
    /** 跨图坐标换算:把 src_id 地图系下的位姿换算到 dst_id 地图系。
     *  仅当 use_coord_transform_=true 时启用(否则 RELOC 用固定电梯口锚点)。 */
    req_frame convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id);
    void sendReplyOnSocket(int client_fd, const replay_frame& reply);

    // ===== 两阶段 =====
    bool doLoad(unsigned long target_map_id);
    bool doReloc(float x, float y, float yaw);
    void loadMap(unsigned long target_map_id, unsigned long req_seq, int client_fd);
    void relocalize(unsigned long target_map_id, unsigned long req_seq, float x, float y, float yaw, int client_fd);

    void handleTcpData(int listenfd);
};
