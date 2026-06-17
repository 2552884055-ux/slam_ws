/**
 * @file tcp_map_switch_node.h
 * @brief TCP地图切换节点类定义文件
 *
 * 本模块实现一个基于TCP的地图管理与切换功能：
 * - 通过TCP监听客户端请求，实现地图切换控制；
 * - 管理地图启动与停止脚本；
 * - 负责不同地图坐标系之间的坐标转换；
 * - 在ROS系统中发布初始位姿以实现重定位。
 *
 * 作者:  
 * 日期: 2025-10-15
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <map>
#include <cmath>
#include <errno.h>

/**
 * @struct req_frame
 * @brief TCP客户端请求数据帧结构
 *
 * 用于描述客户端发起的地图切换请求或坐标转换请求。
 */
struct req_frame
{
    unsigned long frame_type;   ///< 帧类型标识（例如：1=切换地图、2=更新坐标等）
    unsigned long seq;          ///< 请求序列号，用于追踪应答
    float x;                    ///< 源地图中的X坐标
    float y;                    ///< 源地图中的Y坐标
    float yaw;                  ///< 源地图中的航向角（弧度制）
};

/**
 * @struct replay_frame
 * @brief TCP服务端应答数据帧结构
 *
 * 服务端向客户端返回的操作结果与序列号。
 */
struct replay_frame
{
    bool result;                ///< 操作结果（true表示成功，false表示失败）
    unsigned long seq;          ///< 对应请求的序列号
};

/**
 * @struct MapInfo
 * @brief 地图信息结构体
 *
 * 存储地图启动脚本路径与该地图相对于参考地图（通常是map1）的位姿转换关系。
 */
struct MapInfo {
    unsigned long id;           ///< 地图唯一ID
    std::string start_script;   ///< 启动该地图的脚本路径
    float tx_to_map1;           ///< 相对map1的X平移量
    float ty_to_map1;           ///< 相对map1的Y平移量
    float theta_to_map1;        ///< 相对map1的旋转角（弧度）
};

/**
 * @struct MapTransform
 * @brief 地图间位姿转换参数
 */
struct MapTransform {
    float tx;                   ///< 平移量X
    float ty;                   ///< 平移量Y
    float theta;                ///< 旋转角（弧度）
};

/**
 * @class TcpMapSwitchNode
 * @brief TCP地图切换核心类
 *
 * 该类负责管理整个地图切换流程，包括：
 * - 启动TCP服务器监听客户端请求；
 * - 解析客户端发送的请求帧；
 * - 停止当前地图并启动目标地图；
 * - 坐标系转换与初始位姿发布；
 * - 与ROS系统通信（发布/订阅话题）。
 */
class TcpMapSwitchNode
{
public:
    /**
     * @brief 构造函数，初始化内部成员变量。
     */
    TcpMapSwitchNode();

    /**
     * @brief 析构函数，释放资源并安全关闭TCP连接。
     */
    ~TcpMapSwitchNode();

    /**
     * @brief 初始化节点参数、加载配置并创建TCP监听端口。
     * @return true 表示初始化成功；false 表示失败。
     */
    bool init();

    /**
     * @brief 启动TCP服务监听线程。
     */
    void start();

    /**
     * @brief 停止TCP服务并关闭当前地图进程。
     */
    void stop();

private:
    // ===== ROS相关成员 =====
    ros::NodeHandle nh;                     ///< ROS节点句柄
    bool isRunning_;                        ///< 主循环运行状态标志

    // ===== 网络通信相关 =====
    int sockfd_;                            ///< 客户端socket描述符
    int listenfd_;                          ///< 监听socket描述符
    int map_switch_PORT_;                   ///< TCP监听端口号
    std::string server_addr_;               ///< TCP服务器IP地址

    std::thread tcpThread_;                 ///< TCP监听线程
    std::mutex mutex_;                      ///< 通用互斥锁
    std::mutex pidMutex_;                   ///< 地图进程PID互斥锁

    // ===== 地图管理相关 =====
    pid_t currentMapPid;                    ///< 当前运行地图的进程PID
    unsigned long current_map_id_;          ///< 当前激活的地图ID
    std::map<unsigned long, MapInfo> maps_; ///< 所有可用地图信息表

    // ===== 脚本路径配置 =====
    std::string stop_script_path_;          ///< 停止地图的脚本路径
    std::string initial_pose_script_path_;  ///< 发布初始位姿的脚本路径

    /**
     * @brief 创建并绑定TCP监听socket。
     * @return 成功返回socket描述符，失败返回-1。
     */
    int createAndBindTcpSocket();

    /**
     * @brief 启动指定路径的地图启动脚本。
     * @param scriptPath 脚本路径
     * @return 启动的子进程PID。
     */
    pid_t startScript(const std::string& scriptPath);

    /**
     * @brief 执行停止脚本，终止当前运行的地图。
     * @return true 表示执行成功，false 表示失败。
     */
    bool executeStopScript();

    /**
     * @brief 发布初始位姿到ROS话题，用于重定位。
     * @param x X坐标
     * @param y Y坐标
     * @param yaw 航向角（弧度）
     */
    void PublishInitialPose(float x, float y, float yaw);

    /**
     * @brief 执行地图间坐标转换。
     * @param src 源地图下的坐标信息
     * @param src_id 源地图ID
     * @param dst_id 目标地图ID
     * @return 转换后的目标地图坐标。
     */
    req_frame convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id);

    /**
     * @brief 向客户端socket发送应答数据。
     * @param client_fd 客户端socket描述符
     * @param reply 应答数据结构
     */
    void sendReplyOnSocket(int client_fd, const replay_frame& reply);

    /**
     * @brief 启动目标地图并进行坐标变换与重定位。
     * @param target_map_id 目标地图ID
     * @param req_seq 请求序列号
     * @param x 坐标X
     * @param y 坐标Y
     * @param yaw 航向角
     * @param client_fd 客户端socket描述符
     */
    void launchMap(unsigned long target_map_id, unsigned long req_seq, float x, float y, float yaw, int client_fd);

    /**
     * @brief 处理TCP数据接收与解析逻辑。
     * @param listenfd 监听socket描述符
     */
    void handleTcpData(int listenfd);
};

