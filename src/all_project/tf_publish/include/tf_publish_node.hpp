/**
 * @file tf_publish.h
 * @brief TF发布与雷达数据处理模块头文件
 *
 * 本模块主要功能：
 * - 订阅激光雷达（LaserScan）数据；
 * - 通过tf库实现坐标变换发布；
 * - 处理机器人与障碍物的实时坐标与姿态信息；
 * - 管理UDP通信数据结构；
 * - 提供时间戳与均值计算等通用工具。
 *
 * 作者:  
 * 日期: 2025-10-15
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <limits>

#define LINE_NUM 181   ///< 雷达扫描线数量（通常对应LaserScan角度分辨率）

/**
 * @struct coordination_data
 * @brief 三维坐标结构（X、Y、Z）
 */
typedef struct coordination_data {
    float x;    ///< X坐标
    float y;    ///< Y坐标
    float z;    ///< Z坐标
} dog_now_coordition, obstacle_now_coordition;

/**
 * @struct dog_info
 * @brief 机器人信息结构体
 */
typedef struct dog_info {
    dog_now_coordition coordination; ///< 机器人当前坐标
    float angle;                     ///< 当前朝向角（单位：弧度）
} dog_info;

/**
 * @struct obstacle_info
 * @brief 障碍物信息结构体
 */
typedef struct obstacle_info {
    obstacle_now_coordition coordination; ///< 障碍物中心坐标
    float len_x;                          ///< 障碍物X方向尺寸
    float len_y;                          ///< 障碍物Y方向尺寸
} obstacle_info;

/**
 * @struct laddar_data
 * @brief 激光雷达数据结构体
 */
struct laddar_data {
    double x;                    ///< 机器人X坐标
    double y;                    ///< 机器人Y坐标
    double yaw;                  ///< 航向角
    double dist[LINE_NUM];       ///< 雷达扫描距离数组
};

/**
 * @struct laddar_receive_frame
 * @brief 雷达数据帧结构（带有帧头与序号）
 */
struct laddar_receive_frame {
    unsigned long frame_type;    ///< 数据帧类型
    unsigned long seq;           ///< 序列号
    laddar_data data;            ///< 实际雷达数据
};

/**
 * @struct laddar_req_frame
 * @brief 雷达请求帧结构
 */
struct laddar_req_frame {
    unsigned long frame_type;    ///< 请求帧类型
    unsigned long seq;           ///< 请求序列号
};

/**
 * @struct udp_data
 * @brief UDP传输数据结构，用于机器人与上位机通信
 */
typedef struct udp_data {
    int command;                 ///< 控制指令码
    float x;                     ///< X坐标
    float y;                     ///< Y坐标
    float z;                     ///< Z坐标
    float yaw_angle;             ///< 航向角（弧度）
    float left_obs_dist;         ///< 左侧障碍物距离
    float right_obs_dist;        ///< 右侧障碍物距离
} udp_data;

/**
 * @brief 计算数组指定范围的平均值（通用模板函数）
 * @tparam T 数据类型
 * @param arry 输入数组
 * @param start 起始索引
 * @param end 结束索引
 * @return 指定范围内元素的平均值
 */
template<typename T>
float ArrayGetAverage(const std::vector<T>& arry, int start, int end);

/**
 * @brief 获取当前时间戳字符串（精确到毫秒）
 * @return 格式化时间字符串，例如 "2025-10-15 14:30:21.123"
 */
std::string getCurrentTimestamp();

/**
 * @class TfPublish
 * @brief TF发布与雷达信息管理类
 *
 * 该类主要功能包括：
 * - 订阅LaserScan消息并解析；
 * - 发布TF坐标变换；
 * - 管理机器人与障碍物的位姿信息；
 * - 通过UDP进行数据传输；
 * - 提供实时坐标监测与数据同步。
 */
class TfPublish {
private:
    ros::NodeHandle nh;                         ///< ROS节点句柄

    ros::Publisher tf_pub;                      ///< TF数据发布器
    ros::Subscriber sub_LaserScan;              ///< LaserScan订阅器

    tf::TransformListener listener;             ///< TF监听器

    // ===== 配置参数 =====
    std::string pub_topic;                      ///< TF发布话题名
    std::string source_frame;                   ///< 源坐标系
    std::string target_frame;                   ///< 目标坐标系
    std::string sub_LaserScan_topic;            ///< LaserScan订阅话题名
    double publish_tf_rate;                     ///< TF发布频率（Hz）
    double socket_rate;                         ///< Socket发送频率（Hz）
    bool http_en;                               ///< HTTP功能开关
    bool socket_en;                             ///< Socket通信开关
    bool socket_print_en;                       ///< 是否打印Socket通信日志

    // ===== 内部状态变量 =====
    int get_command;                            ///< 当前接收到的控制命令
    double roll, pitch, yaw;                    ///< 机器人姿态角
    double rel_coords_data[6];                  ///< 机器人相对坐标缓存

    dog_now_coordition robot_position;          ///< 当前机器人坐标
    obstacle_now_coordition obstacle_position;  ///< 当前障碍物坐标
    dog_info robot_nav_info;                    ///< 机器人导航信息
    obstacle_info obstacle_nav_info;            ///< 障碍物导航信息

    udp_data nav_data;                          ///< 待发送UDP数据
    udp_data recv_data;                         ///< 接收UDP数据缓存

    float left_distance = std::numeric_limits<float>::infinity();   ///< 左侧距离初始化为无穷大
    float right_distance = std::numeric_limits<float>::infinity();  ///< 右侧距离初始化为无穷大

    laddar_receive_frame locationData_send;     ///< 发送用雷达数据帧
    laddar_req_frame receive_flag;              ///< 雷达数据接收标志帧

    // ===== Socket相关 =====
    int server_PORT;                            ///< 服务器端口号
    int client_PORT;                            ///< 客户端端口号
    std::string client_addr;                    ///< 客户端IP地址
    std::string server_addr;                    ///< 服务器IP地址
    int sockfd;                                 ///< Socket描述符
    struct sockaddr_in socket_ServerAddr;       ///< 服务端Socket结构
    struct sockaddr_in socket_Client;           ///< 客户端Socket结构

    std::mutex mtx;                             ///< 数据互斥锁

public:
    laddar_data location_with_2dLidar;          ///< 当前2D雷达位置数据

public:
    /**
     * @brief 构造函数，初始化ROS节点与成员变量。
     */
    TfPublish();

    /**
     * @brief LaserScan话题回调函数，用于处理雷达扫描数据。
     * @param msg 激光雷达消息指针
     */
    void LaserscanCallback(const sensor_msgs::LaserScan::Ptr &msg);

    /**
     * @brief 获取Socket通信开关状态。
     * @return true 表示启用Socket通信。
     */
    bool socketEn_get() const;

    /**
     * @brief 获取HTTP功能开关状态。
     * @return true 表示启用HTTP功能。
     */
    bool httpEn_get() const;

    /**
     * @brief 初始化并启动Socket服务器。
     * @return 返回创建的socket描述符。
     */
    int socketServer();

    /**
     * @brief 主运行函数，负责TF发布、通信与数据同步。
     */
    void run();
};

