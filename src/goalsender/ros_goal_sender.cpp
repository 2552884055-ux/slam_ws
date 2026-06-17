#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include "socket.h"
#include "data_type.h"
#include "network/frame.hpp"

// 配置参数
struct Config {
    std::string target_ip;
    int target_port;
    int local_port;
    bool initialized;
    int goal_counter;
    
    Config() : target_ip("192.168.110.180"), target_port(8115), 
               local_port(8112), initialized(false), goal_counter(0) {}
};

// 地图参数（用于坐标转换）
struct MapParams {
    float resolution;      // 地图分辨率 (米/像素)
    float origin_x;        // 地图原点 X (米)
    float origin_y;        // 地图原点 Y (米)
    int width;             // 地图宽度 (像素)
    int height;            // 地图高度 (像素)
    bool loaded;           // 是否已加载参数
    
    MapParams() : resolution(0.05), origin_x(0.0), origin_y(0.0), 
                  width(0), height(0), loaded(false) {}
    
    // 世界坐标转像素坐标
    void worldToPixel(float world_x, float world_y, int& pixel_x, int& pixel_y) const {
        pixel_x = static_cast<int>((world_x - origin_x) / resolution);
        pixel_y = static_cast<int>((world_y - origin_y) / resolution);
    }
};

// 最新位置缓存
struct LatestPosition {
    bool has_new_position;
    float x, y, z, yaw;
    int position_id;
    std::string frame_id;
    ros::Time timestamp;
    
    LatestPosition() : has_new_position(false), x(0), y(0), z(0), yaw(0), position_id(0) {}
};

Config g_config;
MapParams g_map_params;
LatestPosition g_latest_position;
std::mutex g_position_mutex;  // 保护位置数据的互斥锁

// 前向声明
void udp_receive_callback(netWorkBase* nethandler, void* buf, int size, sockaddr_in client_addr);
void send_lidar_position(float x, float y, float z, float yaw);

// 发送激光雷达当前位置
void send_lidar_position(float x, float y, float z, float yaw) {
    if (!g_config.initialized) {
        ROS_WARN("UDP not initialized, skipping send");
        return;
    }
    
    // 构建第一帧：发送 X, Y 坐标
    FrameBuilder builder1;
    builder1.source = ROBOT_SYSTEM;      // 源：机器人系统
    builder1.dest = ROBOT_SYSTEM;        // 目标：机器人系统
    builder1.type = RESPONSE;            // 类型：数据上报
    builder1.subObj = SUBOBJ_ROBOT_NAV_XY;           // 子对象：10=位置(XY)
    builder1.data.hasData = 1;           // 有新数据
    builder1.data.data1 = g_latest_position.position_id;  // 位置ID
    builder1.data.data2 = 0;             // 状态：0=实时位置
    builder1.data.data3 = x;             // X坐标
    builder1.data.data4 = y;             // Y坐标
    
    sendCommand(0, builder1, g_config.target_ip.c_str(), g_config.target_port);
    
    // 构建第二帧：发送 Z 坐标和朝向角
    FrameBuilder builder2;
    builder2.source = ROBOT_SYSTEM;
    builder2.dest = ROBOT_SYSTEM;
    builder2.type = RESPONSE;
    builder2.subObj = SUBOBJ_ROBOT_NAV_ZYAW;                // 子对象：11=位置(Z+Yaw)
    builder2.data.hasData = 1;
    builder2.data.data1 = g_latest_position.position_id;  // 相同的位置ID
    builder2.data.data2 = 0;
    builder2.data.data3 = z;             // Z坐标
    builder2.data.data4 = yaw;           // 朝向角（弧度）
    
    sendCommand(0, builder2, g_config.target_ip.c_str(), g_config.target_port);
    
    ROS_INFO("Sent position #%d to %s:%d - Position(%.3f, %.3f, %.3f) Yaw(%.3f rad / %.1f deg)",
             g_latest_position.position_id,
             g_config.target_ip.c_str(),
             g_config.target_port,
             x, y, z, yaw, yaw * 180.0 / M_PI);
}

// UDP 接收回调：处理请求帧
void udp_receive_callback(netWorkBase* nethandler, void* buf, int size, sockaddr_in client_addr) {
    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    int client_port = ntohs(client_addr.sin_port);
    
    if (size < static_cast<int>(sizeof(frame))) {
        return;
    }
    
    frame* pf = reinterpret_cast<frame*>(buf);
    
    // 只处理请求帧
    if (pf->type != REQUEST) {
        return;
    }
    
    uint32_t sub_obj = get_frame_subobj(pf);
    
    // 检查是否是请求激光雷达位置（subObj = 10）
    if (sub_obj == 10) {
        ROS_INFO("----------------------------------------");
        ROS_INFO("Received position request from %s:%d", client_ip, client_port);
        
        std::lock_guard<std::mutex> lock(g_position_mutex);
        
        if (g_latest_position.has_new_position) {
            // 计算像素坐标
            int pixel_x = 0, pixel_y = 0;
            if (g_map_params.loaded) {
                g_map_params.worldToPixel(g_latest_position.x, g_latest_position.y, pixel_x, pixel_y);
            }
            
            ROS_INFO("Sending current LiDAR position #%d:", g_latest_position.position_id);
            ROS_INFO("  World Position: (%.3f, %.3f, %.3f) m", 
                     g_latest_position.x, g_latest_position.y, g_latest_position.z);
            if (g_map_params.loaded) {
                ROS_INFO("  Pixel Position: (%d, %d) px", pixel_x, pixel_y);
            }
            ROS_INFO("  Yaw: %.3f rad (%.1f deg)", 
                     g_latest_position.yaw, g_latest_position.yaw * 180.0 / M_PI);
            
            // 发送位置
            send_lidar_position(
                g_latest_position.x,
                g_latest_position.y,
                g_latest_position.z,
                g_latest_position.yaw
            );
            
            ROS_INFO("Position sent successfully!");
        } else {
            ROS_WARN("No position data available to send");
        }
        ROS_INFO("----------------------------------------");
    }
}

// ROS回调函数：接收激光雷达位置并缓存（不立即发送）
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_position_mutex);
    
    // 提取位置信息
    g_latest_position.x = msg->pose.pose.position.x;
    g_latest_position.y = msg->pose.pose.position.y;
    g_latest_position.z = msg->pose.pose.position.z;
    g_latest_position.frame_id = msg->header.frame_id;
    g_latest_position.timestamp = msg->header.stamp;
    
    // 提取朝向信息（四元数转欧拉角）
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    g_latest_position.yaw = yaw;
    
    // 更新位置ID和标志
    g_config.goal_counter++;
    g_latest_position.position_id = g_config.goal_counter;
    g_latest_position.has_new_position = true;
    
    // 每隔一段时间打印一次（避免刷屏）
    static int print_counter = 0;
    if (++print_counter >= 50) {  // 每50次打印一次
        int pixel_x = 0, pixel_y = 0;
        if (g_map_params.loaded) {
            g_map_params.worldToPixel(g_latest_position.x, g_latest_position.y, pixel_x, pixel_y);
            ROS_INFO("LiDAR Position Updated (ID: %d): World(%.3f, %.3f, %.3f)m Pixel(%d, %d)px Yaw(%.3f rad / %.1f deg)", 
                     g_latest_position.position_id,
                     g_latest_position.x, g_latest_position.y, g_latest_position.z,
                     pixel_x, pixel_y,
                     g_latest_position.yaw, g_latest_position.yaw * 180.0 / M_PI);
        } else {
            ROS_INFO("LiDAR Position Updated (ID: %d): World(%.3f, %.3f, %.3f)m Yaw(%.3f rad / %.1f deg)", 
                     g_latest_position.position_id,
                     g_latest_position.x, g_latest_position.y, g_latest_position.z,
                     g_latest_position.yaw, g_latest_position.yaw * 180.0 / M_PI);
        }
        print_counter = 0;
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_position_sender_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("=== ROS LiDAR Position Sender Node Started (Request-Response Mode) ===");
    
    // 读取参数
    nh_private.param<std::string>("target_ip", g_config.target_ip, "192.168.110.180");
    nh_private.param<int>("target_port", g_config.target_port, 8111);
    nh_private.param<int>("local_port", g_config.local_port, 8112);
    
    std::string odom_topic;
    nh_private.param<std::string>("odom_topic", odom_topic, "/Odometry");
    
    // 读取地图参数（可选）
    nh_private.param<float>("map_resolution", g_map_params.resolution, 0.05);
    nh_private.param<float>("map_origin_x", g_map_params.origin_x, 0.0);
    nh_private.param<float>("map_origin_y", g_map_params.origin_y, 0.0);
    nh_private.param<int>("map_width", g_map_params.width, 0);
    nh_private.param<int>("map_height", g_map_params.height, 0);
    
    // 检查是否加载了地图参数
    if (g_map_params.width > 0 && g_map_params.height > 0) {
        g_map_params.loaded = true;
        ROS_INFO("Map parameters loaded:");
        ROS_INFO("  Resolution: %.3f m/pixel", g_map_params.resolution);
        ROS_INFO("  Origin: (%.3f, %.3f) m", g_map_params.origin_x, g_map_params.origin_y);
        ROS_INFO("  Size: %d x %d pixels", g_map_params.width, g_map_params.height);
    } else {
        ROS_WARN("Map parameters not provided. Pixel coordinates will not be displayed.");
        ROS_WARN("To enable, add map parameters to launch file:");
        ROS_WARN("  <param name=\"map_resolution\" value=\"0.05\" />");
        ROS_WARN("  <param name=\"map_origin_x\" value=\"-10.0\" />");
        ROS_WARN("  <param name=\"map_origin_y\" value=\"-10.0\" />");
        ROS_WARN("  <param name=\"map_width\" value=\"400\" />");
        ROS_WARN("  <param name=\"map_height\" value=\"400\" />");
    }
    
    ROS_INFO("Configuration:");
    ROS_INFO("  Target IP: %s", g_config.target_ip.c_str());
    ROS_INFO("  Target Port: %d", g_config.target_port);
    ROS_INFO("  Local Port: %d (listening for requests)", g_config.local_port);
    ROS_INFO("  Odometry Topic: %s", odom_topic.c_str());
    
    // 初始化统一的 UDP 模块（用于接收和发送）
    if (setup_udp_socket(g_config.local_port) < 0) {
        ROS_ERROR("Failed to initialize UDP module");
        return -1;
    }
    ROS_INFO("UDP module initialized on port %d", g_config.local_port);
    g_config.initialized = true;
    
    // 启动 UDP 接收线程
    start_udp_receiver(udp_receive_callback);
    ROS_INFO("UDP receive thread started, listening for requests...");
    
    // 订阅激光雷达里程计话题
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odometryCallback);
    
    ROS_INFO("Subscribed to %s", odom_topic.c_str());
    ROS_INFO("========================================");
    ROS_INFO("MODE: Request-Response (LiDAR Position)");
    ROS_INFO("  1. LiDAR odometry continuously updates position");
    ROS_INFO("  2. Client sends request to port %d", g_config.local_port);
    ROS_INFO("  3. Current position will be sent to %s:%d", 
             g_config.target_ip.c_str(), g_config.target_port);
    ROS_INFO("========================================");
    ROS_INFO("Waiting for odometry data and requests...");
    
    // 进入ROS循环
    ros::spin();
    
    // 清理
    stop_udp_receiver();
    
    return 0;
}
