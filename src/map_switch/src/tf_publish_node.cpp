#include "tf_publish_node.hpp"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

/**
 * @brief 计算数组指定范围的平均值（忽略无穷大值）
 * @tparam T 数组元素类型
 * @param arry 输入数组
 * @param start 起始索引
 * @param end 结束索引
 * @return 指定范围的平均值，如果全部是无穷大返回 infinity
 */
template<typename T>
float ArrayGetAverage(const std::vector<T>& arry, int start, int end){
    int array_size = arry.size();
    float total_value = 0;
    int filter_flag = 0;

    if(start > (array_size - 1) || end > (array_size - 1) || start > end) 
        return 0;

    for(int i = start; i <= end; i++){
        if(std::isinf(arry[i])){
            continue;
        }
        total_value += arry[i];
        filter_flag++;
    }

    if (filter_flag == 0) {
        return std::numeric_limits<float>::infinity();
    }

    return total_value / filter_flag;
}

/**
 * @brief 获取当前时间戳字符串，格式：YYYY-MM-DD HH:MM:SS.mmm
 * @return 时间戳字符串
 */
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}

/**
 * @brief 构造函数，初始化ROS参数、TF发布器、LaserScan订阅器
 */
TfPublish::TfPublish(){
    nh.param<std::string>("pub_topic",pub_topic,"/robot_position");
    nh.param<std::string>("source_frame",source_frame,"/robot");
    nh.param<std::string>("target_frame",target_frame,"/robot_body");
    nh.param<bool>("socket_print_en",socket_print_en,true);
    nh.param<double>("publish_tf_rate",publish_tf_rate,10.0);

    nh.param<std::string>("server_addr",server_addr,"192.168.110.206");
    nh.param<int>("server_PORT",server_PORT,6001);
    nh.param<std::string>("client_addr",client_addr,"192.168.110.206");
    nh.param<int>("client_PORT",client_PORT,6002);

    nh.param<bool>("socket_en",socket_en,true);
    nh.param<bool>("http_en",http_en,true);

    tf_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_topic,100);
    nh.param<std::string>("sub_LaserScan_topic",sub_LaserScan_topic,"/mid360/scan");
    sub_LaserScan = nh.subscribe(sub_LaserScan_topic,1000,&TfPublish::LaserscanCallback,this);
}

/**
 * @brief LaserScan回调函数，计算左右侧平均距离并保存扫描数据
 * @param msg LaserScan消息指针
 */
void TfPublish::LaserscanCallback(const sensor_msgs::LaserScan::Ptr &msg){
    std::vector<float> angle_range(std::begin(msg->ranges),std::end(msg->ranges));
    left_distance = ArrayGetAverage<float>(angle_range, 265, 275);
    right_distance = ArrayGetAverage<float>(angle_range, 85, 95);
    std::copy(angle_range.rbegin()+90,angle_range.rbegin()+271,locationData_send.data.dist);
}

/**
 * @brief 获取socket启用状态
 * @return true表示启用
 */
bool TfPublish::socketEn_get() const{
    return socket_en;
}

/**
 * @brief 获取HTTP启用状态
 * @return true表示启用
 */
bool TfPublish::httpEn_get() const{
    return http_en;
}

/**
 * @brief 启动UDP socket服务线程，处理客户端请求并发送位置数据
 * @return 成功返回0，失败返回-1
 */
int TfPublish::socketServer(){
    // ROS_WARN("Waiting for relocalization to succeed...");
    // std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);
    // ROS_WARN("get for relocalization to succeed...");

    ROS_WARN("<--------into socket_thread--------->");
    ros::Rate socket_server_rate(100);

    /** 创建UDP socket */
    if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1){
        ROS_ERROR("Socket creation failed");
        return -1;
    }

    /** 配置服务器地址 */
    memset(&socket_ServerAddr, 0,sizeof(socket_ServerAddr));
    socket_ServerAddr.sin_family = AF_INET;
    socket_ServerAddr.sin_port = htons(server_PORT);
    if(inet_pton(AF_INET,server_addr.c_str(),&socket_ServerAddr.sin_addr) <= 0){
        ROS_ERROR("Invalid address/ Address not supported");
        return -1;
    }

    if(bind(sockfd,(struct sockaddr *)&socket_ServerAddr,sizeof(socket_ServerAddr)) < 0){
        ROS_ERROR("bind failed");
        return -1;
    }

    /** 配置客户端地址 */
    memset(&socket_Client, 0,sizeof(socket_Client));
    socket_Client.sin_family = AF_INET;
    socket_Client.sin_port = htons(client_PORT);
    if(inet_pton(AF_INET,client_addr.c_str(),&socket_Client.sin_addr) <= 0){
        ROS_ERROR("Invalid client address/ Address not supported");
        return -1;
    }

    socklen_t clientAddrLen = sizeof(socket_Client);

    /** 循环接收客户端数据并返回当前位置 */
    while (ros::ok()){
        if(recvfrom(sockfd, &receive_flag, sizeof(receive_flag), 0, (struct sockaddr *)&socket_Client, &clientAddrLen) < 0){
            ROS_ERROR("recv failed");
        }

        if(receive_flag.frame_type == 1){
            std::unique_lock<std::mutex> lock(mtx);
            receive_flag.frame_type = 2;
            locationData_send.frame_type = 2;
            locationData_send.seq = receive_flag.seq;
            sendto(sockfd, &locationData_send, sizeof(locationData_send), 0, (struct sockaddr *)&socket_Client, sizeof(socket_Client));

            ROS_WARN("[%s] x: %.3f  y: %.3f  yaw: %.3f",
                    getCurrentTimestamp().c_str(),
                    nav_data.x,
                    nav_data.y,
                    nav_data.yaw_angle);
        }

        socket_server_rate.sleep();
    }
    return 0;
}

/**
 * @brief 运行TF监听和数据发布线程
 */
void TfPublish::run(){
    ros::Rate rate(publish_tf_rate);
    memset(&nav_data, 0,sizeof(nav_data));
    memset(&locationData_send, 0,sizeof(locationData_send));
    memset(&rel_coords_data, 0,sizeof(rel_coords_data));

    ROS_WARN("<--------Transform is running-------->");

    while (ros::ok()){
        try{
            tf::StampedTransform TF;
            tf::Quaternion rotation;

            /** 等待TF变换 */
            listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5.0));
            if (!listener.canTransform(target_frame, source_frame, ros::Time(0))) {
                ROS_WARN("Failed to get transform from %s to %s", source_frame.c_str(), target_frame.c_str());
                continue;
            }

            /** 获取变换信息 */
            listener.lookupTransform(target_frame,source_frame,ros::Time(0),TF);
            rotation = TF.getRotation();
            tf::Matrix3x3 rotation_matrix(rotation);
            rotation_matrix.getRPY(roll,pitch,yaw);

            /** 更新共享导航数据 */
            {
                std::unique_lock<std::mutex> lock(mtx);
                nav_data = {0,float(TF.getOrigin().x()),float(TF.getOrigin().y()),float(TF.getOrigin().z()),float(yaw),left_distance,right_distance};

                locationData_send.frame_type = 0;
                locationData_send.data.x = TF.getOrigin().x();
                locationData_send.data.y = TF.getOrigin().y();
                locationData_send.data.yaw = yaw;

                rel_coords_data[0]=TF.getOrigin().x();
                rel_coords_data[1]=TF.getOrigin().y();
                rel_coords_data[2]=TF.getOrigin().z();
                rel_coords_data[3]=roll;
                rel_coords_data[4]=pitch;
                rel_coords_data[5]=yaw;
            }

            /** 可选日志输出 */
            if(socket_print_en){
                ROS_WARN("Got transform from %s to %s: [%f, %f, %f, %f, %f]",
                    source_frame.c_str(),target_frame.c_str(),
                    TF.getOrigin().x(),
                    TF.getOrigin().y(),
                    yaw,
                    left_distance,
                    right_distance);                 
            } 
        }
        catch(tf::LookupException &e){
            ROS_WARN("Failed to get transform");
        }

        ros::spinOnce();
        rate.sleep();
    }  
}
