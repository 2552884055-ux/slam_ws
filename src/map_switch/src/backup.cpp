// tcp_map_switch_node.cpp
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

struct req_frame
{
    unsigned long frame_type;
    unsigned long seq;
    float x;
    float y;
    float yaw;
};

struct replay_frame
{
    bool result;
    unsigned long seq;
};

struct MapInfo {
    unsigned long id;
    std::string start_script;
    // transform from this map to map1 (tx, ty, theta)
    float tx_to_map1;
    float ty_to_map1;
    float theta_to_map1;
};

struct MapTransform {
    float tx;
    float ty;
    float theta;
};

class TcpMapSwitchNode
{
public:
    TcpMapSwitchNode() :
        sockfd_(-1), listenfd_(-1), isRunning_(false), currentMapPid(-1), current_map_id_(0)
    {
        nh.param<std::string>("server_addr", server_addr_, "192.168.2.100"); 
        nh.param<int>("map_switch_PORT", map_switch_PORT_, 6050);

        nh.param<std::string>("stop_script", stop_script_path_, "/home/orangepi/slam_ws/src/map_switch/scripts/stop_current_map.sh");
        nh.param<std::string>("initial_pose_script", initial_pose_script_path_, "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/scripts/publish_initial_pose.py");

        // Load maps from ROS parameter (YAML)
        XmlRpc::XmlRpcValue map_list;
        if (nh.getParam("maps", map_list) && map_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < map_list.size(); ++i) {
                XmlRpc::XmlRpcValue map_item = map_list[i];
                MapInfo info;
                info.id = static_cast<int>(map_item["id"]);
                info.start_script = static_cast<std::string>(map_item["start_script"]);
                info.tx_to_map1 = static_cast<double>(map_item["tx_to_map1"]);
                info.ty_to_map1 = static_cast<double>(map_item["ty_to_map1"]);
                info.theta_to_map1 = static_cast<double>(map_item["theta_to_map1"]);
                maps_[info.id] = info;
            }
        } else {
            ROS_FATAL("No maps configured in 'maps' parameter. Exiting.");
        }

        ROS_WARN("TcpMapSwitchNode constructed. Listening %s:%d", server_addr_.c_str(), map_switch_PORT_);
    }

    ~TcpMapSwitchNode()
    {
        stop();
    }

    bool init()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        listenfd_ = createAndBindTcpSocket();
        if (listenfd_ == -1)
        {
            ROS_ERROR("Failed to create and bind TCP socket");
            return false;
        }
        if (listen(listenfd_, 10) < 0)
        {
            ROS_FATAL("Failed to listen on TCP socket: %s", strerror(errno));
            close(listenfd_);
            listenfd_ = -1;
            return false;
        }
        ROS_WARN("TCP Map Switch Node started. Waiting for commands...");

        // === 默认启动第一个地图 ===
        if (!maps_.empty() && current_map_id_ == 0)
        {
            auto first_map_it = maps_.begin();
            ROS_WARN("No map active. Launching default map id %lu ...", first_map_it->first);
            launchMap(first_map_it->first, 0, 6.54, -4.2, 1.57, -1); // -1 表示没有 TCP 客户端
        }

        return true;
    }

    void start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (listenfd_ != -1 && !isRunning_)
        {
            isRunning_ = true;
            tcpThread_ = std::thread(&TcpMapSwitchNode::handleTcpData, this, listenfd_);
        }
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!isRunning_) return;
            isRunning_ = false;
            if (listenfd_ != -1) {
                close(listenfd_);
                listenfd_ = -1;
            }
        }
        if (tcpThread_.joinable())
            tcpThread_.join();

        std::lock_guard<std::mutex> lock(pidMutex_);
        if (sockfd_ != -1) {
            close(sockfd_);
            sockfd_ = -1;
        }
    }

private:
    ros::NodeHandle nh;
    bool isRunning_;
    int sockfd_;
    int listenfd_;

    int map_switch_PORT_;
    std::string server_addr_;

    std::thread tcpThread_;
    std::mutex mutex_;
    std::mutex pidMutex_;  

    pid_t currentMapPid;
    unsigned long current_map_id_; // 0 = unknown, otherwise map id

    std::map<unsigned long, MapInfo> maps_;

    std::string stop_script_path_;
    std::string initial_pose_script_path_;

    int createAndBindTcpSocket()
    {
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0){
            ROS_FATAL("Failed to create TCP socket: %s", strerror(errno));
            return -1;
        }

        struct sockaddr_in serverAddr;
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(map_switch_PORT_);

        if (server_addr_ == "0.0.0.0" || server_addr_ == "0") {
            serverAddr.sin_addr.s_addr = INADDR_ANY;
        } else {
            if (inet_pton(AF_INET, server_addr_.c_str(), &(serverAddr.sin_addr)) <= 0) {
                ROS_FATAL("Failed to convert IP address string to network format");
                close(sockfd);
                return -1;
            }
        }

        int opt = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
        {
            ROS_FATAL("Failed to bind TCP socket: %s", strerror(errno));
            close(sockfd);
            return -1;
        }

        return sockfd;
    }

    pid_t startScript(const std::string& scriptPath) {
        std::string command = "bash " + scriptPath + " & echo $!";
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            std::cerr << "Failed to start the script. Error executing popen." << std::endl;
            return -1;
        }

        char buffer[128];
        std::string pidStr;
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            pidStr = buffer;
            pidStr.erase(pidStr.find_last_not_of(" \n\r\t") + 1);
        } else {
            std::cerr << "Failed to read the PID of the started script." << std::endl;
            pclose(pipe);
            return -1;
        }

        pclose(pipe);

        try {
            pid_t pid = static_cast<pid_t>(std::stol(pidStr));
            std::cout << "Script started with PID: " << pid << std::endl;
            return pid;
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse PID: " << e.what() << std::endl;
            return -1;
        }
    }

    bool executeStopScript() {
        std::string command = "bash " + stop_script_path_;
        int result = std::system(command.c_str());
        if (result == 0) {
            std::cout << "Stop script executed successfully." << std::endl;
            return true;
        } else {
            std::cerr << "Failed to execute stop script. Error code: " << result << std::endl;
            return false;
        }
    }

    void PublishInitialPose(float x, float y, float yaw)
    {
        std::stringstream ss;
        ss << initial_pose_script_path_ << " " << x << " " << y << " " << 0 << " " << yaw << " " << 0 << " " << 0;
        std::string cmd = ss.str();
        int result = std::system(cmd.c_str());
        if (result != 0) {
            ROS_ERROR("Failed to execute initial pose script: %s", cmd.c_str());
        }
    }

    req_frame convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id)
    {
        if (src_id == dst_id) return src;

        auto mapToMap1 = [this](const req_frame& in, unsigned long id)->req_frame {
            req_frame out = in;
            if (id == 1) return out;
            auto it = maps_.find(id);
            if (it == maps_.end()) return out;
            float tx = it->second.tx_to_map1;
            float ty = it->second.ty_to_map1;
            float th = it->second.theta_to_map1;
            out.x = in.x * std::cos(th) - in.y * std::sin(th) + tx;
            out.y = in.x * std::sin(th) + in.y * std::cos(th) + ty;
            out.yaw = in.yaw + th;
            out.frame_type = 1;
            return out;
        };

        auto map1ToMap = [this](const req_frame& in, unsigned long id)->req_frame {
            req_frame out = in;
            if (id == 1) return out;
            auto it = maps_.find(id);
            if (it == maps_.end()) return out;
            float tx = it->second.tx_to_map1;
            float ty = it->second.ty_to_map1;
            float th = it->second.theta_to_map1;
            out.x = (in.x - tx) * std::cos(th) + (in.y - ty) * std::sin(th);
            out.y = -(in.x - tx) * std::sin(th) + (in.y - ty) * std::cos(th);
            out.yaw = in.yaw - th;
            out.frame_type = id;
            return out;
        };

        req_frame in_map1 = (src_id == 1) ? src : mapToMap1(src, src_id);
        req_frame out = (dst_id == 1) ? in_map1 : map1ToMap(in_map1, dst_id);
        out.seq = src.seq;
        out.frame_type = dst_id;
        return out;
    }

    void sendReplyOnSocket(int client_fd, const replay_frame& reply)
    {
        if (client_fd < 0) return;
        ssize_t send_len = send(client_fd, &reply, sizeof(reply), 0);
        if (send_len < 0){
            ROS_ERROR("send reply failed: %s", strerror(errno));
        } else {
            ROS_WARN("send reply succeeded for seq %lu", reply.seq);
        }
        close(client_fd);
    }

    void launchMap(unsigned long target_map_id, unsigned long req_seq, float x, float y, float yaw, int client_fd)
    {
        auto switchTask = [this, target_map_id, req_seq, x, y, yaw, client_fd]() {
            replay_frame reply;
            reply.seq = req_seq;
            reply.result = false;

            if (maps_.find(target_map_id) == maps_.end()) {
                ROS_WARN("Unknown target map id: %lu", target_map_id);
                reply.result = false;
                sendReplyOnSocket(client_fd, reply);
                return;
            }

            // Stop current map (via stop script)
            if (current_map_id_ != 0) {
                std::lock_guard<std::mutex> lock(pidMutex_);
                if (currentMapPid != -1) {
                    ROS_INFO("Stopping current map (pid=%d) via stop script...", currentMapPid);
                } else {
                    ROS_INFO("No known current map process; executing stop script to ensure no map runs.");
                }
                this->executeStopScript();
                // Note: stop script expected to terminate any running map; we don't forcibly kill PIDs here.
                currentMapPid = -1;
                current_map_id_ = 0;
            }

            // Start new map script
            const std::string scriptPath = maps_[target_map_id].start_script;
            pid_t newPid = this->startScript(scriptPath);

            {
                std::lock_guard<std::mutex> lock(pidMutex_);
                currentMapPid = newPid;
                current_map_id_ = (newPid != -1 ? target_map_id : 0);
            }

            if (newPid == -1) {
                ROS_ERROR("Failed to start map %lu script.", target_map_id);
                sendReplyOnSocket(client_fd, reply);
                return;
            }

            ROS_WARN("Started target map %lu (pid=%d). Waiting for initial pose flag...", target_map_id, newPid);

            // 阻塞等待一次 /waiting_for_initial_pose 消息
            std_msgs::Bool::ConstPtr initial_pose_flag = ros::topic::waitForMessage<std_msgs::Bool>("waiting_for_initial_pose", nh);
            
            if (initial_pose_flag && initial_pose_flag->data) {
                ROS_WARN("Received initial pose flag, publishing initial pose...");
                PublishInitialPose(x, y, yaw);
            } else {
                // 没接收到正确标志，再等 5 秒
                ros::Duration(5.0).sleep();
                ROS_WARN("Has been waiting for 20 seconds, publishing initial pose...");
                PublishInitialPose(x, y, yaw);
            }

            // 阻塞等待一次 map_to_odom_flag 消息
            std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);

            ROS_WARN("Map switch succeeded...");

            // 根据收到的消息更新回复结果
            if (relocalization_msg && relocalization_msg->data) {
                reply.result = true;
            } else {
                reply.result = false;
            }

            // 发送回复
            sendReplyOnSocket(client_fd, reply);

        };

        std::thread th(switchTask);
        th.detach();
    }

    void handleTcpData(int listenfd) {
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);

        ROS_WARN("handleTcpData thread started.");

        while (ros::ok()) {
            int client_fd = accept(listenfd, (struct sockaddr*)&clientAddr, &clientAddrLen);
            if (client_fd < 0) {
                if (!isRunning_) {
                    ROS_INFO("Listener closed, exiting handleTcpData loop.");
                    break;
                }
                ROS_ERROR("accept failed: %s", strerror(errno));
                continue;
            }

            struct timeval tv;
            tv.tv_sec = 10;
            tv.tv_usec = 0;
            setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

            req_frame request;
            ssize_t recv_len = recv(client_fd, &request, sizeof(request), 0);
            if (recv_len < 0) {
                ROS_ERROR("recv failed from client: %s", strerror(errno));
                close(client_fd);
                continue;
            } else if (recv_len == 0) {
                ROS_WARN("Client closed connection immediately.");
                close(client_fd);
                continue;
            } else if (recv_len != sizeof(request)) {
                ROS_ERROR("Received incorrect data length (%zd expected %zu). Closing client.", recv_len, sizeof(request));
                close(client_fd);
                continue;
            }

            // Received a valid request
            ROS_WARN("Received request: frame_type=%lu seq=%lu x=%f y=%f yaw=%f",
                     request.frame_type, request.seq, request.x, request.y, request.yaw);

            req_frame new_request = request;
            {
                std::lock_guard<std::mutex> lock(pidMutex_);
                if (current_map_id_ == 0) {
                    // unknown current map, no conversion: assume request frame is absolute (no transform)
                    new_request = request;
                    current_map_id_ = request.frame_type; // adopt client's frame as current
                    ROS_INFO("Unknown current map; adopting request's frame as current: %lu", current_map_id_);
                } else if (current_map_id_ == request.frame_type) {
                    // same map -> no transform, but may skip start
                    new_request = request;
                    ROS_WARN("Already at target map %lu. No map transformation needed.", request.frame_type);
                } else {
                    ROS_INFO("Converting coordinates from current_map(%lu) to requested map(%lu)...", current_map_id_, request.frame_type);
                    new_request = convertBetweenMaps(request, current_map_id_, request.frame_type);
                }
            }
            // print old and new coordinates
            ROS_WARN("Old Coordinates: x = %f, y = %f, yaw = %f", request.x, request.y, request.yaw);
            ROS_WARN("New Coordinates: x = %f, y = %f, yaw = %f (target map %lu)", new_request.x, new_request.y, new_request.yaw, new_request.frame_type);

            launchMap(new_request.frame_type, new_request.seq, new_request.x, new_request.y, new_request.yaw, client_fd);
            {
                std::lock_guard<std::mutex> lock(pidMutex_);
                sockfd_ = -1;
            }
        }
            ROS_WARN("Exiting handleTcpData thread.");
    }
};

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tcp_map_switch_node");
    ros::NodeHandle nh;

    TcpMapSwitchNode node;
    if (!node.init())
    {
        return -1;
    }
    node.start();

    ros::spin();

    node.stop();
    return 0;
}


// tf_publish.cpp
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

#define LINE_NUM 181

typedef struct coordination_data{
    float x;
    float y;
    float z;
}dog_now_coordition, obstacle_now_coordition;

typedef struct dog_info{
    dog_now_coordition coordination;
    float angle;
}dog_info;

typedef struct obstacle_info{
    obstacle_now_coordition coordination;
    float len_x;
    float len_y;
}obstacle_info;


/*****************<dwa test>****************/

struct laddar_data
{
    double x;
    double y;
    double yaw;
    double dist[LINE_NUM];
};

struct laddar_receive_frame
{
    unsigned long frame_type;
    unsigned long seq;
    laddar_data data;
};

struct laddar_req_frame
{
    unsigned long frame_type;
    unsigned long seq;
};

/**********************************/

typedef struct udp_data{
    int command;
    float x;
    float y;
    float z;
    float yaw_angle;
    float left_obs_dist;
    float right_obs_dist;
}udp_data;


//取范围内均值函数
template<typename T>
float ArrayGetAverage(const std::vector<T>& arry, int start, int end){

    int array_size = arry.size();
    float total_value = 0;
    float get_average = 0;
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
        return std::numeric_limits<float>::infinity();  //避免除以零(若都为inf则返回inf)
    }

    get_average = total_value/filter_flag;

    //std::cout << "distance:" <<  << std::endl;
    return get_average;
}

// 获取当前时间戳字符串（毫秒精度）
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    
    // 获取毫秒部分
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

class TfPublish{
    private:
        ros::NodeHandle nh;

        ros::Publisher tf_pub;
        ros::Subscriber sub_LaserScan;

        tf::TransformListener listener;

        std::string pub_topic;
        std::string source_frame;
        std::string target_frame;
        std::string sub_LaserScan_topic;
        double publish_tf_rate;
        double socket_rate;
        bool http_en;
        bool socket_en;        
        bool socket_print_en;

        int get_command;
        double roll, pitch, yaw;
        double rel_coords_data[6];
        dog_now_coordition robot_position;
        obstacle_now_coordition obstacle_position;
        dog_info robot_nav_info;
        obstacle_info obstacle_nav_info;
        udp_data nav_data;
        udp_data recv_data;
        float left_distance = std::numeric_limits<float>::infinity();
        float right_distance = std::numeric_limits<float>::infinity();
        //dwa应答数据
        laddar_receive_frame locationData_send;
        laddar_req_frame receive_flag;


        int server_PORT;
        int client_PORT;
        std::string client_addr;
        std::string server_addr;
        int sockfd;
        struct sockaddr_in socket_ServerAddr;
        struct sockaddr_in socket_Client;

        std::mutex mtx;

    public:
        laddar_data location_with_2dLidar;
 
    public:
        TfPublish(){

            nh.param<std::string>("pub_topic",pub_topic,"/robot_position");
            nh.param<std::string>("source_frame",source_frame,"/robot");
            nh.param<std::string>("target_frame",target_frame,"/robot_body");
            nh.param<bool>("socket_print_en",socket_print_en,true);                   //终端打印使能
            nh.param<double>("publish_tf_rate",publish_tf_rate,10.0);         //发布速率

            nh.param<std::string>("server_addr",server_addr,"192.168.110.206");
            nh.param<int>("server_PORT",server_PORT,6001);
            nh.param<std::string>("client_addr",client_addr,"192.168.110.206");
            nh.param<int>("client_PORT",client_PORT,6002);

            nh.param<bool>("socket_en",socket_en,true);
            nh.param<bool>("http_en",http_en,true);

            tf_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_topic,100);  //队列大小100



            nh.param<std::string>("sub_LaserScan_topic",sub_LaserScan_topic,"/mid360/scan");
            sub_LaserScan = nh.subscribe(sub_LaserScan_topic,1000,&TfPublish::LaserscanCallback,this);  //缓冲区为1000


        } 


        //接收点云回调
        void LaserscanCallback(const sensor_msgs::LaserScan::Ptr &msg){
            //ROS_INFO(" get data ,into callback function ");      
            std::vector<float> angle_range(std::begin(msg->ranges),std::end(msg->ranges));

            //解算方向角如下:
            //       180
            //        |
            // 270<-     -> 90
            //        |
            //        0

            left_distance = ArrayGetAverage<float>(angle_range, 265, 275);
            right_distance = ArrayGetAverage<float>(angle_range, 85, 95);

            //location_with_2dLidar.dist
            std::copy(angle_range.rbegin()+90,angle_range.rbegin()+271,locationData_send.data.dist);

        }

        //类中使能返回
        bool socketEn_get() const{
            return socket_en;
        }
        bool httpEn_get() const{
            return http_en;
        }



        int socketServer(){

            // ROS_WARN("Waiting for relocalization to succeed...");
            // std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);
            // ROS_WARN("get for relocalization to succeed...");


            ROS_WARN("<--------into socket_thread--------->");
            ros::Rate socket_server_rate(100);
            /****创建socket连接(TCP:SOCK_STREAM  UDP:SOCK_DGRAM )*****/
            if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1){
                ROS_ERROR("Socket creation failed");
                return -1;
            }
            memset(&socket_ServerAddr, 0,sizeof(socket_ServerAddr));
            socket_ServerAddr.sin_family = AF_INET;
            socket_ServerAddr.sin_port = htons(server_PORT);

            if(inet_pton(AF_INET,server_addr.c_str(),&socket_ServerAddr.sin_addr) <= 0){
                ROS_ERROR("Invalid address/ Address not supported");
                return -1;
            }

            /***bind本机地址***/ 
            if(bind(sockfd,(struct sockaddr *)&socket_ServerAddr,sizeof(socket_ServerAddr)) < 0){
                ROS_ERROR("bind failed");
                return -1;
            }

            /**************UDP***************/
            /***UDP接收***/

            // ROS_INFO("socket accept successfully!!!!!!");
            memset(&socket_Client, 0,sizeof(socket_Client));
            socket_Client.sin_family = AF_INET;
            socket_Client.sin_port = htons(client_PORT);
            if(inet_pton(AF_INET,client_addr.c_str(),&socket_Client.sin_addr) <= 0){
                ROS_ERROR("Invalid client address/ Address not supported");
                return -1;
            }
            
             socklen_t clientAddrLen = sizeof(socket_Client);



            while (ros::ok())
            {

                if(recvfrom(sockfd, &receive_flag, sizeof(receive_flag), 0, (struct sockaddr *)&socket_Client, &clientAddrLen) < 0){
                    ROS_ERROR("recv failed");
                }

                if(receive_flag.frame_type == 1){
                    
                    std::unique_lock<std::mutex> lock(mtx);
                    receive_flag.frame_type = 2;
                    locationData_send.frame_type = 2;
                    locationData_send.seq = receive_flag.seq;
                    sendto(sockfd, &locationData_send, sizeof(locationData_send), 0, (struct sockaddr *)&socket_Client, sizeof(socket_Client));

                    //test

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

        void run(){
            ros::Rate rate(publish_tf_rate);
            memset(&robot_position, 0,sizeof(robot_position));
            memset(&obstacle_position, 0,sizeof(obstacle_position));
            memset(&robot_nav_info, 0,sizeof(robot_nav_info));
            memset(&obstacle_nav_info, 0,sizeof(obstacle_nav_info));
            memset(&nav_data, 0,sizeof(nav_data));


            ROS_WARN("<--------Transform is running-------->");

            while (ros::ok())
            {
                try
                {
                    tf::StampedTransform TF;
                    tf::Quaternion rotation;
                    listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5.0));   //等待数据(每一个监听器都有一个缓冲器),保证安全接收tf tree(look要对应时间戳)
                    if (!listener.canTransform(target_frame, source_frame, ros::Time(0))) {
                        ROS_WARN("Failed to get transform from %s to %s", source_frame.c_str(), target_frame.c_str());
                        continue;  // 重试
                    }
                    listener.lookupTransform(target_frame,source_frame,ros::Time(0),TF);    //获得两个坐标系的TF关系(必须确保在同一个tree)
                    rotation = TF.getRotation();
                    tf::Matrix3x3 rotation_matrix(rotation);
                    rotation_matrix.getRPY(roll,pitch,yaw);

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

                    if(socket_print_en)
                    {
                        ROS_WARN("Got transform from %s to %s: [%f, %f, %f, %f, %f]",
                            source_frame.c_str(),target_frame.c_str(),
                            TF.getOrigin().x(),
                            TF.getOrigin().y(),
                            yaw,
                            left_distance,
                            right_distance);                 
                    } 
                    
                }
                catch(tf::LookupException &e)
                {
                    ROS_WARN("Failed to get transform");
                }
                ros::spinOnce();
                rate.sleep();
                // ros::spinOnce();
            }  
        }



};

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tf_publish");
    TfPublish Tf_pub;

    
    if(Tf_pub.socketEn_get()){
        std::thread socketServerThread(&TfPublish::socketServer,&Tf_pub);
        socketServerThread.detach();
    }

    std::thread mainThread(&TfPublish::run,&Tf_pub);
    mainThread.detach();
    //Tf_pub.run();

    ros::spin();

    return 0;

}



