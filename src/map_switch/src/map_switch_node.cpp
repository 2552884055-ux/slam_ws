#include "map_switch_node.hpp"

/**
 * @class TcpMapSwitchNode
 * @brief TCP地图切换节点，支持多地图切换、初始位姿发布和坐标转换
 *
 * 该节点通过TCP服务接收外部请求，根据请求切换地图，
 * 发布初始位姿，并支持不同地图之间的坐标转换。
 */
TcpMapSwitchNode::TcpMapSwitchNode() :
    sockfd_(-1), listenfd_(-1), isRunning_(false), currentMapPid(-1), current_map_id_(0)
{
    /** 读取TCP服务器参数 */
    nh.param<std::string>("server_addr", server_addr_, "192.168.2.100"); 
    nh.param<int>("map_switch_PORT", map_switch_PORT_, 6050);

    /** 读取控制脚本路径参数 */
    nh.param<std::string>("stop_script", stop_script_path_, "/home/orangepi/slam_ws/src/map_switch/scripts/stop_current_map.sh");
    nh.param<std::string>("initial_pose_script", initial_pose_script_path_, "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/scripts/publish_initial_pose.py");

    /** 加载地图列表参数 */
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
            maps_[info.id] = info;  ///< 保存地图信息到maps字典
        }
    } else {
        ROS_FATAL("No maps configured in 'maps' parameter. Exiting."); ///< 未配置地图直接退出
    }

    ROS_WARN("TcpMapSwitchNode constructed. Listening %s:%d", server_addr_.c_str(), map_switch_PORT_);
}

/**
 * @brief 析构函数，停止节点并释放资源
 */
TcpMapSwitchNode::~TcpMapSwitchNode()
{
    stop();
}

/**
 * @brief 初始化TCP服务并启动默认地图（若未指定）
 * @return 初始化是否成功
 */
bool TcpMapSwitchNode::init()
{
    std::lock_guard<std::mutex> lock(mutex_);

    /** 创建并绑定TCP套接字 */
    listenfd_ = createAndBindTcpSocket();
    if (listenfd_ == -1)
    {
        ROS_ERROR("Failed to create and bind TCP socket");
        return false;
    }

    /** 开始监听TCP端口 */
    if (listen(listenfd_, 10) < 0)
    {
        ROS_FATAL("Failed to listen on TCP socket: %s", strerror(errno));
        close(listenfd_);
        listenfd_ = -1;
        return false;
    }
    ROS_WARN("TCP Map Switch Node started. Waiting for commands...");

    /** 若未指定当前地图，启动默认地图 */
    if (!maps_.empty() && current_map_id_ == 0)
    {
        auto first_map_it = maps_.begin();
        ROS_WARN("No map active. Launching default map id %lu ...", first_map_it->first);
        launchMap(first_map_it->first, 0, 0, 0, 0, -1);
    }

    return true;
}

/**
 * @brief 启动TCP监听线程
 */
void TcpMapSwitchNode::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (listenfd_ != -1 && !isRunning_)
    {
        isRunning_ = true;
        tcpThread_ = std::thread(&TcpMapSwitchNode::handleTcpData, this, listenfd_);
    }
}

/**
 * @brief 停止节点，关闭TCP连接和线程
 */
void TcpMapSwitchNode::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!isRunning_) return;
        isRunning_ = false;

        /** 关闭监听socket */
        if (listenfd_ != -1) {
            close(listenfd_);
            listenfd_ = -1;
        }
    }

    /** 等待线程结束 */
    if (tcpThread_.joinable())
        tcpThread_.join();

    /** 关闭客户端socket */
    std::lock_guard<std::mutex> lock(pidMutex_);
    if (sockfd_ != -1) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

/**
 * @brief 创建并绑定TCP套接字
 * @return 成功返回socket描述符，失败返回-1
 */
int TcpMapSwitchNode::createAndBindTcpSocket()
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

    /** 支持绑定所有IP或指定IP */
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

    /** 绑定TCP端口 */
    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        ROS_FATAL("Failed to bind TCP socket: %s", strerror(errno));
        close(sockfd);
        return -1;
    }

    return sockfd;
}

/**
 * @brief 启动外部脚本
 * @param scriptPath 脚本路径
 * @return 脚本进程PID，失败返回-1
 */
pid_t TcpMapSwitchNode::startScript(const std::string& scriptPath) {
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

/**
 * @brief 执行停止地图脚本
 * @return 执行成功返回true，失败返回false
 */
bool TcpMapSwitchNode::executeStopScript() {
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

/**
 * @brief 发布初始位姿
 * @param x 初始位姿x
 * @param y 初始位姿y
 * @param yaw 初始位姿yaw
 */
void TcpMapSwitchNode::PublishInitialPose(float x, float y, float yaw)
{
    std::stringstream ss;
    ss << initial_pose_script_path_ << " " << x << " " << y << " " << 0 << " " << yaw << " " << 0 << " " << 0;
    std::string cmd = ss.str();
    int result = std::system(cmd.c_str());
    if (result != 0) {
        ROS_ERROR("Failed to execute initial pose script: %s", cmd.c_str());
    }
}

/**
 * @brief 坐标系转换函数，在不同地图之间转换坐标
 * @param src 原坐标
 * @param src_id 源地图ID
 * @param dst_id 目标地图ID
 * @return 转换后的坐标
 */
req_frame TcpMapSwitchNode::convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id)
{
    if (src_id == dst_id) return src;

    // src -> map1
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

    // map1 -> dst
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

/**
 * @brief 向客户端发送回复
 * @param client_fd 客户端socket
 * @param reply 回复内容
 */
void TcpMapSwitchNode::sendReplyOnSocket(int client_fd, const replay_frame& reply)
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

/**
 * @brief 启动指定地图，并处理初始位姿和重定位
 * @param target_map_id 目标地图ID
 * @param req_seq 请求序列号
 * @param x 初始位姿x
 * @param y 初始位姿y
 * @param yaw 初始位姿yaw
 * @param client_fd 客户端socket
 */
void TcpMapSwitchNode::launchMap(unsigned long target_map_id, unsigned long req_seq, float x, float y, float yaw, int client_fd)
{
    auto switchTask = [this, target_map_id, req_seq, x, y, yaw, client_fd]() {
        replay_frame reply;
        reply.seq = req_seq;
        reply.result = false;

        /** 检查目标地图是否存在 */
        if (maps_.find(target_map_id) == maps_.end()) {
            ROS_WARN("Unknown target map id: %lu", target_map_id);
            sendReplyOnSocket(client_fd, reply);
            return;
        }

        /** 停止当前地图 */
        if (current_map_id_ != 0) {
            std::lock_guard<std::mutex> lock(pidMutex_);
            if (currentMapPid != -1) {
                ROS_INFO("Stopping current map (pid=%d) via stop script...", currentMapPid);
            } else {
                ROS_INFO("No known current map process; executing stop script to ensure no map runs.");
            }
            this->executeStopScript();
            currentMapPid = -1;
            current_map_id_ = 0;
        }

        /** 启动目标地图脚本 */
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

        /** 等待初始位姿发布标志 */
        std_msgs::Bool::ConstPtr initial_pose_flag = ros::topic::waitForMessage<std_msgs::Bool>("waiting_for_initial_pose", nh);

        if (initial_pose_flag && initial_pose_flag->data) {
            ROS_WARN("Received initial pose flag, publishing initial pose...");
            PublishInitialPose(x, y, yaw);
        } else {
            ros::Duration(5.0).sleep();
            ROS_WARN("Has been waiting for 5 seconds, publishing initial pose...");
            PublishInitialPose(x, y, yaw);
        }

        /** 等待重定位完成 */
        std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);

        ROS_WARN("Map switch succeeded...");

        if (relocalization_msg && relocalization_msg->data) {
            reply.result = true;
        } else {
            reply.result = false;
        }

        sendReplyOnSocket(client_fd, reply);

    };

    std::thread th(switchTask);
    th.detach();
}

/**
 * @brief TCP数据接收线程处理函数
 * @param listenfd TCP监听socket
 */
void TcpMapSwitchNode::handleTcpData(int listenfd) {
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

        /** 设置接收超时 */
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

        ROS_WARN("Received request: frame_type=%lu seq=%lu x=%f y=%f yaw=%f",
                 request.frame_type, request.seq, request.x, request.y, request.yaw);

        /** 坐标转换处理 */
        req_frame new_request = request;
        {
            std::lock_guard<std::mutex> lock(pidMutex_);
            if (current_map_id_ == 0) {
                new_request = request;
                current_map_id_ = request.frame_type;
                ROS_INFO("Unknown current map; adopting request's frame as current: %lu", current_map_id_);
            } else if (current_map_id_ == request.frame_type) {
                new_request = request;
                ROS_WARN("Already at target map %lu. No map transformation needed.", request.frame_type);
            } else {
                ROS_INFO("Converting coordinates from current_map(%lu) to requested map(%lu)...", current_map_id_, request.frame_type);
                new_request = convertBetweenMaps(request, current_map_id_, request.frame_type);
            }
        }

        //ROS_WARN("Old Coordinates: x = %f, y = %f, yaw = %f", request.x, request.y, request.yaw);
        //ROS_WARN("New Coordinates: x = %f, y = %f, yaw = %f (target map %lu)", new_request.x, new_request.y, new_request.yaw, new_request.frame_type);

        // /** 启动目标地图 */
        // launchMap(new_request.frame_type, new_request.seq, new_request.x, new_request.y, new_request.yaw, client_fd);
        // {
        //     std::lock_guard<std::mutex> lock(pidMutex_);
        //     sockfd_ = -1;
        // }

        /** 启动目标地图 */
        // 由于在肇庆测试时，机器狗在电梯里坐标漂移严重，如果使用实时坐标会出现严重问题，所以肇庆实际应用时改为0,0,0定死
        launchMap(new_request.frame_type, new_request.seq, 0, 0, 0, client_fd);
        {
            std::lock_guard<std::mutex> lock(pidMutex_);
            sockfd_ = -1;
        }
    }

    ROS_WARN("Exiting handleTcpData thread.");
}
