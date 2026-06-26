#include "map_switch_node.hpp"

/**
 * @class TcpMapSwitchNode
 * @brief 两阶段地图切换;节点由 C++ 直接 fork/execvp(rosrun) 启动并按 PID 精确杀死,
 *        不再依赖 start_*.sh 脚本和 publish_map*.launch。
 */
TcpMapSwitchNode::TcpMapSwitchNode() :
    isRunning_(false), sockfd_(-1), listenfd_(-1),
    laserPid_(-1), current_map_id_(0), prev_map_id_(0)
{
    nh.param<std::string>("server_addr", server_addr_, "192.168.2.100");
    nh.param<int>("map_switch_PORT", map_switch_PORT_, 6050);

    /** python 节点的 PYTHONPATH(原 launch 里给 global_localization 设的)。
     *  在此进程级 setenv 一次(此时还是单线程,安全),子进程 fork 后自动继承,
     *  避免在 fork 与 exec 之间调用非 async-signal-safe 的 setenv。 */
    nh.param<std::string>("python_path", python_path_, "/home/orangepi/.local/lib/python3.8/site-packages");
    if (!python_path_.empty()) {
        const char* old = getenv("PYTHONPATH");
        std::string merged = old ? (python_path_ + ":" + old) : python_path_;
        setenv("PYTHONPATH", merged.c_str(), 1);
    }

    /** RELOC 初值来源开关:true=用 convertBetweenMaps 把源图坐标换算到目标图,
     *  false(默认)=用各层固定电梯口锚点 exit_x/exit_y/exit_yaw */
    nh.param<bool>("use_coord_transform", use_coord_transform_, false);

    /** 开机自动起栈的初始地图ID;<=0 表示用 maps 列表里的第一张 */
    nh.param<int>("initial_map_id", initial_map_id_, 0);

    /** 开机起栈时机器人所在的初始位姿(不一定是 0,0,0;与电梯口锚点 exit_* 相互独立) */
    {
        double v;
        nh.param<double>("initial_x",   v, 0.0); initial_x_   = static_cast<float>(v);
        nh.param<double>("initial_y",   v, 0.0); initial_y_   = static_cast<float>(v);
        nh.param<double>("initial_yaw", v, 0.0); initial_yaw_ = static_cast<float>(v);
    }

    nh.param<double>("load_ready_timeout", load_ready_timeout_, 60.0);
    nh.param<double>("laser_alive_timeout", laser_alive_timeout_, 30.0);
    nh.param<double>("reloc_total_timeout", reloc_total_timeout_, 30.0);

    /** 解析 fast_lio_global 包内 PCD 目录(地图文件所在) */
    std::string pkg_path = ros::package::getPath("fast_lio_global");
    if (pkg_path.empty()) {
        ROS_FATAL("Cannot resolve package 'fast_lio_global'. Is it built/sourced?");
    }
    pkg_pcd_dir_ = pkg_path + "/PCD/";

    /** 加载地图列表 */
    XmlRpc::XmlRpcValue map_list;
    if (nh.getParam("maps", map_list) && map_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < map_list.size(); ++i) {
            XmlRpc::XmlRpcValue map_item = map_list[i];
            MapInfo info;
            info.id = static_cast<int>(map_item["id"]);
            if (map_item.hasMember("pcd"))     info.pcd     = static_cast<std::string>(map_item["pcd"]);
            if (map_item.hasMember("gridmap")) info.gridmap = static_cast<std::string>(map_item["gridmap"]);
            /** RELOC 重定位锚点(出梯位姿);缺省为 0 */
            info.exit_x   = map_item.hasMember("exit_x")   ? static_cast<double>(map_item["exit_x"])   : 0.0;
            info.exit_y   = map_item.hasMember("exit_y")   ? static_cast<double>(map_item["exit_y"])   : 0.0;
            info.exit_yaw = map_item.hasMember("exit_yaw") ? static_cast<double>(map_item["exit_yaw"]) : 0.0;
            /** tx/ty/theta_to_map1 仅供已停用的 convertBetweenMaps 使用,改为可选读取 */
            info.tx_to_map1    = map_item.hasMember("tx_to_map1")    ? static_cast<double>(map_item["tx_to_map1"])    : 0.0;
            info.ty_to_map1    = map_item.hasMember("ty_to_map1")    ? static_cast<double>(map_item["ty_to_map1"])    : 0.0;
            info.theta_to_map1 = map_item.hasMember("theta_to_map1") ? static_cast<double>(map_item["theta_to_map1"]) : 0.0;
            if (info.pcd.empty() || info.gridmap.empty()) {
                ROS_WARN("Map id %lu missing 'pcd'/'gridmap'; LOAD will fail for it.", info.id);
            }
            maps_[info.id] = info;
        }
    } else {
        ROS_FATAL("No maps configured in 'maps' parameter. Exiting.");
    }

    initialpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    ROS_WARN("TcpMapSwitchNode constructed. Listening %s:%d (PCD dir: %s)",
             server_addr_.c_str(), map_switch_PORT_, pkg_pcd_dir_.c_str());
}

TcpMapSwitchNode::~TcpMapSwitchNode()
{
    stop();
}

bool TcpMapSwitchNode::init()
{
    std::lock_guard<std::mutex> lock(mutex_);

    listenfd_ = createAndBindTcpSocket();
    if (listenfd_ == -1) {
        ROS_ERROR("Failed to create and bind TCP socket");
        return false;
    }
    if (listen(listenfd_, 10) < 0) {
        ROS_FATAL("Failed to listen on TCP socket: %s", strerror(errno));
        close(listenfd_);
        listenfd_ = -1;
        return false;
    }
    ROS_WARN("TCP Map Switch Node started. Waiting for commands...");

    /** 开机:对初始地图做一次完整起栈(机器人静止,可直接重定位)。
     *  初始地图由 initial_map_id 指定;<=0 或不存在时回退到列表里的第一张。 */
    if (!maps_.empty() && current_map_id_ == 0) {
        unsigned long first_map = maps_.begin()->first;
        if (initial_map_id_ > 0) {
            if (maps_.find(initial_map_id_) != maps_.end()) {
                first_map = static_cast<unsigned long>(initial_map_id_);
            } else {
                ROS_WARN("initial_map_id=%d not found in maps; falling back to first map id %lu.",
                         initial_map_id_, first_map);
            }
        }
        ROS_WARN("No map active. Bringing up initial map id %lu at initial pose (%.3f, %.3f, %.3f) (load + laser + reloc)...",
                 first_map, initial_x_, initial_y_, initial_yaw_);
        busy_.store(true);  ///< 开机起栈期间占用,拒绝外部指令插队
        std::thread([this, first_map]() {
            struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};
            if (doLoad(first_map)) {
                /** 开机重定位用 initial_x/y/yaw(机器人开机所在位姿),而非电梯口锚点 */
                doReloc(initial_x_, initial_y_, initial_yaw_);
            }
        }).detach();
    }
    return true;
}

void TcpMapSwitchNode::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (listenfd_ != -1 && !isRunning_) {
        isRunning_ = true;
        tcpThread_ = std::thread(&TcpMapSwitchNode::handleTcpData, this, listenfd_);
    }
}

void TcpMapSwitchNode::stop()
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

    /** 退出时杀掉所有由本节点拉起的地图节点 */
    stopCurrentNodes();

    std::lock_guard<std::mutex> lock(pidMutex_);
    if (sockfd_ != -1) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

/**
 * @brief 创建并绑定 TCP 监听 socket
 * @return 成功返回 socket fd,失败返回 -1
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

    /** server_addr 为 "0.0.0.0"/"0" 时监听所有网卡,否则绑定指定 IP */
    if (server_addr_ == "0.0.0.0" || server_addr_ == "0") {
        serverAddr.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, server_addr_.c_str(), &(serverAddr.sin_addr)) <= 0) {
            ROS_FATAL("Failed to convert IP address string to network format");
            close(sockfd);
            return -1;
        }
    }

    /** SO_REUSEADDR:进程重启后能立即重新绑定同一端口(避免 TIME_WAIT 占用) */
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        ROS_FATAL("Failed to bind TCP socket: %s", strerror(errno));
        close(sockfd);
        return -1;
    }
    return sockfd;
}

/**
 * @brief 构造 LOAD 阶段 5 个节点(等价于原 publish_mapN_load.launch)
 */
std::vector<NodeSpec> TcpMapSwitchNode::buildLoadSpecs(const MapInfo& m) const
{
    const std::string pcd_path  = pkg_pcd_dir_ + m.pcd;
    const std::string grid_path = pkg_pcd_dir_ + m.gridmap;

    std::vector<NodeSpec> v;
    // map_publishe: rosrun pcl_ros pcd_to_pointcloud <pcd> 5 _frame_id:=pcd_map cloud_pcd:=pcd_map
    v.push_back({"map_publishe", "pcl_ros", "pcd_to_pointcloud",
                 {pcd_path, "5", "_frame_id:=pcd_map", "cloud_pcd:=pcd_map"}});
    // map_server: rosrun map_server map_server <yaml> map:=grid_map
    v.push_back({"map_server", "map_server", "map_server",
                 {grid_path, "map:=grid_map"}});
    // global_localization.py(PYTHONPATH 已在构造函数进程级设置)
    v.push_back({"global_localization", "fast_lio_global", "global_localization.py", {}});
    // transform_fusion.py
    v.push_back({"transform_fusion", "fast_lio_global", "transform_fusion.py", {}});
    // tf_robot2map(map_switch 自带的 tf_publish)
    v.push_back({"tf_robot2map", "map_switch", "tf_publish", {}});
    return v;
}

/**
 * @brief 构造 laserMapping 节点(等价于原 start_laserMapping.launch)
 */
NodeSpec TcpMapSwitchNode::buildLaserSpec() const
{
    return {"laserMapping", "fast_lio_global", "fastlio_mapping_global", {}};
}

/**
 * @brief fork + execvp("rosrun", pkg, type, args..., __name:=name) 启动一个节点
 * @return 子进程 PID(同时是其进程组 PGID),失败返回 -1
 */
pid_t TcpMapSwitchNode::launchNode(const NodeSpec& spec)
{
    /** 组装 argv:rosrun pkg type [args...] __name:=name */
    std::vector<std::string> argv_s;
    argv_s.push_back("rosrun");
    argv_s.push_back(spec.pkg);
    argv_s.push_back(spec.type);
    for (const auto& a : spec.args) argv_s.push_back(a);
    argv_s.push_back("__name:=" + spec.name);

    std::vector<char*> argv;
    argv.reserve(argv_s.size() + 1);
    for (auto& s : argv_s) argv.push_back(const_cast<char*>(s.c_str()));
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
        ROS_ERROR("fork failed for node %s: %s", spec.name.c_str(), strerror(errno));
        return -1;
    }
    if (pid == 0) {
        /** 子进程:只调用 async-signal-safe 的接口(setpgid/execvp/write/_exit);
         *  环境变量(PYTHONPATH)已在构造函数进程级设好,这里不再 setenv。 */
        setpgid(0, 0);  ///< 独立进程组,便于按组杀死 rosrun 及其子节点
        execvp("rosrun", argv.data());
        /** 走到这里说明 exec 失败 */
        const char msg[] = "execvp rosrun failed\n";
        ssize_t wr = write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)wr;
        _exit(127);
    }

    /** 父进程也设一次 pgid,避免竞争(幂等) */
    setpgid(pid, pid);
    ROS_INFO("Launched node %s (rosrun %s %s) pid=%d",
             spec.name.c_str(), spec.pkg.c_str(), spec.type.c_str(), pid);
    return pid;
}

/**
 * @brief 杀死一个节点进程组:先 SIGINT 优雅退出,超时再 SIGKILL,并回收避免僵尸
 */
void TcpMapSwitchNode::killNode(pid_t pid)
{
    if (pid <= 0) return;

    killpg(pid, SIGINT);  ///< 让 roscpp/rospy 走正常 shutdown(FAST-LIO 不至于异常退出)

    /** 最多等约 5s 优雅退出 */
    for (int i = 0; i < 50; ++i) {
        int st;
        pid_t r = waitpid(pid, &st, WNOHANG);
        if (r == pid || (r < 0 && errno == ECHILD)) {
            ROS_INFO("node pid=%d exited.", pid);
            return;
        }
        usleep(100 * 1000);
    }

    ROS_WARN("node pid=%d did not exit on SIGINT, sending SIGKILL.", pid);
    killpg(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
}

/**
 * @brief 杀掉当前所有 load 节点与 laserMapping
 */
void TcpMapSwitchNode::stopCurrentNodes()
{
    std::vector<pid_t> pids;
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        pids = loadPids_;
        if (laserPid_ > 0) pids.push_back(laserPid_);
        loadPids_.clear();
        laserPid_ = -1;
        current_map_id_ = 0;
    }
    for (pid_t p : pids) killNode(p);
}

/**
 * @brief 直接向 /initialpose 发布一帧初始位姿(map 系)
 */
void TcpMapSwitchNode::publishInitialPoseMsg(float x, float y, float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    initialpose_pub_.publish(msg);
    ROS_WARN("Published /initialpose: x=%f y=%f yaw=%f", x, y, yaw);
}

/**
 * @brief 跨图坐标换算:以 map1 为公共参考系,把 src_id 地图系下的位姿换算到 dst_id 地图系。
 *        各层的 tx/ty/theta_to_map1 描述该层坐标系相对 map1 的刚体变换。
 *        仅当 use_coord_transform_=true 时被 RELOC 调用。
 */
req_frame TcpMapSwitchNode::convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id)
{
    if (src_id == dst_id) return src;

    /** 任意地图系 → map1 系(正变换) */
    auto mapToMap1 = [this](const req_frame& in, unsigned long id)->req_frame {
        req_frame out = in;
        if (id == 1) return out;
        auto it = maps_.find(id);
        if (it == maps_.end()) return out;
        float tx = it->second.tx_to_map1, ty = it->second.ty_to_map1, th = it->second.theta_to_map1;
        out.x = in.x * std::cos(th) - in.y * std::sin(th) + tx;
        out.y = in.x * std::sin(th) + in.y * std::cos(th) + ty;
        out.yaw = in.yaw + th;
        return out;
    };
    /** map1 系 → 任意地图系(逆变换) */
    auto map1ToMap = [this](const req_frame& in, unsigned long id)->req_frame {
        req_frame out = in;
        if (id == 1) return out;
        auto it = maps_.find(id);
        if (it == maps_.end()) return out;
        float tx = it->second.tx_to_map1, ty = it->second.ty_to_map1, th = it->second.theta_to_map1;
        out.x = (in.x - tx) * std::cos(th) + (in.y - ty) * std::sin(th);
        out.y = -(in.x - tx) * std::sin(th) + (in.y - ty) * std::cos(th);
        out.yaw = in.yaw - th;
        return out;
    };

    req_frame in_map1 = (src_id == 1) ? src : mapToMap1(src, src_id);
    req_frame out = (dst_id == 1) ? in_map1 : map1ToMap(in_map1, dst_id);
    out.seq = src.seq;
    out.frame_type = dst_id;
    return out;
}

void TcpMapSwitchNode::sendReplyOnSocket(int client_fd, const replay_frame& reply)
{
    if (client_fd < 0) return;
    ssize_t send_len = send(client_fd, &reply, sizeof(reply), 0);
    if (send_len < 0)
        ROS_ERROR("send reply failed: %s", strerror(errno));
    else
        ROS_WARN("send reply succeeded for seq %lu (result=%d)", reply.seq, reply.result);
    close(client_fd);
}

/**
 * @brief LOAD:停旧节点 → 起目标图预加载节点 → 等就绪。不启动 laserMapping、不重定位。
 */
bool TcpMapSwitchNode::doLoad(unsigned long target_map_id)
{
    auto it = maps_.find(target_map_id);
    if (it == maps_.end() || it->second.pcd.empty() || it->second.gridmap.empty()) {
        ROS_ERROR("doLoad: unknown map id %lu or missing pcd/gridmap.", target_map_id);
        return false;
    }

    /** 记录切换前的源地图(供 use_coord_transform 时坐标换算的 src),再停旧节点 */
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        prev_map_id_ = current_map_id_;
    }

    /** 停掉旧楼层全部节点(含上一次 RELOC 起的 laserMapping) */
    ROS_INFO("doLoad: stopping current nodes...");
    stopCurrentNodes();
    ros::Duration(1.0).sleep();

    /** 启动目标图预加载节点(不含 laserMapping) */
    std::vector<pid_t> pids;
    bool ok = true;
    for (const NodeSpec& spec : buildLoadSpecs(it->second)) {
        pid_t pid = launchNode(spec);
        if (pid < 0) { ok = false; break; }
        pids.push_back(pid);
    }
    if (!ok) {
        ROS_ERROR("doLoad: failed to launch a load node; rolling back.");
        for (pid_t p : pids) killNode(p);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        loadPids_ = pids;
        current_map_id_ = target_map_id;
    }

    ROS_WARN("doLoad: map %lu loading... waiting for global_localization ready (/waiting_for_initial_pose).", target_map_id);
    auto ready = ros::topic::waitForMessage<std_msgs::Bool>(
        "waiting_for_initial_pose", nh, ros::Duration(load_ready_timeout_));
    if (!ready) {
        ROS_ERROR("doLoad: timeout waiting for map %lu to be ready.", target_map_id);
        return false;
    }
    ROS_WARN("doLoad: map %lu loaded and ready (suspended, awaiting RELOC).", target_map_id);
    return true;
}

/**
 * @brief RELOC:起 laserMapping → 等 FAST-LIO 出 /Odometry → 发初值(重试)→ 等重定位完成。
 *        必须在机器人到达新层并静止时调用。
 */
bool TcpMapSwitchNode::doReloc(float x, float y, float yaw)
{
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        if (current_map_id_ == 0) {
            ROS_ERROR("doReloc called but no map loaded. Send LOAD first.");
            return false;
        }
    }

    pid_t pid = launchNode(buildLaserSpec());
    if (pid < 0) {
        ROS_ERROR("doReloc: failed to launch laserMapping.");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        laserPid_ = pid;
    }

    ROS_WARN("doReloc: laserMapping starting... waiting for FAST-LIO (/Odometry).");
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
        "/Odometry", nh, ros::Duration(laser_alive_timeout_));
    if (!odom) {
        ROS_ERROR("doReloc: timeout waiting for FAST-LIO /Odometry.");
        return false;
    }
    ros::Duration(1.0).sleep();  ///< 等首帧 scan / LIO 暂态过去

    /** 重试发初值:单发可能因①与 global_localization 的 wait_for_message 存在竞争而被错过、
     *  ②首帧 scan 尚未到(cur_scan 为空)、③ICP fitness 未达标 而失败;
     *  故循环重发并短超时等待重定位完成标志,直到成功或总超时。 */
    ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < reloc_total_timeout_) {
        publishInitialPoseMsg(x, y, yaw);
        auto flag = ros::topic::waitForMessage<std_msgs::Bool>(
            "map_to_odom_flag", nh, ros::Duration(2.0));
        if (flag && flag->data) {
            ROS_WARN("doReloc: relocalization succeeded.");
            return true;
        }
        ROS_WARN("doReloc: relocalization not done yet, re-publishing initial pose...");
    }
    ROS_ERROR("doReloc: relocalization timeout.");
    return false;
}

/**
 * @brief LOAD 指令入口:在独立 detach 线程里执行 doLoad,完成后把结果回执给 client_fd
 *        (异步处理,避免阻塞 accept 主循环)
 */
void TcpMapSwitchNode::loadMap(unsigned long target_map_id, unsigned long req_seq, int client_fd)
{
    std::thread([this, target_map_id, req_seq, client_fd]() {
        /** 处理结束(任何返回路径)自动释放 busy_,允许下一条指令 */
        struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};

        replay_frame reply;
        reply.seq = req_seq;
        reply.result = false;

        if (maps_.find(target_map_id) == maps_.end()) {
            ROS_WARN("loadMap: unknown target map id: %lu", target_map_id);
            sendReplyOnSocket(client_fd, reply);
            return;
        }
        reply.result = doLoad(target_map_id);
        sendReplyOnSocket(client_fd, reply);
    }).detach();
}

/**
 * @brief RELOC 指令入口:在独立 detach 线程里执行 doReloc,完成后回执给 client_fd
 *        重定位初值取该层配置锚点,请求里的 x/y/yaw 仅用于日志
 */
void TcpMapSwitchNode::relocalize(unsigned long target_map_id, unsigned long req_seq, float x, float y, float yaw, int client_fd)
{
    std::thread([this, target_map_id, req_seq, x, y, yaw, client_fd]() {
        /** 处理结束(任何返回路径)自动释放 busy_,允许下一条指令 */
        struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};

        replay_frame reply;
        reply.seq = req_seq;
        reply.result = false;

        unsigned long loaded, src;
        {
            std::lock_guard<std::mutex> lock(pidMutex_);
            loaded = current_map_id_;
            src = prev_map_id_;
        }
        if (loaded == 0) {
            ROS_WARN("relocalize: no map loaded; send LOAD first.");
            sendReplyOnSocket(client_fd, reply);
            return;
        }
        if (loaded != target_map_id)
            ROS_WARN("relocalize: loaded map(%lu) != RELOC target(%lu). Proceeding with loaded map.", loaded, target_map_id);

        /** 重定位初值二选一:
         *  - use_coord_transform_=true :把上位机请求里的 x/y/yaw(源图 prev_map_id_ 坐标系)
         *    用 convertBetweenMaps 换算到目标图坐标系,作为初值;
         *  - use_coord_transform_=false(默认):用该层固定电梯口锚点 exit_x/exit_y/exit_yaw,
         *    忽略请求位姿(电梯流程更稳)。 */
        float ax = 0.0f, ay = 0.0f, ayaw = 0.0f;
        if (use_coord_transform_) {
            req_frame in;  in.x = x;  in.y = y;  in.yaw = yaw;
            req_frame out = convertBetweenMaps(in, src, loaded);
            ax = out.x;  ay = out.y;  ayaw = out.yaw;
            ROS_WARN("relocalize: [coord_transform] map %lu <- src %lu: request (%.3f,%.3f,%.3f) -> initial (%.3f,%.3f,%.3f)",
                     loaded, src, x, y, yaw, ax, ay, ayaw);
        } else {
            auto it = maps_.find(loaded);
            if (it != maps_.end()) { ax = it->second.exit_x; ay = it->second.exit_y; ayaw = it->second.exit_yaw; }
            ROS_WARN("relocalize: [fixed_anchor] map %lu anchor=(%.3f, %.3f, %.3f); request pose (%.3f, %.3f, %.3f) ignored.",
                     loaded, ax, ay, ayaw, x, y, yaw);
        }

        reply.result = doReloc(ax, ay, ayaw);
        sendReplyOnSocket(client_fd, reply);
    }).detach();
}

/**
 * @brief TCP 监听线程:循环 accept 连接,读取一帧 req_frame 并按 cmd 分发到 LOAD/RELOC。
 *        每条指令处理完由对应入口回执并关闭该连接(故每条指令一条新连接)。
 */
void TcpMapSwitchNode::handleTcpData(int listenfd) {
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    ROS_WARN("handleTcpData thread started.");

    while (ros::ok()) {
        int client_fd = accept(listenfd, (struct sockaddr*)&clientAddr, &clientAddrLen);
        if (client_fd < 0) {
            /** stop() 关闭 listenfd 会令 accept 返回错误,据 isRunning_ 区分正常退出与真异常 */
            if (!isRunning_) {
                ROS_INFO("Listener closed, exiting handleTcpData loop.");
                break;
            }
            ROS_ERROR("accept failed: %s", strerror(errno));
            continue;
        }

        /** 接收超时,防止客户端连上却不发数据时永久阻塞 */
        struct timeval tv;
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        /** 循环接收直到凑满定长帧(TCP 是字节流,40字节可能分多次到达);
         *  出错/对端关闭/超时则丢弃该连接。 */
        req_frame request;
        char* buf = reinterpret_cast<char*>(&request);
        size_t got = 0;
        bool recv_ok = true;
        while (got < sizeof(request)) {
            ssize_t n = recv(client_fd, buf + got, sizeof(request) - got, 0);
            if (n < 0) {
                ROS_ERROR("recv failed from client: %s", strerror(errno));
                recv_ok = false; break;
            } else if (n == 0) {
                ROS_WARN("Client closed before full frame (%zu/%zu bytes).", got, sizeof(request));
                recv_ok = false; break;
            }
            got += static_cast<size_t>(n);
        }
        if (!recv_ok) { close(client_fd); continue; }

        ROS_WARN("Received request: cmd=%lu frame_type=%lu seq=%lu x=%f y=%f yaw=%f",
                 request.cmd, request.frame_type, request.seq, request.x, request.y, request.yaw);

        /** 命令合法性 + 并发互斥:已有切换在进行时拒绝新指令,避免两线程同时起停节点而竞争。
         *  LOAD/RELOC 成功占用 busy_ 后,由对应 worker 线程在完成时释放(见 loadMap/relocalize)。 */
        if (request.cmd == CMD_LOAD || request.cmd == CMD_RELOC) {
            if (busy_.exchange(true)) {
                ROS_WARN("Busy: another switch in progress, rejecting cmd=%lu seq=%lu.", request.cmd, request.seq);
                replay_frame reply; reply.seq = request.seq; reply.result = false;
                sendReplyOnSocket(client_fd, reply);
                continue;
            }
        }

        /** 按命令字分发;client_fd 交由入口异步回执后关闭,busy_ 也由 worker 释放 */
        switch (request.cmd) {
            case CMD_LOAD:
                ROS_INFO("CMD_LOAD: pre-loading map %lu (no relocalization).", request.frame_type);
                loadMap(request.frame_type, request.seq, client_fd);
                break;
            case CMD_RELOC:
                ROS_INFO("CMD_RELOC: relocalizing map %lu with pose (%f, %f, %f).",
                         request.frame_type, request.x, request.y, request.yaw);
                relocalize(request.frame_type, request.seq, request.x, request.y, request.yaw, client_fd);
                break;
            default: {
                ROS_ERROR("Unknown cmd=%lu. Expect 1=LOAD or 2=RELOC.", request.cmd);
                replay_frame reply;
                reply.seq = request.seq;
                reply.result = false;
                sendReplyOnSocket(client_fd, reply);
                break;
            }
        }
    }

    ROS_WARN("Exiting handleTcpData thread.");
}
