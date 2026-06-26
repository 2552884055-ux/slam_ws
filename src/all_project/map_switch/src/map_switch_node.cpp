#include "map_switch_node.hpp"

#include <cstdlib>
#include <cstring>

/**
 * @class MapSwitchNode
 * @brief 两阶段地图切换编排。TCP 收发交给 TcpServer,进程起停交给 NodeLauncher,
 *        本类只负责配置、两阶段流程与并发互斥。
 */
MapSwitchNode::MapSwitchNode() :
    laserPid_(-1), current_map_id_(0), prev_map_id_(0)
{
    nh.param<std::string>("server_addr", server_addr_, "192.168.2.100");
    nh.param<int>("map_switch_PORT", map_switch_PORT_, 6050);

    /** python 节点的 PYTHONPATH:此处进程级 setenv 一次(单线程,安全),子进程 fork 后继承,
     *  避免在 fork 与 exec 之间调用非 async-signal-safe 的 setenv。 */
    nh.param<std::string>("python_path", python_path_, "/home/orangepi/.local/lib/python3.8/site-packages");
    if (!python_path_.empty()) {
        const char* old = getenv("PYTHONPATH");
        std::string merged = old ? (python_path_ + ":" + old) : python_path_;
        setenv("PYTHONPATH", merged.c_str(), 1);
    }

    nh.param<bool>("use_coord_transform", use_coord_transform_, false);
    nh.param<int>("initial_map_id", initial_map_id_, 0);
    {
        double v;
        nh.param<double>("initial_x",   v, 0.0); initial_x_   = static_cast<float>(v);
        nh.param<double>("initial_y",   v, 0.0); initial_y_   = static_cast<float>(v);
        nh.param<double>("initial_yaw", v, 0.0); initial_yaw_ = static_cast<float>(v);
    }
    nh.param<double>("load_ready_timeout",  load_ready_timeout_,  60.0);
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
            info.exit_x   = map_item.hasMember("exit_x")   ? static_cast<double>(map_item["exit_x"])   : 0.0;
            info.exit_y   = map_item.hasMember("exit_y")   ? static_cast<double>(map_item["exit_y"])   : 0.0;
            info.exit_yaw = map_item.hasMember("exit_yaw") ? static_cast<double>(map_item["exit_yaw"]) : 0.0;
            info.tx_to_map1    = map_item.hasMember("tx_to_map1")    ? static_cast<double>(map_item["tx_to_map1"])    : 0.0;
            info.ty_to_map1    = map_item.hasMember("ty_to_map1")    ? static_cast<double>(map_item["ty_to_map1"])    : 0.0;
            info.theta_to_map1 = map_item.hasMember("theta_to_map1") ? static_cast<double>(map_item["theta_to_map1"]) : 0.0;
            if (info.pcd.empty() || info.gridmap.empty())
                ROS_WARN("Map id %lu missing 'pcd'/'gridmap'; LOAD will fail for it.", info.id);
            maps_[info.id] = info;
        }
    } else {
        ROS_FATAL("No maps configured in 'maps' parameter. Exiting.");
    }

    initialpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    ROS_WARN("MapSwitchNode constructed. Listening %s:%d (PCD dir: %s)",
             server_addr_.c_str(), map_switch_PORT_, pkg_pcd_dir_.c_str());
}

MapSwitchNode::~MapSwitchNode() { stop(); }

bool MapSwitchNode::init()
{
    if (!server_.bindAndListen(server_addr_, map_switch_PORT_)) {
        ROS_ERROR("Failed to bind/listen TCP server");
        return false;
    }
    server_.start([this](const req_frame& req, int fd) { onRequest(req, fd); });
    ROS_WARN("TCP Map Switch Node started. Waiting for commands...");

    /** 开机:对初始地图做一次完整起栈(机器人静止,可直接重定位)。
     *  初始地图由 initial_map_id 指定;<=0 或不存在时回退到列表里的第一张。 */
    if (!maps_.empty() && current_map_id_ == 0) {
        unsigned long first_map = maps_.begin()->first;
        if (initial_map_id_ > 0) {
            if (maps_.find(initial_map_id_) != maps_.end())
                first_map = static_cast<unsigned long>(initial_map_id_);
            else
                ROS_WARN("initial_map_id=%d not found in maps; falling back to first map id %lu.",
                         initial_map_id_, first_map);
        }
        ROS_WARN("No map active. Bringing up initial map id %lu at initial pose (%.3f, %.3f, %.3f)...",
                 first_map, initial_x_, initial_y_, initial_yaw_);
        busy_.store(true);  ///< 开机起栈期间占用,拒绝外部指令插队
        std::thread([this, first_map]() {
            struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};
            if (doLoad(first_map))
                doReloc(initial_x_, initial_y_, initial_yaw_);
        }).detach();
    }
    return true;
}

void MapSwitchNode::stop()
{
    server_.stop();
    stopCurrentNodes();
}

/**
 * @brief TcpServer 回调:命令合法性 + 并发互斥 + 按 cmd 分发
 */
void MapSwitchNode::onRequest(const req_frame& request, int client_fd)
{
    /** 已有切换在进行时拒绝新指令,避免两线程同时起停节点而竞争。
     *  busy_ 占用后由对应 worker 线程在完成时释放(见 loadMap/relocalize)。 */
    if (request.cmd == CMD_LOAD || request.cmd == CMD_RELOC) {
        if (busy_.exchange(true)) {
            ROS_WARN("Busy: another switch in progress, rejecting cmd=%lu seq=%lu.", request.cmd, request.seq);
            replay_frame reply; reply.seq = request.seq; reply.result = false;
            TcpServer::sendReply(client_fd, reply);
            return;
        }
    }

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
            replay_frame reply; reply.seq = request.seq; reply.result = false;
            TcpServer::sendReply(client_fd, reply);
            break;
        }
    }
}

/**
 * @brief 构造 LOAD 阶段 5 个节点
 */
std::vector<NodeSpec> MapSwitchNode::buildLoadSpecs(const MapInfo& m) const
{
    const std::string pcd_path  = pkg_pcd_dir_ + m.pcd;
    const std::string grid_path = pkg_pcd_dir_ + m.gridmap;

    std::vector<NodeSpec> v;
    v.push_back({"map_publishe", "pcl_ros", "pcd_to_pointcloud",
                 {pcd_path, "5", "_frame_id:=pcd_map", "cloud_pcd:=pcd_map"}});
    v.push_back({"map_server", "map_server", "map_server",
                 {grid_path, "map:=grid_map"}});
    v.push_back({"global_localization", "fast_lio_global", "global_localization.py", {}});
    v.push_back({"transform_fusion", "fast_lio_global", "transform_fusion.py", {}});
    v.push_back({"tf_robot2map", "all_project", "tf_publish", {}});
    return v;
}

NodeSpec MapSwitchNode::buildLaserSpec() const
{
    return {"laserMapping", "fast_lio_global", "fastlio_mapping_global", {}};
}

void MapSwitchNode::stopCurrentNodes()
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
    for (pid_t p : pids) launcher_.killNode(p);
}

void MapSwitchNode::publishInitialPoseMsg(float x, float y, float yaw)
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
 *        仅当 use_coord_transform_=true 时被 RELOC 调用。
 */
req_frame MapSwitchNode::convertBetweenMaps(const req_frame& src, unsigned long src_id, unsigned long dst_id)
{
    if (src_id == dst_id) return src;

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

/**
 * @brief LOAD:停旧节点 → 起目标图预加载节点 → 等就绪。不启动 laserMapping、不重定位。
 */
bool MapSwitchNode::doLoad(unsigned long target_map_id)
{
    auto it = maps_.find(target_map_id);
    if (it == maps_.end() || it->second.pcd.empty() || it->second.gridmap.empty()) {
        ROS_ERROR("doLoad: unknown map id %lu or missing pcd/gridmap.", target_map_id);
        return false;
    }

    /** 记录切换前的源地图(供坐标换算 src),再停旧节点 */
    { std::lock_guard<std::mutex> lock(pidMutex_); prev_map_id_ = current_map_id_; }

    ROS_INFO("doLoad: stopping current nodes...");
    stopCurrentNodes();
    ros::Duration(1.0).sleep();

    std::vector<pid_t> pids;
    bool ok = true;
    for (const NodeSpec& spec : buildLoadSpecs(it->second)) {
        pid_t pid = launcher_.launch(spec);
        if (pid < 0) { ok = false; break; }
        pids.push_back(pid);
    }
    if (!ok) {
        ROS_ERROR("doLoad: failed to launch a load node; rolling back.");
        for (pid_t p : pids) launcher_.killNode(p);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        loadPids_ = pids;
        current_map_id_ = target_map_id;
    }

    ROS_WARN("doLoad: map %lu loading... waiting for /waiting_for_initial_pose.", target_map_id);
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
bool MapSwitchNode::doReloc(float x, float y, float yaw)
{
    {
        std::lock_guard<std::mutex> lock(pidMutex_);
        if (current_map_id_ == 0) {
            ROS_ERROR("doReloc called but no map loaded. Send LOAD first.");
            return false;
        }
    }

    pid_t pid = launcher_.launch(buildLaserSpec());
    if (pid < 0) {
        ROS_ERROR("doReloc: failed to launch laserMapping.");
        return false;
    }
    { std::lock_guard<std::mutex> lock(pidMutex_); laserPid_ = pid; }

    ROS_WARN("doReloc: laserMapping starting... waiting for FAST-LIO (/Odometry).");
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
        "/Odometry", nh, ros::Duration(laser_alive_timeout_));
    if (!odom) {
        ROS_ERROR("doReloc: timeout waiting for FAST-LIO /Odometry.");
        return false;
    }
    ros::Duration(1.0).sleep();  ///< 等首帧 scan / LIO 暂态过去

    /** 重试发初值:单发可能因①与 global_localization 的 wait_for_message 竞争被错过、
     *  ②首帧 scan 未到、③ICP fitness 未达标 而失败;故循环重发直到成功或总超时。 */
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
 * @brief LOAD 指令入口:独立 detach 线程执行 doLoad,完成后回执 client_fd
 */
void MapSwitchNode::loadMap(unsigned long target_map_id, unsigned long req_seq, int client_fd)
{
    std::thread([this, target_map_id, req_seq, client_fd]() {
        struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};

        replay_frame reply; reply.seq = req_seq; reply.result = false;
        if (maps_.find(target_map_id) == maps_.end()) {
            ROS_WARN("loadMap: unknown target map id: %lu", target_map_id);
            TcpServer::sendReply(client_fd, reply);
            return;
        }
        reply.result = doLoad(target_map_id);
        TcpServer::sendReply(client_fd, reply);
    }).detach();
}

/**
 * @brief RELOC 指令入口:独立 detach 线程执行 doReloc,完成后回执 client_fd
 */
void MapSwitchNode::relocalize(unsigned long target_map_id, unsigned long req_seq,
                               float x, float y, float yaw, int client_fd)
{
    std::thread([this, target_map_id, req_seq, x, y, yaw, client_fd]() {
        struct BusyGuard { std::atomic<bool>* b; ~BusyGuard() { b->store(false); } } _bg{&busy_};

        replay_frame reply; reply.seq = req_seq; reply.result = false;

        unsigned long loaded, src;
        {
            std::lock_guard<std::mutex> lock(pidMutex_);
            loaded = current_map_id_;
            src = prev_map_id_;
        }
        if (loaded == 0) {
            ROS_WARN("relocalize: no map loaded; send LOAD first.");
            TcpServer::sendReply(client_fd, reply);
            return;
        }
        if (loaded != target_map_id)
            ROS_WARN("relocalize: loaded map(%lu) != RELOC target(%lu). Proceeding with loaded map.", loaded, target_map_id);

        /** 重定位初值二选一:
         *  - use_coord_transform_=true :把请求 x/y/yaw(源图 prev_map_id_ 系)换算到目标图;
         *  - false(默认):用该层固定电梯口锚点 exit_*,忽略请求位姿。 */
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
        TcpServer::sendReply(client_fd, reply);
    }).detach();
}
