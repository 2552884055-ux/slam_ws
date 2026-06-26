#include "tcp_server.hpp"

#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

TcpServer::TcpServer() : listenfd_(-1), running_(false) {}

TcpServer::~TcpServer() { stop(); }

/**
 * @brief 创建并绑定 TCP 监听 socket
 */
int TcpServer::createAndBindTcpSocket(const std::string& addr, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ROS_FATAL("Failed to create TCP socket: %s", strerror(errno));
        return -1;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    /** "0.0.0.0"/"0" 监听所有网卡,否则绑定指定 IP */
    if (addr == "0.0.0.0" || addr == "0") {
        serverAddr.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, addr.c_str(), &(serverAddr.sin_addr)) <= 0) {
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

bool TcpServer::bindAndListen(const std::string& addr, int port)
{
    listenfd_ = createAndBindTcpSocket(addr, port);
    if (listenfd_ == -1) {
        ROS_ERROR("TcpServer: failed to create/bind socket");
        return false;
    }
    if (listen(listenfd_, 10) < 0) {
        ROS_FATAL("TcpServer: listen failed: %s", strerror(errno));
        close(listenfd_);
        listenfd_ = -1;
        return false;
    }
    ROS_WARN("TcpServer listening on %s:%d", addr.c_str(), port);
    return true;
}

void TcpServer::start(Handler handler)
{
    if (listenfd_ == -1 || running_) return;
    handler_ = std::move(handler);
    running_ = true;
    thread_ = std::thread(&TcpServer::acceptLoop, this);
}

void TcpServer::stop()
{
    if (!running_.exchange(false)) {
        /** 未运行:仅关闭可能已创建的监听 socket */
        if (listenfd_ != -1) { close(listenfd_); listenfd_ = -1; }
        return;
    }
    /** 关闭监听 socket 使 accept 返回错误而退出循环 */
    if (listenfd_ != -1) { close(listenfd_); listenfd_ = -1; }
    if (thread_.joinable()) thread_.join();
}

void TcpServer::acceptLoop()
{
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    ROS_WARN("TcpServer accept loop started.");
    while (running_ && ros::ok()) {
        int client_fd = accept(listenfd_, (struct sockaddr*)&clientAddr, &clientAddrLen);
        if (client_fd < 0) {
            if (!running_) {
                ROS_INFO("TcpServer: listener closed, exiting accept loop.");
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

        /** 循环接收直到凑满定长帧(TCP 是字节流,可能分多次到达) */
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

        /** 交给业务回调(回调负责异步处理 + sendReply + 关闭 fd) */
        if (handler_) handler_(request, client_fd);
        else { close(client_fd); }
    }
    ROS_WARN("TcpServer accept loop exited.");
}

void TcpServer::sendReply(int client_fd, const replay_frame& reply)
{
    if (client_fd < 0) return;
    ssize_t send_len = send(client_fd, &reply, sizeof(reply), 0);
    if (send_len < 0)
        ROS_ERROR("send reply failed: %s", strerror(errno));
    else
        ROS_WARN("send reply succeeded for seq %lu (result=%d)", reply.seq, reply.result);
    close(client_fd);
}
