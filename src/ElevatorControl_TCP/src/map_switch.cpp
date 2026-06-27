#include "map_switch.hpp"
#include <sys/time.h>
#include <cerrno>
#include <string>

// 内部函数:建立 TCP 连接,发送 40 字节请求帧,阻塞等待 16 字节回执
// 循环收发防半包;收发超时均为 timeout_sec 秒(LOAD/RELOC 服务端内部最长各约 60s,默认给 90s)
static replay_frame sendRequest(const req_frame& request, const char* server_addr,
                                int port, int timeout_sec = 90) {
    replay_frame reply = {false, 0};

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return reply;
    }

    struct timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, server_addr, &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(sockfd);
        return reply;
    }

    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        close(sockfd);
        return reply;
    }

    // 发送请求(循环发满)
    const char* p = reinterpret_cast<const char*>(&request);
    size_t sent = 0;
    while (sent < sizeof(request)) {
        ssize_t n = send(sockfd, p + sent, sizeof(request) - sent, 0);
        if (n <= 0) {
            std::cerr << "Send failed" << std::endl;
            close(sockfd);
            return reply;
        }
        sent += static_cast<size_t>(n);
    }
    std::cout << "Request sent successfully" << std::endl;
    std::cout << "Waiting for the map switch to be completed..." << std::endl;

    // 接收回执(循环收满,抗半包)
    char* r = reinterpret_cast<char*>(&reply);
    size_t got = 0;
    while (got < sizeof(reply)) {
        ssize_t n = recv(sockfd, r + got, sizeof(reply) - got, 0);
        if (n < 0) {
            std::cerr << "Receive reply failed: " << strerror(errno) << std::endl;
            reply.result = false;
            close(sockfd);
            return reply;
        }
        if (n == 0) {
            std::cerr << "Server closed the connection" << std::endl;
            reply.result = false;
            close(sockfd);
            return reply;
        }
        got += static_cast<size_t>(n);
    }

    close(sockfd);
    return reply;
}

// 进电梯时调用(非阻塞):起后台线程发 CMD_LOAD,立即返回;地图加载完成时触发 cb(ok)
// 服务端收到后:停旧层节点 → 起目标层预加载节点 → 等地图就绪 → 回执
// 回执到达(地图已加载完成)时 ok=true,此后可安全调用 SendReloc
void SendLoad(const char* server_addr, int port, unsigned long map_id,
              MapSwitchResultCb cb) {
    static std::atomic<unsigned long> seq{0};
    std::string addr(server_addr ? server_addr : "");
    unsigned long s = ++seq;

    std::thread([addr, port, map_id, s, cb]() {
        req_frame request = {CMD_LOAD, map_id, s, 0.0f, 0.0f, 0.0f};
        std::cout << "[开始加载地图]" << std::endl;
        replay_frame reply = sendRequest(request, addr.c_str(), port);
        bool ok = (reply.seq == request.seq && reply.result);
        std::cout << (ok ? "[地图加载成功]" : "[地图加载失败]") << std::endl;
        if (cb) cb(ok);
    }).detach();
}

// 出电梯停稳后调用(阻塞):发 CMD_RELOC,等服务端启动 laserMapping + 重定位成功后返回
// 服务端收到后:起 laserMapping → 等 /Odometry → 发 /initialpose → 等重定位完成 → 回执
// x/y/yaw:机器人在源图坐标系下的位姿;服务端根据 use_coord_transform 决定是否使用
bool SendReloc(const char* server_addr, int port, unsigned long map_id,
               float x, float y, float yaw) {
    static std::atomic<unsigned long> seq{1000};
    req_frame request = {CMD_RELOC, map_id, ++seq, x, y, yaw};

    std::cout << "[开始切换地图]" << std::endl;
    replay_frame reply = sendRequest(request, server_addr, port);

    bool ok = (reply.seq == request.seq && reply.result);
    std::cout << (ok ? "[地图切换成功]" : "[地图切换失败]") << std::endl;
    return ok;
}
