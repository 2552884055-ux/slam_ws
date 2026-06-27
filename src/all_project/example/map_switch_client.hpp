// map_switch_client.hpp —— 上位机地图切换客户端(header-only,纯 socket,不依赖 ROS)
//
// 对外两个函数:
//
//   void SendLoad (addr, port, map_id, cb);          // 进电梯:预加载地图(cmd=1)。【非阻塞】,完成回调 cb(ok)
//   bool SendReloc(addr, port, map_id, x, y, yaw);   // 出电梯:重定位(cmd=2)。【阻塞】,返回是否成功
//
//   典型用法:主循环里 SendLoad 非阻塞发出 → 回调里把 load_ok 置位 →
//             主循环检测到 load_ok 后,再进入 SendReloc。
//   ⚠ SendLoad 的 cb 在【后台线程】执行,回调里碰共享数据请用原子量/加锁。
//   x/y/yaw 默认 0;仅当服务端 use_coord_transform=true 时才作为源图位姿被使用。
//
// 注意:req_frame / replay_frame 必须与服务端(map_switch_node.hpp)完全一致。

#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <string>
#include <atomic>
#include <thread>
#include <functional>

// ===== 与服务端一致的协议帧 =====
enum MapSwitchCmd { CMD_LOAD = 1, CMD_RELOC = 2 };

struct req_frame {
    unsigned long cmd;          // 1=LOAD 2=RELOC
    unsigned long frame_type;   // 目标地图ID
    unsigned long seq;          // 请求序列号
    float x;                    // 初始位姿X(RELOC;电梯流程填0)
    float y;                    // 初始位姿Y
    float yaw;                  // 初始位姿yaw(弧度)
};

struct replay_frame {
    bool result;                // true成功 / false失败
    unsigned long seq;          // 对应请求序列号
};

/** LOAD 结果回调:ok=true 表示加载成功 */
using MapSwitchResultCb = std::function<void(bool ok)>;

// ===== 内部实现(调用方无需直接使用)=====
namespace mapswitch_detail {

// 发一帧请求并等回执(循环收满 + 超时,抗半包)。
inline replay_frame sendRequest(const req_frame& request, const std::string& server_addr,
                                int map_switch_PORT, int timeout_sec = 90) {
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
    serverAddr.sin_port = htons(map_switch_PORT);
    if (inet_pton(AF_INET, server_addr.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << server_addr << std::endl;
        close(sockfd);
        return reply;
    }

    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection failed: " << strerror(errno) << std::endl;
        close(sockfd);
        return reply;
    }

    // 发送请求(循环发满)
    const char* p = reinterpret_cast<const char*>(&request);
    size_t sent = 0;
    while (sent < sizeof(request)) {
        ssize_t n = send(sockfd, p + sent, sizeof(request) - sent, 0);
        if (n <= 0) {
            std::cerr << "Send failed: " << strerror(errno) << std::endl;
            close(sockfd);
            return reply;
        }
        sent += static_cast<size_t>(n);
    }

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
            std::cerr << "Server closed before full reply" << std::endl;
            reply.result = false;
            close(sockfd);
            return reply;
        }
        got += static_cast<size_t>(n);
    }

    close(sockfd);
    return reply;
}

}  // namespace mapswitch_detail

// ===== 对外两个函数 =====

// 进电梯时调用(非阻塞):预加载目标层地图(不重定位)。立即返回,完成时回调 cb(ok)。
inline void SendLoad(const char* server_addr, int map_switch_PORT, unsigned long map_id,
                     MapSwitchResultCb cb = nullptr) {
    static std::atomic<unsigned long> seq{0};
    std::string addr(server_addr ? server_addr : "");
    unsigned long s = ++seq;

    std::thread([addr, map_switch_PORT, map_id, s, cb]() {
        req_frame request = {CMD_LOAD, map_id, s, 0.0f, 0.0f, 0.0f};
        std::cout << "[开始加载地图]" << std::endl;
        replay_frame reply = mapswitch_detail::sendRequest(request, addr, map_switch_PORT);
        bool ok = (reply.seq == request.seq && reply.result);
        std::cout << (ok ? "[地图加载成功]" : "[地图加载失败]") << std::endl;
        if (cb) cb(ok);
    }).detach();
}

// 出电梯停稳后调用(阻塞):启动 laserMapping 并重定位,等回执后返回是否成功。
// 前提:LOAD 已成功(地图加载完成)。
inline bool SendReloc(const char* server_addr, int map_switch_PORT, unsigned long map_id,
                      float x = 0.0f, float y = 0.0f, float yaw = 0.0f) {
    static std::atomic<unsigned long> seq{1000};
    req_frame request = {CMD_RELOC, map_id, ++seq, x, y, yaw};

    std::cout << "[开始切换地图]" << std::endl;
    replay_frame reply = mapswitch_detail::sendRequest(request,
                             server_addr ? server_addr : "", map_switch_PORT);

    bool ok = (reply.seq == request.seq && reply.result);
    std::cout << (ok ? "[地图切换成功]" : "[地图切换失败]") << std::endl;
    return ok;
}
