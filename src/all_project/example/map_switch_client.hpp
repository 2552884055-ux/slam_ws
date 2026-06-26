// map_switch_client.hpp —— 上位机地图切换客户端接口(header-only,纯 socket,不依赖 ROS)
//
// 用法:#include "map_switch_client.hpp" 后直接调用:
//   SendLoad(host, port, map_id);                 // 进电梯:预加载目标层地图(cmd=1,不重定位)
//   SendReloc(host, port, map_id[, x, y, yaw]);   // 出电梯停稳:启动 laserMapping 并重定位(cmd=2)
//   SendMapSwitchRequest(req, host, port);        // 底层:发一帧请求并等回执
//
// 注意:req_frame / replay_frame 必须与服务端(map_switch_node.hpp)完全一致;
//       服务端每条指令处理完即回执并关闭连接,故每条指令各用一条新连接。

#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstring>
#include <cerrno>
#include <iostream>

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

// 发送一帧请求并等待回执(循环收满 + 超时,抗半包/长等待)。
// timeout_sec:回执等待上限;LOAD/RELOC 服务端内部最长各约 60s,建议 >=70。
inline replay_frame SendMapSwitchRequest(const req_frame& request,
                                         const char* server_addr,
                                         int map_switch_PORT,
                                         int timeout_sec = 90) {
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
    if (inet_pton(AF_INET, server_addr, &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
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
    std::cout << "Request sent (cmd=" << request.cmd
              << " map=" << request.frame_type
              << " seq=" << request.seq << "). Waiting for completion..." << std::endl;

    // 接收回执(循环收满,抗半包)
    char* r = reinterpret_cast<char*>(&reply);
    size_t got = 0;
    while (got < sizeof(reply)) {
        ssize_t n = recv(sockfd, r + got, sizeof(reply) - got, 0);
        if (n < 0) {
            std::cerr << "Receive reply failed: " << strerror(errno) << std::endl;
            reply.result = false; close(sockfd); return reply;
        } else if (n == 0) {
            std::cerr << "Server closed before full reply" << std::endl;
            reply.result = false; close(sockfd); return reply;
        }
        got += static_cast<size_t>(n);
    }

    close(sockfd);
    return reply;
}

// 进电梯时调用:预加载目标层地图(不重定位)
inline bool SendLoad(const char* server_addr, int map_switch_PORT, unsigned long map_id) {
    static unsigned long seq = 0;
    req_frame request = {CMD_LOAD, map_id, ++seq, 0.0f, 0.0f, 0.0f};

    std::cout << "[开始加载地图]" << std::endl;
    // 服务端地址和端口（地图切换的地址、端口）
    replay_frame reply = SendMapSwitchRequest(request, server_addr, map_switch_PORT);
    if (reply.seq == request.seq && reply.result) {
        std::cout << "[地图加载成功]" << std::endl;
        return true;
    } else {
        std::cout << "[地图加载失败]" << std::endl;
        return false;
    }
}

// 出电梯停稳后调用:启动 laserMapping 并重定位
// x/y/yaw 默认 0(初值由服务端 floors.yaml 每层锚点决定);
// 仅当服务端 use_coord_transform=true 时才需填机器人在源图坐标系下的真实位姿。
inline bool SendReloc(const char* server_addr, int map_switch_PORT, unsigned long map_id,
                      float x = 0.0f, float y = 0.0f, float yaw = 0.0f) {
    static unsigned long seq = 1000;
    req_frame request = {CMD_RELOC, map_id, ++seq, x, y, yaw};

    std::cout << "[开始切换地图]" << std::endl;
    // 服务端地址和端口（地图切换的地址、端口）
    replay_frame reply = SendMapSwitchRequest(request, server_addr, map_switch_PORT);
    if (reply.seq == request.seq && reply.result) {
        std::cout << "[地图切换成功]" << std::endl;
        return true;
    } else {
        std::cout << "[地图切换失败]" << std::endl;
        return false;
    }
}
