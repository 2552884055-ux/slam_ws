// ============================================================================
//  模拟地图切换服务端 (map_switch mock)
//
//  用途：在没有机器人侧 all_project/map_switch 节点的情况下，模拟其 TCP 服务，
//        供 ElevatorControl_TCP/RTU 的 SendLoad / SendReloc 联调测试。
//
//  协议(与 map_switch.hpp 严格一致)：
//    客户端连上后发送 40 字节 req_frame，服务端处理后回 16 字节 replay_frame。
//      req_frame   : cmd(8) frame_type(8) seq(8) x(4) y(4) yaw(4) + 尾部填充 = 40B
//      replay_frame: result(1) + 填充(7) + seq(8) = 16B
//    cmd: 1=CMD_LOAD(预加载地图)  2=CMD_RELOC(重定位)
//
//  每个请求是一条独立 TCP 连接，故采用「每连接一线程」处理。
//
//  编译: g++ -std=c++11 map_switch_sim.cpp -pthread -o map_switch_sim
//  运行: ./map_switch_sim 6050            # [port] [load耗时s] [reloc耗时s]
// ============================================================================

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

#define CMD_LOAD  1
#define CMD_RELOC 2

// 必须与 map_switch.hpp 中的定义逐字段一致(同机同 ABI，sizeof 自动对齐)
struct req_frame {
    unsigned long cmd;
    unsigned long frame_type;
    unsigned long seq;
    float x;
    float y;
    float yaw;
};
struct replay_frame {
    bool result;
    unsigned long seq;
};

static std::atomic<bool> g_running{true};
static int g_listen_fd = -1;

static void onSignal(int) {
    g_running = false;
    if (g_listen_fd != -1) { close(g_listen_fd); g_listen_fd = -1; }
}

static int g_load_sec  = 15;   // 模拟地图加载耗时
static int g_reloc_sec = 5;   // 模拟重定位耗时

// 收满 n 字节，抗半包
static bool recvAll(int fd, void* buf, size_t n) {
    char* p = static_cast<char*>(buf);
    size_t got = 0;
    while (got < n) {
        ssize_t r = recv(fd, p + got, n - got, 0);
        if (r <= 0) return false;
        got += static_cast<size_t>(r);
    }
    return true;
}

// 发满 n 字节
static bool sendAll(int fd, const void* buf, size_t n) {
    const char* p = static_cast<const char*>(buf);
    size_t sent = 0;
    while (sent < n) {
        ssize_t s = send(fd, p + sent, n - sent, 0);
        if (s <= 0) return false;
        sent += static_cast<size_t>(s);
    }
    return true;
}

static void handleClient(int fd) {
    req_frame req;
    std::memset(&req, 0, sizeof(req));
    if (!recvAll(fd, &req, sizeof(req))) {
        std::cerr << "[map_switch] 读取请求失败/连接中断" << std::endl;
        close(fd);
        return;
    }

    const char* name = (req.cmd == CMD_LOAD) ? "CMD_LOAD(预加载)"
                     : (req.cmd == CMD_RELOC) ? "CMD_RELOC(重定位)" : "未知命令";
    std::cout << "[map_switch] 收到请求: " << name
              << " 目标图=" << req.frame_type
              << " seq=" << req.seq
              << " pose=(" << req.x << "," << req.y << "," << req.yaw << ")" << std::endl;

    int delay = (req.cmd == CMD_RELOC) ? g_reloc_sec : g_load_sec;
    std::cout << "[map_switch] 模拟处理中(" << delay << "s)..." << std::endl;
    for (int i = 0; i < delay && g_running; ++i)
        std::this_thread::sleep_for(std::chrono::seconds(1));

    replay_frame reply;
    std::memset(&reply, 0, sizeof(reply));
    reply.result = true;          // 始终返回成功(可按需改成偶发失败做异常测试)
    reply.seq    = req.seq;       // 回执 seq 必须与请求一致，否则客户端判失败

    if (sendAll(fd, &reply, sizeof(reply)))
        std::cout << "[map_switch] 已回执: result=成功 seq=" << reply.seq << std::endl;
    else
        std::cerr << "[map_switch] 发送回执失败" << std::endl;

    close(fd);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);  // 客户端提前断开时不让进程被 SIGPIPE 杀掉

    int port = (argc > 1) ? std::atoi(argv[1]) : 6050;
    if (argc > 2) g_load_sec  = std::atoi(argv[2]);
    if (argc > 3) g_reloc_sec = std::atoi(argv[3]);

    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0) { std::cerr << "创建 socket 失败\n"; return 1; }

    int opt = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(g_listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "绑定端口 " << port << " 失败 (可能被占用)\n";
        close(g_listen_fd);
        return 1;
    }
    if (listen(g_listen_fd, 8) < 0) {
        std::cerr << "listen 失败\n";
        close(g_listen_fd);
        return 1;
    }

    std::cout << "[启动] 地图切换模拟服务端，监听 0.0.0.0:" << port
              << " (load=" << g_load_sec << "s, reloc=" << g_reloc_sec << "s)\n"
              << "[map_switch] 等待请求..." << std::endl;

    while (g_running) {
        sockaddr_in cli;
        socklen_t len = sizeof(cli);
        int fd = accept(g_listen_fd, reinterpret_cast<sockaddr*>(&cli), &len);
        if (fd < 0) {
            if (!g_running) break;
            continue;
        }
        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &cli.sin_addr, ip, sizeof(ip));
        std::cout << "[map_switch] 新连接来自 " << ip << std::endl;
        std::thread(handleClient, fd).detach();
    }

    if (g_listen_fd != -1) close(g_listen_fd);
    std::cout << "\n[退出] 地图切换模拟服务端已停止。" << std::endl;
    return 0;
}
