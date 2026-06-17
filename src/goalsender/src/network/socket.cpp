#include "socket.h"
#include "data_type.h"
#include <iostream>
#include <mutex>
#include "network/tcpUdp.hpp"

// 全局UDP对象（供外部访问）
std::unique_ptr<udpTool> g_udp;
std::mutex g_udp_mutex;

int setup_udp_socket(int port) {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    ipPort selfIp{"", static_cast<unsigned short>(port)};
    g_udp = std::make_unique<udpTool>(selfIp, MODULE_TYPE::HTTPSERVER_OBJ);
    if (g_udp->createNet(FRAME_MAX_SIZE) != ERROR_NUM::SUCCESS) {
        std::cerr << "Failed to initialize UDP module on port " << port << std::endl;
        g_udp.reset();
        return -1;
    }
    return 0;
}

void sendCommand(int, const FrameBuilder& builder, const char* ip, uint16_t port) {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    if (!g_udp) {
        std::cerr << "UDP module not initialized, cannot send command." << std::endl;
        return;
    }

    std::vector<char> buffer;
    builder.build_to_buffer(buffer);
    ipPort dest{std::string(ip), port};
    g_udp->sendData(buffer.data(), buffer.size(), dest);
}

void start_udp_receiver(void (*callback)(netWorkBase*, void*, int, sockaddr_in)) {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    if (g_udp) {
        g_udp->startRecvThread(callback);
        std::cout << "UDP receive thread started" << std::endl;
    }
}

void stop_udp_receiver() {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    if (g_udp) {
        g_udp->endRecvThread();
        std::cout << "UDP receive thread stopped" << std::endl;
    }
}
