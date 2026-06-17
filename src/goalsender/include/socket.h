#ifndef SOCKET_H
#define SOCKET_H

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include "network/frame.hpp"
#include "network/moduleObj.hpp"

// 网络通讯相关常量
#define UDP_PORT 8111
#define HTTP_PORT 8080
#define SERVER_IP "192.168.31.70"

// 前向声明
struct FrameBuilder;
class udpTool;
class netWorkBase;
struct sockaddr_in;

// 回调函数类型
typedef void (*udp_callback_t)(netWorkBase*, void*, int, sockaddr_in);

// 全局变量声明（UDP对象）
extern std::unique_ptr<udpTool> g_udp;
extern std::mutex g_udp_mutex;

// 全局函数声明
int setup_udp_socket(int port);
void sendCommand(int sockfd, const FrameBuilder& builder, const char* ip, uint16_t port);
void start_udp_receiver(udp_callback_t callback);
void stop_udp_receiver();

#endif // SOCKET_H
