// 两阶段地图切换协议(上位机侧)
// 与 all_project 包内 map_switch 节点(服务端)通信,实现楼层切换:
//   进电梯:发 CMD_LOAD(非阻塞预加载地图,乘梯期间后台完成)
//   出电梯:发 CMD_RELOC(阻塞重定位,等定位成功后返回)

#ifndef MAP_SWITCH_H
#define MAP_SWITCH_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <thread>
#include <functional>

// 命令字:进电梯预加载用 CMD_LOAD,出电梯重定位用 CMD_RELOC
#define CMD_LOAD  1
#define CMD_RELOC 2

// 目标地图ID(对应服务端 floors.yaml 的 id)
#define map1 1
#define map2 2
#define map3 3
#define map4 4
#define map5 5

// 请求帧(须与服务端 req_frame 完全一致,40字节定长)
struct req_frame {
    unsigned long cmd;          // 命令字:1=CMD_LOAD  2=CMD_RELOC
    unsigned long frame_type;   // 目标地图ID
    unsigned long seq;          // 请求序列号(用于匹配回执)
    float x;                    // 机器人在源图坐标系下的位姿X
    float y;                    // 位姿Y
    float yaw;                  // 位姿yaw(弧度)
};

// 回执帧(16字节定长)
struct replay_frame {
    bool result;                // true=成功 / false=失败
    unsigned long seq;          // 对应请求的序列号
};

// LOAD 完成回调类型:ok=true 表示地图预加载成功
using MapSwitchResultCb = std::function<void(bool ok)>;

// 进电梯时调用(非阻塞):预加载目标层地图,立即返回;地图加载完成时触发回调 cb(ok)
void SendLoad(const char* server_addr, int port, unsigned long map_id,
              MapSwitchResultCb cb = nullptr);

// 出电梯停稳后调用(阻塞):启动 laserMapping 并重定位,等成功后返回
// x/y/yaw:机器人在源图坐标系下的位姿(始终传入;
//   服务端 use_coord_transform=false 时不使用,由 floors.yaml 各层锚点决定;
//   服务端 use_coord_transform=true  时将其换算到目标图作为重定位初值)
bool SendReloc(const char* server_addr, int port, unsigned long map_id,
               float x, float y, float yaw);

#endif
