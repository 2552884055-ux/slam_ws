#ifndef MAP_SWITCH_H
#define MAP_SWITCH_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// frame_type=10时启动地图1，frame_type=20时启动地图2
#define map1 10
#define map2 20

// 请求结构体
struct req_frame
{
    unsigned long frame_type;
    unsigned long seq;
    float x;
    float y;
    float yaw;
};

// 回复结构体
struct replay_frame
{
    bool result;
    unsigned long seq;
};

replay_frame SendMapSwitchRequest(const req_frame& request, const char* server_addr, int map_switch_PORT);

#endif    