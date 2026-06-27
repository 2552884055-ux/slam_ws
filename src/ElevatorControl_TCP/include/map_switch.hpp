#pragma once

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>


#define map1 1 
#define map2 2

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

