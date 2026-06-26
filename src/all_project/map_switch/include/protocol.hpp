// protocol.hpp —— 地图切换协议帧与数据结构(纯数据定义,无逻辑)

#pragma once

#include <string>
#include <vector>

/** TCP 请求命令字(req_frame.cmd) */
enum MapSwitchCmd {
    CMD_LOAD  = 1,   ///< 进电梯:预加载目标地图(不含 laserMapping,不重定位)
    CMD_RELOC = 2    ///< 出电梯:启动 laserMapping 并发布初始位姿触发重定位
};

/**
 * @struct req_frame
 * @brief TCP 客户端请求帧(定长,二进制;须与上位机一致)
 */
struct req_frame {
    unsigned long cmd;          ///< 1=LOAD 2=RELOC
    unsigned long frame_type;   ///< 目标地图ID
    unsigned long seq;          ///< 请求序列号
    float x;                    ///< 初始位姿X(RELOC;电梯流程填0)
    float y;                    ///< 初始位姿Y
    float yaw;                  ///< 初始位姿yaw(弧度)
};

/**
 * @struct replay_frame
 * @brief TCP 服务端应答帧
 */
struct replay_frame {
    bool result;                ///< true成功 / false失败
    unsigned long seq;          ///< 对应请求序列号
};

/**
 * @struct MapInfo
 * @brief 地图信息(由 config/floors.yaml 提供)
 */
struct MapInfo {
    unsigned long id;           ///< 地图唯一ID
    std::string pcd;            ///< 三维点云地图文件名(位于 fast_lio_global/PCD/)
    std::string gridmap;        ///< 二维栅格地图 yaml 文件名
    float exit_x;               ///< RELOC 重定位锚点(出梯位姿,map系)X,默认0
    float exit_y;               ///< RELOC 重定位锚点 Y,默认0
    float exit_yaw;             ///< RELOC 重定位锚点 yaw(弧度),默认0
    float tx_to_map1;           ///< 相对map1的X平移量(convertBetweenMaps 用)
    float ty_to_map1;           ///< 相对map1的Y平移量
    float theta_to_map1;        ///< 相对map1的旋转角(弧度)
};

/**
 * @struct NodeSpec
 * @brief 一个 ROS 节点的启动描述(供 NodeLauncher 用 rosrun 启动)
 */
struct NodeSpec {
    std::string name;                   ///< 节点名(__name:=)
    std::string pkg;                    ///< rosrun 包名
    std::string type;                   ///< rosrun 可执行/脚本名
    std::vector<std::string> args;      ///< 透传给节点的位置参数/remap/私有参数
};
