#ifndef __FRAME_DATA__HPP
#define __FRAME_DATA__HPP

#define FRAME_MAX_SIZE (400 + 24u)

// 帧结构定义
struct frame {
    unsigned int source;
    unsigned int dest;
    unsigned int type;
    unsigned int len_data;
    unsigned int checksum;
    unsigned int reserve;
    char data[];
};

struct mydata {
    int seq;
    int eof;
    int sub_obj;
    int type;
};

// 帧类型定义 (frameType)
enum FRAME_TYPE {
    REQUEST = 0,          // 请求帧/命令
    RESPONSE,             // 数据上报，底下各部分上传到服务器
    COMNMAND,            // 控制命令，包含开启、关闭等各种控制命令
    INQUIRY,             // 状态查询
    BACK                 // 状态反馈
};

// 源地址定义 (source) / 目标地址定义 (dest)
enum LOCATION {
    HTTP_SERVER = 0,       // HTTP服务器，服务器向下发送请求
    VISION_SYSTEM,         // 视觉系统
    ROBOT_SYSTEM,          // 机器人系统
    METER_SYSTEM,          // 仪表系统
    GAS_SYSTEM,            // 气体检测系统
    TEMP_SYSTEM,           // 温度检测系统
    VOICEACTOR_SYSTEM      // 语音串口系统
};

// 子对象定义 (subObj) - 视觉系统
enum SUBOBJ {
    FACE = 0,              // 人脸检测
    METER,                 // 仪表检测
    HARDHAT,               // 安全帽检测
    INVASION,              // 人员入侵检测
    FLAME,                 // 火焰检测
    SMOKE                  // 吸烟检测
};

// 子对象定义 (subObj) - 气体系统
#define SUBOBJ_YANWU1      0   // 烟雾1
#define SUBOBJ_YANWU2      1   // 烟雾2

// 子对象定义 (subObj) - 仪表系统
#define SUBOBJ_METER       1   // 仪表数据

// 子对象定义 (subObj) - 温度系统
#define SUBOBJ_TEMPERATURE 0   // 温度数据

// 子对象定义 (subObj) - 机器人系统
#define SUBOBJ_ROBOT       0   // 机器人数据

// 子对象定义 (subObj) 控制数据开关
#define DATA_STOP          5   // 关闭
#define DATA_BEGIN         6   // 开启

// 数据标志定义 (hasData)
#define HAS_DATA_NODATA    0   // 无数据
#define HAS_DATA_OPEN      1   // 开启
#define HAS_DATA_CLOSE     2   // 关闭

// data1定义
#define VISION_DATA_CLOSE  0   // 关闭
#define VISION_DATA_OPEN   1   // 开启

// 子对象定义 (subObj) - 机器人系统导航
#define SUBOBJ_ROBOT_NAV_XY    10   // 导航目标点 XY 坐标
#define SUBOBJ_ROBOT_NAV_ZYAW  11   // 导航目标点 Z 坐标和朝向角


#endif
