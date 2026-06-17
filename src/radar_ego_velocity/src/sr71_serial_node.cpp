// sr71_serial_node.cpp
// SR71(纳雷科技)毫米波雷达 串口(TTL/UART)驱动:
//   读串口字节流 -> 帧同步 -> 解析为 RadarScan -> 发布 /radar_scan。
//
// ⚠️ 重要: 串口帧的具体格式(帧头/长度/字段位偏移/缩放/校验)因固件而异,
//    必须对照纳雷《SR71 串口通信协议文档》填写 FRAME_HEADER / parseFrame()。
//    本文件给出可运行的串口读取 + 帧同步状态机 + 解析占位, 补全 TODO 即可。
//
// 用 POSIX termios 实现, 适用于 Linux 实机(/dev/ttyUSB0 等)。Windows 不编译(见 CMakeLists)。

#include <ros/ros.h>
#include "radar_ego_velocity/RadarScan.h"
#include "radar_ego_velocity/RadarTarget.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <cstdint>

// ===== TODO: 按协议文档填写帧格式常量 =====
static const uint8_t  FRAME_HEAD0   = 0xAA;   // 帧头字节0 (示例)
static const uint8_t  FRAME_HEAD1   = 0xAA;   // 帧头字节1 (示例)
static const uint8_t  FRAME_TAIL0   = 0x55;   // 帧尾字节0 (示例, 若用帧尾)
static const uint8_t  FRAME_TAIL1   = 0x55;   // 帧尾字节1 (示例)
static const size_t   TARGET_BYTES  = 8;      // 每个目标占用字节数 (示例)

static speed_t toBaud(int b)
{
    switch (b) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

class SR71SerialDriver
{
public:
    SR71SerialDriver(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param<std::string>("port",       port_,      "/dev/ttyUSB0");
        pnh.param<int>("baudrate",           baud_,      115200);
        pnh.param<std::string>("frame_id",   frame_id_,  "radar");
        pnh.param<std::string>("scan_topic", scan_topic_,"/radar_scan");

        pub_ = nh.advertise<radar_ego_velocity::RadarScan>(scan_topic_, 50);
        if (!openPort()) { ros::shutdown(); return; }
    }

    ~SR71SerialDriver() { if (fd_ >= 0) close(fd_); }

    void spin()
    {
        uint8_t buf[512];
        while (ros::ok())
        {
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0)
            {
                acc_.insert(acc_.end(), buf, buf + n);   // 累积到缓冲
                parseBuffer();                            // 帧同步 + 解析
            }
            ros::spinOnce();
        }
    }

private:
    bool openPort()
    {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) { ROS_ERROR("[sr71] open %s failed", port_.c_str()); return false; }
        fcntl(fd_, F_SETFL, 0);                 // 阻塞读

        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd_, &tty) != 0) { ROS_ERROR("[sr71] tcgetattr failed"); return false; }

        speed_t spd = toBaud(baud_);
        cfsetospeed(&tty, spd);
        cfsetispeed(&tty, spd);

        tty.c_cflag |= (CLOCAL | CREAD);        // 本地连接, 使能接收
        tty.c_cflag &= ~CSIZE;  tty.c_cflag |= CS8;   // 8 数据位
        tty.c_cflag &= ~PARENB;                 // 无校验 (按协议改)
        tty.c_cflag &= ~CSTOPB;                 // 1 停止位
        tty.c_cflag &= ~CRTSCTS;                // 无硬件流控
        cfmakeraw(&tty);                        // 原始模式
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;                    // 0.1s 读超时

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ROS_ERROR("[sr71] tcsetattr failed"); return false; }
        tcflush(fd_, TCIOFLUSH);
        ROS_INFO("[sr71] serial %s @ %d opened", port_.c_str(), baud_);
        return true;
    }

    // 帧同步: 从累积缓冲里找完整帧并解析
    void parseBuffer()
    {
        // 简单状态机: 找帧头 -> 读长度 -> 凑齐一帧 -> 解析 -> 移除已消费字节
        while (acc_.size() >= 4)
        {
            // 1) 对齐帧头
            if (!(acc_[0] == FRAME_HEAD0 && acc_[1] == FRAME_HEAD1))
            {
                acc_.erase(acc_.begin());       // 丢一个字节继续找
                continue;
            }
            // 2) TODO: 从帧头后取"帧长 / 目标数". 示例: 目标数在第3字节
            size_t num_targets = acc_[2];                 // TODO: 按协议替换
            size_t frame_len   = 4 + num_targets * TARGET_BYTES + 2; // 头(4)+载荷+帧尾(2), 按协议改
            if (acc_.size() < frame_len) return;          // 数据还没收齐, 等下次

            // 3) TODO: 校验(checksum/CRC). 失败则丢弃该帧头继续
            // if (!checksumOK(&acc_[0], frame_len)) { acc_.erase(acc_.begin()); continue; }

            // 4) 解析载荷
            radar_ego_velocity::RadarScan scan;
            scan.header.stamp = ros::Time::now();
            scan.header.frame_id = frame_id_;
            for (size_t i = 0; i < num_targets; ++i)
            {
                const uint8_t *p = &acc_[4 + i * TARGET_BYTES];
                radar_ego_velocity::RadarTarget tgt;
                if (parseTarget(p, tgt)) scan.targets.push_back(tgt);
            }
            if (!scan.targets.empty()) pub_.publish(scan);

            // 5) 消费掉这一帧
            acc_.erase(acc_.begin(), acc_.begin() + frame_len);
        }
    }

    // ===== TODO: 按《SR71 串口协议文档》解析单个目标 =====
    // p 指向该目标的 TARGET_BYTES 字节。返回false表示无效目标。
    bool parseTarget(const uint8_t *p, radar_ego_velocity::RadarTarget &t)
    {
        // ⚠️ 占位/示意, 字节顺序、缩放系数务必按协议替换!
        //   uint16 raw_range = (p[0] << 8) | p[1];
        //   int16  raw_speed = (int16)((p[2] << 8) | p[3]);  // 多普勒
        //   int16  raw_angle = (int16)((p[4] << 8) | p[5]);  // 方位角
        //   t.range     = raw_range * 0.01f;                 // m
        //   t.doppler   = raw_speed * 0.01f;                 // m/s
        //   t.azimuth   = raw_angle * 0.01f * M_PI / 180.0;  // deg->rad
        //   t.elevation = 0.0f;                              // SR71 object模式只有方位角=>2D
        //   t.id = p[6];
        //   return true;
        (void)p; (void)t;
        return false;   // TODO: 替换为真实解析
    }

    ros::Publisher pub_;
    std::string port_, frame_id_, scan_topic_;
    int baud_;
    int fd_ = -1;
    std::vector<uint8_t> acc_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sr71_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SR71SerialDriver drv(nh, pnh);
    drv.spin();
    return 0;
}
