// sr71_can_node.cpp
// SR71(纳雷科技)毫米波雷达 CAN 驱动: 读 SocketCAN 报文 -> 解析为 RadarScan -> 发布。
//
// ⚠️ 重要: CAN 报文的具体字节布局(报文ID、各字段位偏移/缩放)因固件而异,
//    必须对照纳雷提供的《SR71 CAN 通信协议文档》填写 parseFrame()。
//    本文件给出可运行的 SocketCAN 读取框架 + 解析占位, 把 TODO 处补全即可。
//
// 仅 Linux(SocketCAN)。Windows 开发机不编译(见 CMakeLists)。
// 使用前: sudo ip link set can0 type can bitrate 500000 ; sudo ip link set up can0

#include <ros/ros.h>
#include "radar_ego_velocity/RadarScan.h"
#include "radar_ego_velocity/RadarTarget.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

class SR71CanDriver
{
public:
    SR71CanDriver(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param<std::string>("can_interface", can_if_, "can0");
        pnh.param<std::string>("frame_id",      frame_id_, "radar");
        pnh.param<std::string>("scan_topic",    scan_topic_, "/radar_scan");
        // SR71 CAN 报文ID(示例, 以协议文档为准)
        pnh.param<int>("id_header", id_header_, 0x70A);   // 帧头: 含目标个数
        pnh.param<int>("id_target", id_target_, 0x70B);   // 目标: 含 A/R/V

        pub_ = nh.advertise<radar_ego_velocity::RadarScan>(scan_topic_, 50);
        if (!openCan()) { ros::shutdown(); return; }
    }

    ~SR71CanDriver() { if (sock_ >= 0) close(sock_); }

    void spin()
    {
        radar_ego_velocity::RadarScan scan;
        int expected = 0;       // 帧头声明的目标数
        while (ros::ok())
        {
            struct can_frame frame;
            int nbytes = read(sock_, &frame, sizeof(frame));
            if (nbytes < (int)sizeof(frame)) { ros::spinOnce(); continue; }

            if ((int)frame.can_id == id_header_)
            {
                // 上一帧凑齐就发布, 然后开新的一帧
                if (!scan.targets.empty())
                {
                    scan.header.stamp = ros::Time::now();
                    scan.header.frame_id = frame_id_;
                    pub_.publish(scan);
                }
                scan.targets.clear();
                expected = parseHeader(frame);   // TODO: 从帧头取目标数
            }
            else if ((int)frame.can_id == id_target_)
            {
                radar_ego_velocity::RadarTarget tgt;
                if (parseTarget(frame, tgt)) scan.targets.push_back(tgt);
                // 收满一帧也可直接发布
                if (expected > 0 && (int)scan.targets.size() >= expected)
                {
                    scan.header.stamp = ros::Time::now();
                    scan.header.frame_id = frame_id_;
                    pub_.publish(scan);
                    scan.targets.clear();
                    expected = 0;
                }
            }
            ros::spinOnce();
        }
    }

private:
    bool openCan()
    {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) { ROS_ERROR("[sr71] socket() failed"); return false; }
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_if_.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        { ROS_ERROR("[sr71] interface %s not found", can_if_.c_str()); return false; }
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        { ROS_ERROR("[sr71] bind() failed"); return false; }
        ROS_INFO("[sr71] CAN %s opened, header=0x%X target=0x%X", can_if_.c_str(), id_header_, id_target_);
        return true;
    }

    // ===== TODO: 按《SR71 CAN 协议文档》实现下面两个解析函数 =====

    // 从帧头报文解析"本帧目标个数"
    int parseHeader(const struct can_frame &f)
    {
        // 示例: 目标数可能在某个字节, 如 f.data[0]
        // return (int)f.data[0];
        (void)f;
        return 0;   // TODO: 替换为真实解析
    }

    // 从目标报文解析 方位角/距离/多普勒。返回false表示无效目标。
    bool parseTarget(const struct can_frame &f, radar_ego_velocity::RadarTarget &t)
    {
        // ⚠️ 下面是占位/示意, 字节位置与缩放系数务必按协议文档替换!
        // 典型车规雷达打包方式举例(仅示意):
        //   uint16 raw_range = (f.data[1] << 8) | f.data[2];
        //   int16  raw_speed = (int16)((f.data[3] << 8) | f.data[4]);
        //   int16  raw_angle = (int16)((f.data[5] << 8) | f.data[6]);
        //   t.range   = raw_range * 0.01f;                 // m,  缩放按文档
        //   t.doppler = raw_speed * 0.01f;                 // m/s
        //   t.azimuth = raw_angle * 0.01f * M_PI / 180.0f; // deg->rad
        //   t.elevation = 0.0f;                            // SR71 object模式仅方位角=>2D
        //   t.id = f.data[0];
        //   return true;
        (void)f; (void)t;
        return false;   // TODO: 替换为真实解析
    }

    ros::Publisher pub_;
    std::string can_if_, frame_id_, scan_topic_;
    int id_header_, id_target_;
    int sock_ = -1;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sr71_can_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SR71CanDriver drv(nh, pnh);
    drv.spin();
    return 0;
}
