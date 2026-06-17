// radar_ego_velocity_node.cpp
// 订阅雷达一帧目标(RadarScan), 用 RANSAC + 最小二乘估计雷达系自速度,
// 发布 geometry_msgs/TwistStamped (/radar_velocity), 供 FAST-LIO 紧耦合融合。
//
// 原理: 对静止目标, 其多普勒径向速度 V 完全由载体运动产生:
//        V_i = doppler_sign * ( d_i^T * v_S )
//   其中 d_i 为该目标视线单位向量, v_S 为雷达系自速度。
//   收集多个不同角度的静止目标, 用 RANSAC 剔除动目标, 最小二乘解出 v_S。

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <cmath>

#include "radar_ego_velocity/RadarScan.h"

class RadarEgoVelocity
{
public:
    RadarEgoVelocity(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param<std::string>("radar_scan_topic", scan_topic_, "/radar_scan");
        pnh.param<std::string>("output_topic",     out_topic_,  "/radar_velocity");
        pnh.param<bool>("mode_2d",          mode_2d_,        true);   // 2D雷达(只有方位角)只解(vx,vy)
        pnh.param<double>("doppler_sign",   doppler_sign_,   1.0);    // 多普勒符号约定: +1或-1, 按驱动调
        pnh.param<int>("ransac_iter",       ransac_iter_,    100);
        pnh.param<double>("inlier_thresh",  inlier_thresh_,  0.15);   // 径向速度残差门限 (m/s)
        pnh.param<int>("min_inliers",       min_inliers_,    5);      // 内点数下限, 不足则丢弃该帧
        pnh.param<double>("min_range",      min_range_,      0.5);    // 近距盲区(m)
        pnh.param<double>("max_range",      max_range_,      20.0);
        pnh.param<double>("max_speed",      max_speed_,      5.0);    // 自速度上限(m/s), 超过视为异常

        sub_ = nh.subscribe(scan_topic_, 50, &RadarEgoVelocity::scanCallback, this);
        pub_ = nh.advertise<geometry_msgs::TwistStamped>(out_topic_, 50);

        ROS_INFO("[radar_ego_vel] mode_2d=%d, in=%s, out=%s",
                 (int)mode_2d_, scan_topic_.c_str(), out_topic_.c_str());
    }

private:
    // 视线单位向量: d = [cosE cosA, cosE sinA, sinE]
    static Eigen::Vector3d los(double azimuth, double elevation)
    {
        double ce = std::cos(elevation);
        return Eigen::Vector3d(ce * std::cos(azimuth), ce * std::sin(azimuth), std::sin(elevation));
    }

    void scanCallback(const radar_ego_velocity::RadarScan::ConstPtr &msg)
    {
        // 1) 取有效目标的 (视线向量 d, 量测 b=sign*doppler)
        std::vector<Eigen::Vector3d> dirs;
        std::vector<double> meas;
        dirs.reserve(msg->targets.size());
        meas.reserve(msg->targets.size());
        for (const auto &t : msg->targets)
        {
            if (t.range < min_range_ || t.range > max_range_) continue;
            Eigen::Vector3d d = los(t.azimuth, mode_2d_ ? 0.0 : t.elevation);
            dirs.push_back(d);
            meas.push_back(doppler_sign_ * t.doppler);   // 期望: b_i = d_i^T v_S
        }
        const int N = (int)dirs.size();
        const int min_pts = mode_2d_ ? 2 : 3;
        if (N < std::max(min_pts, min_inliers_))
        {
            ROS_DEBUG("[radar_ego_vel] too few targets (%d), skip", N);
            return;
        }

        // 2) RANSAC: 随机取 min_pts 个点解候选, 统计内点
        Eigen::Vector3d best_v = Eigen::Vector3d::Zero();
        int best_inliers = 0;
        for (int it = 0; it < ransac_iter_; ++it)
        {
            Eigen::MatrixXd M(min_pts, mode_2d_ ? 2 : 3);
            Eigen::VectorXd b(min_pts);
            for (int k = 0; k < min_pts; ++k)
            {
                int idx = std::rand() % N;
                if (mode_2d_) M.row(k) << dirs[idx](0), dirs[idx](1);
                else          M.row(k) = dirs[idx].transpose();
                b(k) = meas[idx];
            }
            if (std::fabs(M.determinant()) < 1e-6) continue;     // 退化采样, 跳过
            Eigen::VectorXd v = M.colPivHouseholderQr().solve(b);

            Eigen::Vector3d v3 = Eigen::Vector3d::Zero();
            if (mode_2d_) { v3(0) = v(0); v3(1) = v(1); }
            else          v3 = v;
            if (v3.norm() > max_speed_) continue;                // 不合理速度, 丢弃候选

            int cnt = 0;
            for (int i = 0; i < N; ++i)
                if (std::fabs(dirs[i].dot(v3) - meas[i]) < inlier_thresh_) ++cnt;
            if (cnt > best_inliers) { best_inliers = cnt; best_v = v3; }
        }

        if (best_inliers < min_inliers_)
        {
            ROS_DEBUG("[radar_ego_vel] RANSAC inliers=%d < %d, skip", best_inliers, min_inliers_);
            return;
        }

        // 3) 用全部内点做最终最小二乘
        std::vector<int> inlier_idx;
        for (int i = 0; i < N; ++i)
            if (std::fabs(dirs[i].dot(best_v) - meas[i]) < inlier_thresh_) inlier_idx.push_back(i);

        const int cols = mode_2d_ ? 2 : 3;
        Eigen::MatrixXd Mf(inlier_idx.size(), cols);
        Eigen::VectorXd bf(inlier_idx.size());
        for (size_t r = 0; r < inlier_idx.size(); ++r)
        {
            int i = inlier_idx[r];
            if (mode_2d_) Mf.row(r) << dirs[i](0), dirs[i](1);
            else          Mf.row(r) = dirs[i].transpose();
            bf(r) = meas[i];
        }
        Eigen::VectorXd vf = Mf.colPivHouseholderQr().solve(bf);

        Eigen::Vector3d v_S = Eigen::Vector3d::Zero();
        if (mode_2d_) { v_S(0) = vf(0); v_S(1) = vf(1); }
        else          v_S = vf;

        // 4) 发布 (雷达系自速度)
        geometry_msgs::TwistStamped out;
        out.header = msg->header;                 // 沿用雷达帧时间戳/坐标系
        out.twist.linear.x = v_S(0);
        out.twist.linear.y = v_S(1);
        out.twist.linear.z = v_S(2);
        pub_.publish(out);
    }

    ros::Subscriber sub_;
    ros::Publisher  pub_;
    std::string scan_topic_, out_topic_;
    bool   mode_2d_;
    double doppler_sign_, inlier_thresh_, min_range_, max_range_, max_speed_;
    int    ransac_iter_, min_inliers_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_ego_velocity_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RadarEgoVelocity node(nh, pnh);
    ros::spin();
    return 0;
}
