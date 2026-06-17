#!/bin/bash
# 加载 ROS 环境变量
source /home/orangepi/slam_ws/devel/setup.bash


# 启动定位文件，开始定位导航
roslaunch /home/orangepi/slam_ws/src/map_switch/launch/publish_map2.launch