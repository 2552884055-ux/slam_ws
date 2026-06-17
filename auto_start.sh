# !/bin/bash

sleep 2
# 切换到工作空间
cd /home/orangepi/slam_ws


xfce4-terminal --title="Livox Driver" --hold -e "bash -c 'source ./devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch'" &

# 等待雷达节点初始化
sleep 5

# 启动地图定位模块（在新终端打开，并保持终端窗口）
xfce4-terminal --title="Map Switch" --hold -e "bash -c 'source ./devel/setup.bash; roslaunch map_switch project.launch'" &

# 自启动脚本

