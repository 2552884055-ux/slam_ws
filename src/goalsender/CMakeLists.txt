cmake_minimum_required(VERSION 3.0.2)
project(goal_sender)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
)

## 查找线程库
find_package(Threads REQUIRED)

## catkin 配置
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs tf
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 网络库源文件
set(NETWORK_SOURCES
  src/network/socket.cpp
  src/network/networkBase.cpp
  src/network/tcpUdp.cpp
)

## ROS目标点发送节点
add_executable(goal_sender_node
  ros_goal_sender.cpp
  ${NETWORK_SOURCES}
)

target_link_libraries(goal_sender_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

## 安装规则
install(TARGETS goal_sender_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
