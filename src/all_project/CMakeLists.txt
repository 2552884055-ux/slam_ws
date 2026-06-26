cmake_minimum_required(VERSION 3.0.2)
project(all_project)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(CURL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

## 头文件路径:两个节点各自的 include 子目录
include_directories(
  map_switch/include
  tf_publish/include
  ${catkin_INCLUDE_DIRS}
)

## ===== 节点1:map_switch(楼层切换,按职责拆为多文件)=====
add_executable(map_switch
  map_switch/src/map_switch.cpp
  map_switch/src/map_switch_node.cpp
  map_switch/src/tcp_server.cpp
  map_switch/src/node_launcher.cpp
)
add_dependencies(map_switch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_switch ${catkin_LIBRARIES})

## ===== 节点2:tf_publish(位姿/激光 UDP 上报)=====
add_executable(tf_publish
  tf_publish/src/tf_publish.cpp
  tf_publish/src/tf_publish_node.cpp
)
add_dependencies(tf_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_publish ${catkin_LIBRARIES})
