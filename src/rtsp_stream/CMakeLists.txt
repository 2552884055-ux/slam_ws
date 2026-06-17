cmake_minimum_required(VERSION 3.0.2)
project(rtsp_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 编译RTSP流服务器
add_executable(rtsp_streamer src/rtsp_streamer.cpp)
target_link_libraries(rtsp_streamer ${catkin_LIBRARIES})

# 安装可执行文件
install(TARGETS rtsp_streamer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/rtsp_server.py
  scripts/rviz_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# 安装web文件
install(DIRECTORY web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)
