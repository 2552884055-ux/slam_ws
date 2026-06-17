cmake_minimum_required(VERSION 3.0.2)
project(radar_ego_velocity)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  RadarTarget.msg
  RadarScan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ---- 自速度估计节点 (核心, 平台无关) ----
add_executable(radar_ego_velocity_node src/radar_ego_velocity_node.cpp)
add_dependencies(radar_ego_velocity_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_ego_velocity_node ${catkin_LIBRARIES})

# ---- SR71 驱动节点 (POSIX 串口/CAN; 仅 Linux 编译, Windows 不编译) ----
if(UNIX AND NOT APPLE)
  # 串口(TTL/UART)驱动 —— SR71 实际使用
  add_executable(sr71_serial_node src/sr71_serial_node.cpp)
  add_dependencies(sr71_serial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(sr71_serial_node ${catkin_LIBRARIES})

  # CAN 驱动 —— 备用(如改用 CAN 接口)
  add_executable(sr71_can_node src/sr71_can_node.cpp)
  add_dependencies(sr71_can_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(sr71_can_node ${catkin_LIBRARIES})
endif()
