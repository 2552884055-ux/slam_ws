cmake_minimum_required(VERSION 3.5)
project(elevator_simulator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# 地图切换模拟服务端(纯 socket，无需 libmodbus)
add_executable(map_switch_sim map_switch_sim.cpp)
target_link_libraries(map_switch_sim PRIVATE Threads::Threads)

# libmodbus (优先用 pkg-config，找不到再回退到默认头/库路径)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(MODBUS libmodbus)
endif()

add_executable(elevator_sim elevator_sim.cpp)

if(MODBUS_FOUND)
    target_include_directories(elevator_sim PRIVATE ${MODBUS_INCLUDE_DIRS})
    target_link_libraries(elevator_sim PRIVATE ${MODBUS_LIBRARIES} Threads::Threads)
else()
    target_link_libraries(elevator_sim PRIVATE modbus Threads::Threads)
endif()
