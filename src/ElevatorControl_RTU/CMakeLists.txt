cmake_minimum_required(VERSION 3.10)
project(ElevatorControl)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

add_subdirectory(src)

add_executable(ElevatorControl main.cpp)

target_include_directories(ElevatorControl PRIVATE ${LIBMODBUS_INCLUDE_DIRS})
target_link_directories(ElevatorControl PRIVATE ${LIBMODBUS_LIBRARY_DIRS})

target_link_libraries(ElevatorControl
    PRIVATE
        elevator_modules
        ${LIBMODBUS_LIBRARIES}
)
