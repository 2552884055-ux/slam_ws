
add_library(elevator_modules
    elevator_controller.cpp
    map_switch.cpp
    robot_elevator_client.cpp
)

target_include_directories(elevator_modules PUBLIC ${CMAKE_SOURCE_DIR}/include)

find_package(Threads REQUIRED)

target_link_libraries(elevator_modules PRIVATE ${CMAKE_THREAD_LIBS_INIT})

