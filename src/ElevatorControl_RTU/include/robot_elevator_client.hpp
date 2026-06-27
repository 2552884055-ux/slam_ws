 // RTU方式
#ifndef ROBOT_ELEVATOR_CLIENT_H
#define ROBOT_ELEVATOR_CLIENT_H

#include "elevator_controller.hpp"
#include "map_switch.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>


class RobotElevatorClient {
public:
    RobotElevatorClient(const std::string& device, int baudrate, char parity, int data_bits, int stop_bits, int slave_id);

    void performElevatorRide(const std::string& currentFloor, const std::string& targetFloor);

    // bool callElevatorAndOpenDoor(int FromFloor, int ToFloor);
    // bool rideToTargetFloorAndOpenDoor(int FromFloor, int ToFloor);
    bool callElevatorAndOpenDoor(int FromFloor);
    bool rideToTargetFloorAndOpenDoor(int ToFloor);
    bool closeDoorAndSwitchMap(const req_frame& request, const char* server_addr, int map_switch_PORT);

private:
    ElevatorController m_controller;
    
    // 通用带重试的控制指令
    bool sendRideCommandWithRetry(const std::string& floor, int retryLimit = 100, int intervalMs = 10);
    bool requestOpenMainDoorWithRetry(int retryLimit = 100, int intervalMs = 10);
    bool requestCloseDoorWithRetry(int retryLimit = 100, int intervalMs = 10);
    // 等待逻辑
    bool waitElevatorOnlineAndActive(int timeout_sec = 60);
    bool waitElevatorArrives(const std::string& floor, int timeout_sec = 180);

};

#endif      
