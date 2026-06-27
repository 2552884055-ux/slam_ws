#include "map_switch.hpp"

replay_frame SendMapSwitchRequest(const req_frame& request, const char* server_addr, int map_switch_PORT) {
    replay_frame reply = {false, 0};

    // 创建套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return reply;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(map_switch_PORT);
    if (inet_pton(AF_INET, server_addr, &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(sockfd);
        return reply;
    }

    // 连接到服务端
    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        close(sockfd);
        return reply;
    }

    // 发送请求
    ssize_t send_len = send(sockfd, &request, sizeof(request), 0);
    if (send_len < 0) {
        std::cerr << "Send failed" << std::endl;
    } else {
        std::cout << "Request sent successfully" << std::endl;
    }

    std::cout << "Waiting for the map switch to be completed..." << std::endl;

    // 接收服务端的回复
    ssize_t recv_len = recv(sockfd, &reply, sizeof(reply), 0);
    if (recv_len < 0) {
        std::cerr << "Receive reply failed: " << strerror(errno) << std::endl;
    }
    else if (recv_len == 0) {
        std::cerr << "Server closed the connection" << std::endl;
    }

    close(sockfd);
    return reply;
}    

