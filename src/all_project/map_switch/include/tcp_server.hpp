// tcp_server.hpp —— 通用 TCP 请求/回执服务(只管收发,不含业务逻辑)
//
// 职责:绑定监听端口、accept 连接、循环收满定长 req_frame、把 (请求, client_fd) 交给回调;
//       回调负责异步处理并最终用 sendReply() 回执 + 关闭该 fd。

#pragma once

#include "protocol.hpp"
#include <functional>
#include <thread>
#include <atomic>
#include <string>

class TcpServer {
public:
    using Handler = std::function<void(const req_frame&, int client_fd)>;

    TcpServer();
    ~TcpServer();

    /** 创建并绑定监听 socket。addr 为 "0.0.0.0"/"0" 时监听所有网卡。成功返回 true。 */
    bool bindAndListen(const std::string& addr, int port);
    /** 启动 accept 线程,每收到一帧完整请求调用 handler(request, client_fd)。 */
    void start(Handler handler);
    /** 停止监听并回收线程。 */
    void stop();

    /** 向 client_fd 发送回执并关闭(任意线程可调用)。 */
    static void sendReply(int client_fd, const replay_frame& reply);

private:
    int  createAndBindTcpSocket(const std::string& addr, int port);
    void acceptLoop();

    int               listenfd_;
    std::atomic<bool> running_;
    std::thread       thread_;
    Handler           handler_;
};
