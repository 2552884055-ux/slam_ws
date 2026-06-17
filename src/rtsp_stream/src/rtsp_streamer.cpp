/**
 * RTSP视频流服务器 - C++实现
 * 使用mediamtx作为RTSP服务器，FFmpeg捕获屏幕
 */

#include <iostream>
#include <string>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <cstring>

// 全局进程ID，用于信号处理
pid_t mediamtx_pid = 0;
pid_t ffmpeg_pid = 0;

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "\n正在停止..." << std::endl;
    
    if (ffmpeg_pid > 0) {
        kill(ffmpeg_pid, SIGTERM);
        waitpid(ffmpeg_pid, nullptr, 0);
    }
    
    if (mediamtx_pid > 0) {
        kill(mediamtx_pid, SIGTERM);
        waitpid(mediamtx_pid, nullptr, 0);
    }
    
    // 删除临时配置文件
    remove("/tmp/mediamtx.yml");
    
    exit(0);
}

// 获取本机IP地址
std::string getLocalIP() {
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];
    std::string ip = "127.0.0.1";
    
    if (getifaddrs(&ifaddr) == -1) {
        return ip;
    }
    
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        
        int family = ifa->ifa_addr->sa_family;
        
        if (family == AF_INET) {
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                              host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
            if (s == 0) {
                std::string temp(host);
                // 跳过回环地址
                if (temp != "127.0.0.1" && temp.find("127.") != 0) {
                    ip = temp;
                    break;
                }
            }
        }
    }
    
    freeifaddrs(ifaddr);
    return ip;
}

// 检查命令是否存在
bool commandExists(const std::string& cmd) {
    std::string command = "command -v " + cmd + " > /dev/null 2>&1";
    return system(command.c_str()) == 0;
}

// 创建mediamtx配置文件
bool createMediamtxConfig(int rtspPort, const std::string& streamName) {
    std::ofstream config("/tmp/mediamtx.yml");
    if (!config.is_open()) {
        std::cerr << "错误: 无法创建配置文件" << std::endl;
        return false;
    }
    
    config << "# mediamtx配置\n";
    config << "rtspAddress: :" << rtspPort << "\n";
    config << "rtmpAddress: :1935\n";
    config << "hlsAddress: :8888\n";
    config << "webrtcAddress: :8889\n";
    config << "\n";
    config << "paths:\n";
    config << "  " << streamName << ":\n";
    config << "    runOnInit: echo \"Stream started\"\n";
    config << "    runOnDemand: echo \"Stream requested\"\n";
    
    config.close();
    return true;
}

// 启动mediamtx服务器
pid_t startMediamtx() {
    pid_t pid = fork();
    
    if (pid == 0) {
        // 子进程：执行mediamtx
        execlp("mediamtx", "mediamtx", "/tmp/mediamtx.yml", nullptr);
        // 如果exec失败
        std::cerr << "错误: 无法启动mediamtx" << std::endl;
        exit(1);
    }
    
    return pid;
}

// 启动FFmpeg捕获
pid_t startFFmpeg(const std::string& displayNum, const std::string& resolution,
                  int fps, int rtspPort, const std::string& streamName) {
    pid_t pid = fork();
    
    if (pid == 0) {
        // 子进程：执行FFmpeg
        std::string input = displayNum;
        std::string rtspUrl = "rtsp://localhost:" + std::to_string(rtspPort) + "/" + streamName;
        std::string fpsStr = std::to_string(fps);
        std::string gValue = std::to_string(fps * 2);
        
        execlp("ffmpeg", "ffmpeg",
               "-f", "x11grab",
               "-video_size", resolution.c_str(),
               "-framerate", fpsStr.c_str(),
               "-i", input.c_str(),
               "-c:v", "libx264",
               "-preset", "ultrafast",
               "-tune", "zerolatency",
               "-b:v", "2M",
               "-maxrate", "2M",
               "-bufsize", "4M",
               "-pix_fmt", "yuv420p",
               "-g", gValue.c_str(),
               "-f", "rtsp",
               "-rtsp_transport", "tcp",
               rtspUrl.c_str(),
               nullptr);
        
        // 如果exec失败
        std::cerr << "错误: 无法启动FFmpeg" << std::endl;
        exit(1);
    }
    
    return pid;
}

int main(int argc, char* argv[]) {
    // 默认参数
    std::string displayNum = ":0";
    int rtspPort = 8554;
    std::string streamName = "mapping";
    int fps = 15;
    std::string resolution = "1920x1080";
    
    // 解析命令行参数
    if (argc > 1) rtspPort = std::stoi(argv[1]);
    if (argc > 2) streamName = argv[2];
    if (argc > 3) fps = std::stoi(argv[3]);
    if (argc > 4) resolution = argv[4];
    
    // 检查DISPLAY环境变量
    const char* display_env = getenv("DISPLAY");
    if (display_env != nullptr) {
        displayNum = display_env;
    }
    
    // 打印配置信息
    std::cout << "==========================================" << std::endl;
    std::cout << "RTSP视频流 (使用mediamtx)" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "显示器: " << displayNum << std::endl;
    std::cout << "RTSP端口: " << rtspPort << std::endl;
    std::cout << "流名称: " << streamName << std::endl;
    std::cout << "帧率: " << fps << " FPS" << std::endl;
    std::cout << "分辨率: " << resolution << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;
    
    // 检查依赖
    if (!commandExists("mediamtx")) {
        std::cerr << "错误: mediamtx未安装" << std::endl;
        std::cerr << std::endl;
        std::cerr << "安装方法:" << std::endl;
        std::cerr << "  bash install_mediamtx.sh" << std::endl;
        return 1;
    }
    
    if (!commandExists("ffmpeg")) {
        std::cerr << "错误: FFmpeg未安装" << std::endl;
        std::cerr << "安装: sudo apt-get install ffmpeg" << std::endl;
        return 1;
    }
    
    // 获取本机IP
    std::string localIP = getLocalIP();
    
    std::cout << "RTSP流地址:" << std::endl;
    std::cout << "  本地: rtsp://localhost:" << rtspPort << "/" << streamName << std::endl;
    std::cout << "  远程: rtsp://" << localIP << ":" << rtspPort << "/" << streamName << std::endl;
    std::cout << std::endl;
    std::cout << "播放命令:" << std::endl;
    std::cout << "  vlc rtsp://" << localIP << ":" << rtspPort << "/" << streamName << std::endl;
    std::cout << std::endl;
    std::cout << "按Ctrl+C停止" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 创建mediamtx配置
    if (!createMediamtxConfig(rtspPort, streamName)) {
        return 1;
    }
    
    // 启动mediamtx
    std::cout << "启动mediamtx服务器..." << std::endl;
    mediamtx_pid = startMediamtx();
    
    if (mediamtx_pid < 0) {
        std::cerr << "错误: 无法启动mediamtx" << std::endl;
        return 1;
    }
    
    // 等待mediamtx启动
    sleep(3);
    
    // 启动FFmpeg
    std::cout << "启动屏幕捕获..." << std::endl;
    ffmpeg_pid = startFFmpeg(displayNum, resolution, fps, rtspPort, streamName);
    
    if (ffmpeg_pid < 0) {
        std::cerr << "错误: 无法启动FFmpeg" << std::endl;
        kill(mediamtx_pid, SIGTERM);
        return 1;
    }
    
    std::cout << std::endl;
    std::cout << "服务器已启动！" << std::endl;
    std::cout << "mediamtx PID: " << mediamtx_pid << std::endl;
    std::cout << "FFmpeg PID: " << ffmpeg_pid << std::endl;
    std::cout << std::endl;
    std::cout << "等待几秒后可以开始播放..." << std::endl;
    
    // 等待子进程
    int status;
    pid_t wpid;
    while ((wpid = wait(&status)) > 0) {
        if (wpid == mediamtx_pid) {
            std::cerr << "mediamtx进程已退出" << std::endl;
            if (ffmpeg_pid > 0) {
                kill(ffmpeg_pid, SIGTERM);
            }
            break;
        } else if (wpid == ffmpeg_pid) {
            std::cerr << "FFmpeg进程已退出" << std::endl;
            if (mediamtx_pid > 0) {
                kill(mediamtx_pid, SIGTERM);
            }
            break;
        }
    }
     
    // 清理
    remove("/tmp/mediamtx.yml");
    
    return 0;
}
