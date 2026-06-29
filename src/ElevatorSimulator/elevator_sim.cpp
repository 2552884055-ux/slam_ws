// ============================================================================
//  模拟电梯控制器 (Modbus 从站 / 服务器)
//
//  用途：在没有真实电梯控制器的情况下，模拟一台支持 Modbus 的电梯，
//        供 ElevatorControl 上位机程序(TCP/RTU 二合一)联调测试。
//
//  上位机是 Modbus 主站(client)，本程序是 Modbus 从站(server)，寄存器布局依据
//  《机器人乘梯通信协议》，并与上位机 elevator_controller.cpp 解码方式严格一致：
//
//   —— 电梯公共数据表 / 输入寄存器 (FC04 只读, 电梯→机器人) ——
//     IR[3]  低字节 bit0     电梯投入状态(0退出/1投入) isActive
//     IR[4]  高/低字节        电梯通讯计数器, 约1秒自增1(机器人据此判电梯在线)
//     IR[5]  低字节 bit0      上行 isUpward
//            低字节 bit1      下行 isDownward
//            低字节 bit2      运行中 isRuning
//            低字节 bit3      正常 isNormal
//            低字节 bit5      主门开门到位 mainDoorOpen
//            低字节 bit7      副门开门到位 viceDoorOpen
//            高字节 bit1      机器人使用中
//     IR[6]  低字节 = 轿厢楼层 ASCII 第0字符
//     IR[7]  高字节 = 第1字符, 低字节 = 第2字符        (例: "001")
//     IR[9]  高字节 bit0      回显:开主门;  高字节 bit1 回显:开副门
//            低字节 bit2      内呼(乘梯指令确认, 最后一次)
//     IR[10] 低字节 = 外呼/内呼楼层 ASCII 第0字符
//     IR[11] 高字节 = 第1字符, 低字节 = 第2字符
//
//   —— 机器人乘梯数据表 / 保持寄存器 (FC03/06/10 读写, 机器人→电梯) ——
//     HR[2]  高/低字节        机器人通讯计数器, 约1秒自增1 (startCommFlagThread)
//     HR[7]  低字节 bit0开主门 / bit1开副门;  0=关门  1=开主门  2=开副门
//     HR[8]  乘梯请求: 高字节 bit7=乘梯 + 低字节 bit2=内呼 → 0x8004
//     HR[9]  低字节 = 用梯楼层 ASCII 第0字符; 高字节 bit0: 0=主门/1=副门
//     HR[10] 高字节 = 第1字符, 低字节 = 第2字符
//
//  编译: g++ -std=c++11 elevator_sim.cpp -lmodbus -pthread -o elevator_sim
//  运行(TCP): ./elevator_sim --tcp 0.0.0.0 8000
//  运行(RTU): ./elevator_sim --rtu /dev/ttyUSB0 9600 N 8 1 1  (USB-TTL,与控制器侧串口交叉对接)
// ============================================================================

#include <modbus/modbus.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <iomanip>

#include <errno.h>
#include <unistd.h>
#include <sys/select.h>

// ---------------- 寄存器索引 ----------------
// 依据《机器人乘梯通信协议》:
//   电梯公共数据表  (输入寄存器, 30001~, FC04 只读)  —— 电梯 → 机器人
//   机器人乘梯数据表(保持寄存器, 40001~, FC03/06/10) —— 机器人 → 电梯
namespace reg {
// —— 输入寄存器 (电梯公共数据表) ——
constexpr int IR_ACTIVE    = 3;   // [3] 低字节 bit0: 电梯投入状态(0退出/1投入)
constexpr int IR_COMM      = 4;   // [4] 电梯通讯计数器(高/低字节), 约1秒自增1
constexpr int IR_STATUS    = 5;   // [5] 门/运行状态位 + 高字节 bit1 机器人使用中
constexpr int IR_CURFLOOR  = 6;   // [6]低字节+[7]: 电梯轿厢楼层(3字符ASCII)
constexpr int IR_CMDECHO   = 9;   // [9] 机器人门控制指令回显 + 乘梯指令确认(最后一次)
constexpr int IR_CALLFLOOR = 10;  // [10]低字节+[11]: 机器人外呼/内呼楼层(3字符ASCII)

// —— 保持寄存器 (机器人乘梯数据表) ——
constexpr int HR_COMM     = 2;    // [2] 机器人通讯计数器(高/低字节), 约1秒自增1
constexpr int HR_DOOR     = 7;    // [7] 机器人开门请求: 低字节 bit0开主门/bit1开副门
constexpr int HR_RIDE     = 8;    // [8] 机器人乘梯: 高字节 bit7乘梯 / 低字节 bit2内呼
constexpr int HR_RIDE_C0  = 9;    // [9] 用梯楼层字符0(低字节); 高字节 bit0 0=主门/1=副门
constexpr int HR_RIDE_C12 = 10;   // [10] 用梯楼层字符1(高字节)/字符2(低字节)

constexpr uint16_t RIDE_MARK = 0x8004;   // [8] 乘梯(bit15) + 内呼(bit2)

// IR_STATUS([5]) 位:低字节为电梯运行/门状态,高字节 bit1(=bit9) 为机器人使用中
constexpr uint16_t ST_UP          = 0x0001;  // 低 bit0 上行
constexpr uint16_t ST_DOWN        = 0x0002;  // 低 bit1 下行
constexpr uint16_t ST_RUN         = 0x0004;  // 低 bit2 运行中
constexpr uint16_t ST_NORMAL      = 0x0008;  // 低 bit3 正常
constexpr uint16_t ST_MAIN        = 0x0020;  // 低 bit5 主门开门到位
constexpr uint16_t ST_VICE        = 0x0080;  // 低 bit7 副门开门到位
constexpr uint16_t ST_ROBOT_USING = 0x0200;  // 高 bit1 机器人使用中

// IR_CMDECHO([9]) 位:高字节回显门控制指令,低字节 bit2 回显乘梯确认
constexpr uint16_t ECHO_OPEN_MAIN    = 0x0100;  // 高 bit0 开主门
constexpr uint16_t ECHO_OPEN_VICE    = 0x0200;  // 高 bit1 开副门
constexpr uint16_t ECHO_RIDE_CONFIRM = 0x0004;  // 低 bit2 内呼(乘梯指令确认)

// 机器人使用中判定:机器人通讯计数器 N 毫秒内有变化则视为占用中(协议为3秒)
constexpr long ROBOT_USING_TIMEOUT_MS = 3000;
}  // namespace reg

// 保持/输入寄存器数量。协议地址范围为 0~14(30001~300015 / 40001~40015 共15个)，
// 取 32 留足余量，确保 0~14 任意地址都能被读到；未使用的保留寄存器由
// modbus_mapping_new 初始化为 0，模拟器不写它们 → 读出即为 0。
constexpr int NB_REGS = 32;

// ---------------- 可调仿真参数 ----------------
constexpr int   SIM_TICK_MS       = 100;   // 仿真步进周期
constexpr int   FLOOR_TRAVEL_MS   = 5000;  // 每层运行耗时
constexpr int   DOOR_MOVE_MS      = 3000;  // 开/关门耗时

// ---------------- 工具函数 ----------------
static std::string formatFloor(int floor) {
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << floor;
    return ss.str();
}

using clk = std::chrono::steady_clock;
static long ms_since(clk::time_point t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t).count();
}

// ---------------- 全局运行标志 ----------------
static std::atomic<bool> g_running{true};
static modbus_t* g_ctx = nullptr;
static int g_server_socket = -1;

static void onSignal(int) {
    g_running = false;
    if (g_server_socket != -1) { close(g_server_socket); g_server_socket = -1; }
}

// ============================================================================
//  电梯仿真器：维护一台电梯的内部状态，根据保持寄存器(指令)推进，
//             并把结果写回输入寄存器(状态)。
// ============================================================================
class ElevatorSim {
public:
    ElevatorSim(modbus_mapping_t* mb, std::mutex& mtx, int start_floor)
        : m_mb(mb), m_mtx(mtx), m_curFloor(start_floor), m_callFloor(start_floor) {
        std::lock_guard<std::mutex> lk(m_mtx);
        // 上电即激活、正常、门关闭、停在起始层
        m_mb->tab_input_registers[reg::IR_ACTIVE] = 0x0001;
        writeStatus_();
        writeFloor_(reg::IR_CURFLOOR, m_curFloor);
        writeFloor_(reg::IR_CALLFLOOR, m_callFloor);
        std::cout << "[仿真] 电梯上电，停在 " << formatFloor(m_curFloor)
                  << " 层，门关闭，已激活。" << std::endl;
    }

    // 每个仿真步调用一次
    void tick() {
        std::lock_guard<std::mutex> lk(m_mtx);

        updateComm_();          // 电梯通讯计数器自增 + 机器人占用判定
        handleRideCommand_();
        handleMotion_();
        handleDoor_();
        writeStatus_();         // 写 IR[5] 状态位
        writeCmdEcho_();        // 写 IR[9] 指令回显/确认
    }

private:
    enum class Door { CLOSED, OPENING, OPEN, CLOSING };

    // ---- 电梯通讯计数器(IR[4]) + 机器人占用判定 ----
    // 真实电梯约每秒把通讯计数器+1,机器人据此判断"电梯设备通讯标志"是否有效;
    // 同时监视机器人侧的通讯计数器(HR[2]):3秒内有变化则认为机器人正在用梯。
    void updateComm_() {
        if (ms_since(m_lastComm) >= 1000) {
            m_lastComm = clk::now();
            m_mb->tab_input_registers[reg::IR_COMM] = ++m_commCounter;
        }
        uint16_t hb = m_mb->tab_registers[reg::HR_COMM];
        if (hb != m_lastHbVal) {
            m_lastHbVal = hb;
            m_lastHbChange = clk::now();
            m_hbSeen = true;
        }
        m_robotUsing = m_hbSeen && ms_since(m_lastHbChange) < reg::ROBOT_USING_TIMEOUT_MS;
    }

    // ---- 解析乘梯指令(保持寄存器 8/9/10) ----
    void handleRideCommand_() {
        if (m_mb->tab_registers[reg::HR_RIDE] != reg::RIDE_MARK) return;

        uint16_t c0  = m_mb->tab_registers[reg::HR_RIDE_C0]  & 0x00FF;
        uint16_t c12 = m_mb->tab_registers[reg::HR_RIDE_C12];
        char s[4] = { static_cast<char>(c0),
                      static_cast<char>(c12 >> 8),
                      static_cast<char>(c12 & 0x00FF), '\0' };
        int target = std::atoi(s);
        if (target <= 0) return;

        // 立即回显召梯楼层，让上位机 sendRideCommandWithRetry 确认。
        // 注意:这里只记录目标层,真正起步的判断放到 handleMotion_ 每个 tick 评估——
        // 因为上位机关门后(门还在 CLOSING)就会马上发乘梯指令,此刻门未完全关闭,
        // 不能在这一刻一次性决定是否起步,否则会永远卡住不动。
        if (target != m_callFloor) {
            m_callFloor = target;
            writeFloor_(reg::IR_CALLFLOOR, m_callFloor);
            std::cout << "[仿真] 收到乘梯指令 → 目标楼层 " << formatFloor(target) << std::endl;
        }
    }

    // ---- 楼层运动 ----
    void handleMotion_() {
        // 每个 tick 评估起步:目标层≠当前层 且 门已完全关闭 且 当前未运动
        if (!m_moving && m_curFloor != m_callFloor && m_door == Door::CLOSED) {
            m_moving = true;
            m_movingUp = (m_callFloor > m_curFloor);
            m_lastMove = clk::now();
            std::cout << "[仿真] 开始" << (m_movingUp ? "上行" : "下行")
                      << "，从 " << formatFloor(m_curFloor)
                      << " 前往 " << formatFloor(m_callFloor) << std::endl;
        }

        if (!m_moving) return;

        if (m_curFloor == m_callFloor) {  // 已到达
            m_moving = false;
            std::cout << "[仿真] 已到达目标楼层 " << formatFloor(m_curFloor) << std::endl;
            return;
        }

        if (ms_since(m_lastMove) >= FLOOR_TRAVEL_MS) {
            m_curFloor += (m_movingUp ? 1 : -1);
            m_lastMove = clk::now();
            writeFloor_(reg::IR_CURFLOOR, m_curFloor);
            std::cout << "[仿真] 经过楼层 " << formatFloor(m_curFloor) << std::endl;
            if (m_curFloor == m_callFloor) {
                m_moving = false;
                std::cout << "[仿真] 已到达目标楼层 " << formatFloor(m_curFloor) << std::endl;
            }
        }
    }

    // ---- 门控制(保持寄存器 7) ----
    void handleDoor_() {
        uint16_t cmd = m_mb->tab_registers[reg::HR_DOOR];

        // 运动中忽略开门请求
        if (m_moving) return;

        // 状态机推进
        switch (m_door) {
            case Door::CLOSED:
                if (cmd == 0x0001 || cmd == 0x0002) {
                    m_door = Door::OPENING;
                    m_doorStart = clk::now();
                    std::cout << "[仿真] 收到开门指令，开门中..." << std::endl;
                }
                break;
            case Door::OPENING:
                if (cmd == 0x0000) {  // 中途撤销
                    m_door = Door::CLOSING; m_doorStart = clk::now();
                } else if (ms_since(m_doorStart) >= DOOR_MOVE_MS) {
                    m_door = Door::OPEN;
                    std::cout << "[仿真] 主门已完全打开。" << std::endl;
                }
                break;
            case Door::OPEN:
                if (cmd == 0x0000) {
                    m_door = Door::CLOSING;
                    m_doorStart = clk::now();
                    std::cout << "[仿真] 收到关门指令，关门中..." << std::endl;
                }
                break;
            case Door::CLOSING:
                if (cmd == 0x0001 || cmd == 0x0002) {  // 中途又要开
                    m_door = Door::OPENING; m_doorStart = clk::now();
                } else if (ms_since(m_doorStart) >= DOOR_MOVE_MS) {
                    m_door = Door::CLOSED;
                    std::cout << "[仿真] 主门已完全关闭。" << std::endl;
                }
                break;
        }
    }

    // ---- 把内部状态写入输入寄存器 IR[5] ----
    void writeStatus_() {
        uint16_t st = reg::ST_NORMAL;
        if (m_moving) {
            st |= reg::ST_RUN;
            st |= (m_movingUp ? reg::ST_UP : reg::ST_DOWN);
        }
        if (m_door == Door::OPEN) st |= reg::ST_MAIN;
        if (m_robotUsing)         st |= reg::ST_ROBOT_USING;  // 高字节 bit1 机器人使用中
        m_mb->tab_input_registers[reg::IR_STATUS] = st;
    }

    // ---- 写指令回显寄存器 IR[9]:门控制指令回显 + 乘梯指令确认(最后一次) ----
    // 真实电梯把机器人下发的门控制(HR[7])与乘梯请求(HR[8])回显到公共数据区,
    // 供机器人确认指令已被电梯正确接收。
    void writeCmdEcho_() {
        uint16_t echo = 0;
        uint16_t door = m_mb->tab_registers[reg::HR_DOOR];
        if (door == 0x0001) echo |= reg::ECHO_OPEN_MAIN;   // 开主门
        else if (door == 0x0002) echo |= reg::ECHO_OPEN_VICE;  // 开副门
        if (m_mb->tab_registers[reg::HR_RIDE] == reg::RIDE_MARK)
            echo |= reg::ECHO_RIDE_CONFIRM;                // 内呼(乘梯指令确认)
        m_mb->tab_input_registers[reg::IR_CMDECHO] = echo;
    }

    // ---- 楼层(int)编码到一对输入寄存器，与上位机解码方式一致 ----
    void writeFloor_(int base, int floor) {
        std::string s = formatFloor(floor);
        m_mb->tab_input_registers[base]     = static_cast<uint8_t>(s[0]);
        m_mb->tab_input_registers[base + 1] =
            (static_cast<uint8_t>(s[1]) << 8) | static_cast<uint8_t>(s[2]);
    }

    modbus_mapping_t* m_mb;
    std::mutex&       m_mtx;

    int  m_curFloor;
    int  m_callFloor;
    bool m_moving   = false;
    bool m_movingUp = false;
    Door m_door     = Door::CLOSED;
    clk::time_point m_lastMove  = clk::now();
    clk::time_point m_doorStart = clk::now();

    // 通讯计数器与机器人占用判定
    uint16_t m_commCounter = 0;            // 电梯自身通讯计数(写入 IR[4])
    clk::time_point m_lastComm = clk::now();
    uint16_t m_lastHbVal = 0;              // 上次看到的机器人通讯计数(HR[2])
    bool m_hbSeen = false;                 // 是否曾收到过机器人心跳
    clk::time_point m_lastHbChange = clk::now();
    bool m_robotUsing = false;             // 机器人使用中(写入 IR[5] 高字节 bit1)
};

// ============================================================================
//  Modbus 服务器主循环
// ============================================================================
static void usage(const char* prog) {
    std::cerr <<
        "用法:\n"
        "  " << prog << " --tcp [ip] [port] [start_floor]\n"
        "        默认: 0.0.0.0 8000 1\n"
        "  " << prog << " --rtu <device> [baud] [parity] [data_bits] [stop_bits] [slave_id] [start_floor]\n"
        "        默认: 9600 N 8 1 1 1   (USB-TTL, 与控制器侧串口交叉对接 TX↔RX)\n"
        "\n示例:\n"
        "  " << prog << " --tcp 0.0.0.0 8000\n"
        "  " << prog << " --rtu /dev/ttyUSB0 9600 N 8 1 1\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (argc < 2) { usage(argv[0]); return 1; }

    std::string mode = argv[1];
    int slave_id    = 1;
    int start_floor = 1;
    bool is_tcp;

    if (mode == "--tcp") {
        is_tcp = true;
        std::string ip = (argc > 2) ? argv[2] : "0.0.0.0";
        int port       = (argc > 3) ? std::atoi(argv[3]) : 8000;
        start_floor    = (argc > 4) ? std::atoi(argv[4]) : 1;
        g_ctx = modbus_new_tcp(ip.c_str(), port);
        if (!g_ctx) { std::cerr << "创建 Modbus TCP 上下文失败\n"; return 1; }
        std::cout << "[启动] Modbus TCP 电梯模拟器，监听 " << ip << ":" << port
                  << " (slave_id 任意, 起始楼层 " << formatFloor(start_floor) << ")\n";
    } else if (mode == "--rtu") {
        is_tcp = false;
        if (argc < 3) { usage(argv[0]); return 1; }
        std::string dev = argv[2];
        int  baud   = (argc > 3) ? std::atoi(argv[3]) : 9600;
        char parity = (argc > 4) ? argv[4][0] : 'N';
        int  dbits  = (argc > 5) ? std::atoi(argv[5]) : 8;
        int  sbits  = (argc > 6) ? std::atoi(argv[6]) : 1;
        slave_id    = (argc > 7) ? std::atoi(argv[7]) : 1;
        start_floor = (argc > 8) ? std::atoi(argv[8]) : 1;
        g_ctx = modbus_new_rtu(dev.c_str(), baud, parity, dbits, sbits);
        if (!g_ctx) { std::cerr << "创建 Modbus RTU 上下文失败\n"; return 1; }
        modbus_set_slave(g_ctx, slave_id);
        std::cout << "[启动] Modbus RTU 电梯模拟器，串口 " << dev << " " << baud
                  << " " << parity << " " << dbits << " " << sbits
                  << " slave_id=" << slave_id
                  << " (起始楼层 " << formatFloor(start_floor) << ")\n";
    } else {
        usage(argv[0]);
        return 1;
    }

    // 寄存器映射：保持寄存器 + 输入寄存器各 NB_REGS 个
    modbus_mapping_t* mb = modbus_mapping_new(0, 0, NB_REGS, NB_REGS);
    if (!mb) {
        std::cerr << "分配寄存器映射失败: " << modbus_strerror(errno) << "\n";
        modbus_free(g_ctx);
        return 1;
    }

    std::mutex mtx;
    ElevatorSim sim(mb, mtx, start_floor);

    // 仿真线程：周期性推进电梯状态
    std::thread sim_thread([&]() {
        while (g_running) {
            sim.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(SIM_TICK_MS));
        }
    });

    // ---------------- 服务器收发循环 ----------------
    if (is_tcp) {
        // backlog 取较大值:上位机 RobotElevatorClient 会连续多次 modbus_connect
        // (构造控制器 + 构造 client + waitElevatorOnlineAndActive),backlog 太小
        // 会导致后续连接握手卡在 EINPROGRESS。
        g_server_socket = modbus_tcp_listen(g_ctx, 16);
        if (g_server_socket == -1) {
            std::cerr << "监听失败: " << modbus_strerror(errno) << "\n";
            g_running = false; sim_thread.join();
            modbus_mapping_free(mb); modbus_free(g_ctx);
            return 1;
        }
        // 用 select() 监听「监听套接字 + 所有已连接客户端」，支持多/重复连接。
        // 上位机代码里构造函数连一次、waitElevatorOnlineAndActive 又连一次,
        // 会同时存在多条 TCP 连接,单连接阻塞模型会卡死,故必须多路复用。
        fd_set refset;
        FD_ZERO(&refset);
        FD_SET(g_server_socket, &refset);
        int fdmax = g_server_socket;
        std::cout << "[网络] 等待上位机连接..." << std::endl;

        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        while (g_running) {
            fd_set rdset = refset;
            struct timeval tv{1, 0};  // 1s 超时,以便周期性检查退出标志
            int n = select(fdmax + 1, &rdset, nullptr, nullptr, &tv);
            if (n < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (n == 0) continue;  // 超时,无事件

            for (int fd = 0; fd <= fdmax; ++fd) {
                if (!FD_ISSET(fd, &rdset)) continue;

                if (fd == g_server_socket) {
                    // 新连接
                    int client = modbus_tcp_accept(g_ctx, &g_server_socket);
                    if (client != -1) {
                        FD_SET(client, &refset);
                        if (client > fdmax) fdmax = client;
                        std::cout << "[网络] 上位机已连接 (fd=" << client << ")。" << std::endl;
                    }
                } else {
                    // 已连接客户端有请求
                    modbus_set_socket(g_ctx, fd);
                    int rc = modbus_receive(g_ctx, query);
                    if (rc > 0) {
                        std::lock_guard<std::mutex> lk(mtx);
                        modbus_reply(g_ctx, query, rc, mb);
                    } else if (rc == -1) {
                        std::cout << "[网络] 上位机断开连接 (fd=" << fd << ")。" << std::endl;
                        close(fd);
                        FD_CLR(fd, &refset);
                    }
                }
            }
        }
    } else {
        // RTU：单点对单点，直接收发
        if (modbus_connect(g_ctx) == -1) {
            std::cerr << "打开串口失败: " << modbus_strerror(errno) << "\n";
            g_running = false; sim_thread.join();
            modbus_mapping_free(mb); modbus_free(g_ctx);
            return 1;
        }
        // 给 modbus_receive 设等待请求(indication)超时,使其每 0.5s 返回一次,
        // 否则它会无限阻塞在串口读上,Ctrl-C(SIGINT)只置位 g_running 却无法被检查到,
        // 导致程序退不出。设了超时后,循环能周期性检查 g_running 并优雅退出。
        modbus_set_indication_timeout(g_ctx, 0, 500000);

        std::cout << "[串口] 已就绪，等待上位机请求...(Ctrl-C 退出)" << std::endl;
        uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
        while (g_running) {
            int rc = modbus_receive(g_ctx, query);
            if (rc > 0) {
                std::lock_guard<std::mutex> lk(mtx);
                modbus_reply(g_ctx, query, rc, mb);
            }
            // rc == -1 时多为等待超时(ETIMEDOUT)或偶发 CRC 错误,忽略并回到循环顶部
            // 重新检查 g_running;真正的退出由 SIGINT 把 g_running 置 false 触发。
        }
    }

    g_running = false;
    if (sim_thread.joinable()) sim_thread.join();
    modbus_mapping_free(mb);
    modbus_close(g_ctx);
    modbus_free(g_ctx);
    std::cout << "\n[退出] 电梯模拟器已停止。" << std::endl;
    return 0;
}
