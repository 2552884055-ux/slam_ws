// node_launcher.hpp —— ROS 节点进程管理(fork + rosrun 启动 / 按进程组杀死)
//
// 职责:把节点用 rosrun 拉起为独立进程组,并能按 PID(进程组)优雅→强制杀死并回收。
//       不含任何切换业务逻辑。

#pragma once

#include "protocol.hpp"
#include <sys/types.h>

class NodeLauncher {
public:
    /** fork + execvp("rosrun", pkg, type, args..., __name:=name),子进程独立进程组。
     *  @return 子进程 PID(同时是其 PGID),失败返回 -1。 */
    pid_t launch(const NodeSpec& spec);

    /** 杀死一个节点进程组:先 SIGINT 优雅退出,超时再 SIGKILL,并 waitpid 回收。 */
    void killNode(pid_t pid);
};
