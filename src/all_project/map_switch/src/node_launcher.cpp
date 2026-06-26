#include "node_launcher.hpp"

#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstring>
#include <cerrno>
#include <vector>
#include <string>

pid_t NodeLauncher::launch(const NodeSpec& spec)
{
    /** 组装 argv:rosrun pkg type [args...] __name:=name */
    std::vector<std::string> argv_s;
    argv_s.push_back("rosrun");
    argv_s.push_back(spec.pkg);
    argv_s.push_back(spec.type);
    for (const auto& a : spec.args) argv_s.push_back(a);
    argv_s.push_back("__name:=" + spec.name);

    std::vector<char*> argv;
    argv.reserve(argv_s.size() + 1);
    for (auto& s : argv_s) argv.push_back(const_cast<char*>(s.c_str()));
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
        ROS_ERROR("fork failed for node %s: %s", spec.name.c_str(), strerror(errno));
        return -1;
    }
    if (pid == 0) {
        /** 子进程:只调用 async-signal-safe 接口(setpgid/execvp/write/_exit);
         *  PYTHONPATH 已在主进程构造时进程级 setenv,这里不再 setenv。 */
        setpgid(0, 0);  ///< 独立进程组,便于按组杀死 rosrun 及其子节点
        execvp("rosrun", argv.data());
        const char msg[] = "execvp rosrun failed\n";
        ssize_t wr = write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)wr;
        _exit(127);
    }

    /** 父进程也设一次 pgid,避免竞争(幂等) */
    setpgid(pid, pid);
    ROS_INFO("Launched node %s (rosrun %s %s) pid=%d",
             spec.name.c_str(), spec.pkg.c_str(), spec.type.c_str(), pid);
    return pid;
}

void NodeLauncher::killNode(pid_t pid)
{
    if (pid <= 0) return;

    killpg(pid, SIGINT);  ///< 让 roscpp/rospy 走正常 shutdown(FAST-LIO 不至于异常退出)

    /** 最多等约 5s 优雅退出 */
    for (int i = 0; i < 50; ++i) {
        int st;
        pid_t r = waitpid(pid, &st, WNOHANG);
        if (r == pid || (r < 0 && errno == ECHILD)) {
            ROS_INFO("node pid=%d exited.", pid);
            return;
        }
        usleep(100 * 1000);
    }

    ROS_WARN("node pid=%d did not exit on SIGINT, sending SIGKILL.", pid);
    killpg(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
}
