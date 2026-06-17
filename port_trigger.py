#!/usr/bin/env python3
import socket
import subprocess
import os

HOST = "0.0.0.0"
PORT = 8080
SCRIPT_PATH = "/home/orangepi/maduo_ws/auto_start.sh"

# -------------------------------------------------
# 启动脚本
# -------------------------------------------------
def run_auto_start():
    if not os.path.exists(SCRIPT_PATH):
        print(f"[ERROR] 文件不存在: {SCRIPT_PATH}")
        return
    try:
        subprocess.Popen(["/bin/bash", SCRIPT_PATH])
        print("[INFO] auto_start.sh 已启动")
    except Exception as e:
        print(f"[ERROR] 无法启动脚本: {e}")


# -------------------------------------------------
# 安全停止所有 ROS 节点
# -------------------------------------------------
def stop_ros():
    print("[INFO] 正在安全关闭所有 ROS 节点...")

    ros_nodes = [
        "/Init_IMU2robot",
        "/body2robot",
        "/laserMapping",
        "/livox_lidar_publisher2",
        "/point2laserscan",
        "/pointcloud_to_laserscan",
        "/tf_robot2map",
        "/rosout"
    ]

    # 杀节点
    for node in ros_nodes:
        try:
            subprocess.call(["rosnode", "kill", node])
        except Exception as e:
            print(f"[WARN] 无法终止 {node}: {e}")

    # 冗余清理
    cleanup_cmds = [
        "pkill -f livox",
        "pkill -f laserMapping",
        "pkill -f pointcloud_to_laserscan",
    ]
    for cmd in cleanup_cmds:
        try:
            subprocess.call(cmd, shell=True)
        except Exception as e:
            print(f"[WARN] 清理命令失败: {cmd}")

    print("[INFO] 所有 ROS 节点已结束")


# -------------------------------------------------
# 查询运行中的 ROS 节点 (status)
# -------------------------------------------------
def ros_status():
    try:
        result = subprocess.check_output(["rosnode", "list"]).decode()
        return result
    except Exception as e:
        return f"[ERROR] 无法查询 ROS 状态: {e}"


# -------------------------------------------------
# 主 TCP 监听服务
# -------------------------------------------------
def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)

        print(f"[INFO] 正在监听端口 {PORT}...")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"[INFO] 来自 {addr} 的连接")

                data = conn.recv(1024).decode().strip()
                print(f"[INFO] 收到数据: {data}")

                # ---------- START ----------
                if data.lower() == "start":
                    run_auto_start()
                    conn.sendall(b"OK: auto_start.sh started\n")

                # ---------- STOP ----------
                elif data.lower() == "stop":
                    stop_ros()
                    conn.sendall(b"OK: all ros nodes stopped\n")

                # ---------- STATUS ----------
                elif data.lower() == "status":
                    status = ros_status()
                    conn.sendall(status.encode())

                # ---------- UNKNOWN ----------
                else:
                    conn.sendall(b"IGNORED: unknown command\n")


if __name__ == "__main__":
    main()
