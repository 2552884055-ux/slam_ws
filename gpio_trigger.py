#!/usr/bin/env python3
"""GPIO 按键触发 → 直接启动 ROS 各模块，无需外部 shell 脚本。"""

import gpiod
import subprocess
import time
import logging
import os
import shutil

# === 配置 ===
CHIP_NAME   = "gpiochip1"
LINE_OFFSET = 4          # orangepi5plus GPIO1_A4
DEBOUNCE    = 0.5
CHECK_INTERVAL = 0.05

WS_DIR      = "/home/orangepi/slam_ws"
LOG_DIR     = "/home/orangepi/logs"
LOG_FILE    = f"{LOG_DIR}/gpio_trigger.log"
LIVOX_LAUNCH   = "livox_ros_driver2 msg_MID360.launch"
PROJECT_LAUNCH = "all_project project.launch"
DELAY_BEFORE_LIVOX   = 2  # 启动前等待系统就绪（秒）
DELAY_AFTER_LIVOX    = 5  # 等待雷达节点初始化（秒）

# === 日志 ===
os.makedirs(LOG_DIR, exist_ok=True)
logger = logging.getLogger()
logger.setLevel(logging.INFO)
fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
fh = logging.FileHandler(LOG_FILE)
fh.setFormatter(fmt)
logger.addHandler(fh)
ch = logging.StreamHandler()
ch.setFormatter(fmt)
logger.addHandler(ch)


def _launch_cmd(launch_args: str) -> str:
    """拼接带 source 的 roslaunch 命令字符串。"""
    return f"source {WS_DIR}/devel/setup.bash && roslaunch {launch_args}"


def _start_node(title: str, launch_args: str) -> None:
    """根据是否有桌面环境选择启动方式。"""
    cmd = _launch_cmd(launch_args)
    outlog = os.path.join(LOG_DIR, title.replace(" ", "_") + ".log")

    if os.environ.get("DISPLAY") and shutil.which("xfce4-terminal"):
        subprocess.Popen(
            ["xfce4-terminal", "--title", title, "--hold", "-e", f"bash -c '{cmd}'"]
        )
    elif shutil.which("tmux"):
        subprocess.Popen(
            ["tmux", "new-window", "-d", "-n", title,
             f"bash -c '{cmd} 2>&1 | tee {outlog}'"]
        )
    else:
        with open(outlog, "a") as f:
            subprocess.Popen(["bash", "-c", cmd], stdout=f, stderr=f)

    logger.info("已启动 [%s]", title)


def _already_running() -> bool:
    """幂等检查：驱动进程已存在则返回 True。"""
    try:
        out = subprocess.check_output(["pgrep", "-f", "msg_MID360.launch"],
                                      stderr=subprocess.DEVNULL)
        return bool(out.strip())
    except subprocess.CalledProcessError:
        return False


def start_ros() -> None:
    """启动入口：幂等 → Livox 驱动 → 等待就绪 → 地图定位。"""
    if _already_running():
        logger.info("Livox 驱动已在运行，跳过重复启动。")
        return

    if not os.path.isfile(f"{WS_DIR}/devel/setup.bash"):
        logger.error("未找到 setup.bash，请先 catkin_make: %s", WS_DIR)
        return

    logger.info("=== 自启动开始 ===")
    time.sleep(DELAY_BEFORE_LIVOX)
    _start_node("Livox Driver", LIVOX_LAUNCH)
    logger.info("等待雷达节点初始化 %ds...", DELAY_AFTER_LIVOX)
    time.sleep(DELAY_AFTER_LIVOX)
    _start_node("Map Switch", PROJECT_LAUNCH)
    logger.info("=== 自启动完成 ===")


# === GPIO 监控主循环 ===
logger.info("GPIO监控启动: %s line %d", CHIP_NAME, LINE_OFFSET)

chip = gpiod.Chip(CHIP_NAME)
line = chip.get_line(LINE_OFFSET)
line.request(consumer="gpio_trigger", type=gpiod.LINE_REQ_DIR_IN)

triggered = False
last_state = 0
last_trigger_time = 0.0

try:
    while True:
        value = line.get_value()
        now = time.time()

        if value == 1 and last_state == 0 and (now - last_trigger_time) > DEBOUNCE:
            if triggered:
                logger.info("按键触发，但已启动过，忽略。")
            else:
                logger.info("按键触发，开始启动 ROS 模块。")
                start_ros()
                triggered = True
            last_trigger_time = now

        last_state = value
        time.sleep(CHECK_INTERVAL)

except KeyboardInterrupt:
    logger.info("GPIO监控脚本手动终止。")
except Exception as e:
    logger.error("运行异常: %s", e)
finally:
    line.release()
    chip.close()






# 原文件备份

# #!/usr/bin/env python3
# import gpiod
# import subprocess
# import time
# import logging

# # === GPIO 配置 ===       该引脚对应 orangepi5plus 板卡上的 GPIO1_A4 
# CHIP_NAME = "gpiochip1"
# LINE_OFFSET = 4
# SCRIPT_PATH = "/home/orangepi/slam_ws/auto_start.sh"
# DEBOUNCE = 0.5
# CHECK_INTERVAL = 0.05

# # === 启动标志 ===
# triggered = 0  # 0 表示未启动，1 表示已启动

# # === 日志设置 ===
# logger = logging.getLogger()
# logger.setLevel(logging.INFO)
# file_handler = logging.FileHandler('/home/orangepi/logs/gpio_trigger.log')
# formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
# file_handler.setFormatter(formatter)
# logger.addHandler(file_handler)
# console_handler = logging.StreamHandler()
# console_handler.setFormatter(formatter)
# logger.addHandler(console_handler)

# logger.info(f"GPIO监控启动: {CHIP_NAME} line {LINE_OFFSET}")

# # === 初始化 GPIO ===
# chip = gpiod.Chip(CHIP_NAME)
# line = chip.get_line(LINE_OFFSET)
# line.request(consumer="gpio_trigger", type=gpiod.LINE_REQ_DIR_IN)

# last_state = 0
# last_trigger_time = 0

# try:
#     while True:
#         value = line.get_value()
#         now = time.time()

#         # 低 -> 高 跳变
#         if value == 1 and last_state == 0 and (now - last_trigger_time) > DEBOUNCE:
#             if triggered == 1:
#                 logger.info("按键触发，但脚本已执行过，忽略。")
#             else:
#                 logger.info(f"按键触发，执行脚本：{SCRIPT_PATH}")
#                 subprocess.run(["/bin/bash", SCRIPT_PATH], check=False)
#                 triggered = 1  # 设置标志，确保只执行一次

#             last_trigger_time = now

#         last_state = value
#         time.sleep(CHECK_INTERVAL)

# except KeyboardInterrupt:
#     logger.info("GPIO监控脚本手动终止。")
# except Exception as e:
#     logger.error(f"运行异常: {e}")
# finally:
#     line.release()
#     chip.close()

# # 该版本按键触发对应orangepi5plus板卡

# # tail -f /home/orangepi/logs/gpio_trigger.log