#!/usr/bin/env python3
import gpiod
import subprocess
import time
import logging

# === GPIO 配置 ===       该引脚对应 orangepi5plus 板卡上的 GPIO1_A4 
CHIP_NAME = "gpiochip1"
LINE_OFFSET = 4
SCRIPT_PATH = "/home/orangepi/slam_ws/auto_start.sh"
DEBOUNCE = 0.5
CHECK_INTERVAL = 0.05

# === 启动标志 ===
triggered = 0  # 0 表示未启动，1 表示已启动

# === 日志设置 ===
logger = logging.getLogger()
logger.setLevel(logging.INFO)
file_handler = logging.FileHandler('/home/orangepi/logs/gpio_trigger.log')
formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

logger.info(f"GPIO监控启动: {CHIP_NAME} line {LINE_OFFSET}")

# === 初始化 GPIO ===
chip = gpiod.Chip(CHIP_NAME)
line = chip.get_line(LINE_OFFSET)
line.request(consumer="gpio_trigger", type=gpiod.LINE_REQ_DIR_IN)

last_state = 0
last_trigger_time = 0

try:
    while True:
        value = line.get_value()
        now = time.time()

        # 低 -> 高 跳变
        if value == 1 and last_state == 0 and (now - last_trigger_time) > DEBOUNCE:
            if triggered == 1:
                logger.info("按键触发，但脚本已执行过，忽略。")
            else:
                logger.info(f"按键触发，执行脚本：{SCRIPT_PATH}")
                subprocess.run(["/bin/bash", SCRIPT_PATH], check=False)
                triggered = 1  # 设置标志，确保只执行一次

            last_trigger_time = now

        last_state = value
        time.sleep(CHECK_INTERVAL)

except KeyboardInterrupt:
    logger.info("GPIO监控脚本手动终止。")
except Exception as e:
    logger.error(f"运行异常: {e}")
finally:
    line.release()
    chip.close()

# 该版本按键触发对应orangepi5plus板卡

# tail -f /home/orangepi/logs/gpio_trigger.log