#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import subprocess
import threading
from nav_msgs.msg import OccupancyGrid
import time

# ---------------- 配置 ----------------
RTSP_URL = "rtsp://127.0.0.1:8554/mapping"
FPS = 15
OUT_WIDTH = 800
OUT_HEIGHT = 600

latest_map = None
map_lock = threading.Lock()

# ---------------- 将 ROS 地图转换为 BGR 图像 ----------------
def map_to_image(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    data = np.array(map_msg.data).reshape((height, width))
    data = np.flipud(data)  # 上下翻转

    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[data == -1] = [128, 128, 128]  # 未知
    img[data == 0] = [255, 255, 255]   # 空
    img[data > 0] = [0, 0, 0]          # 占用

    # 缩放逻辑：大于输出尺寸按比例缩小
    scale = min(OUT_WIDTH / width, OUT_HEIGHT / height, 1.0)
    if scale < 1.0:
        new_w = int(width * scale)
        new_h = int(height * scale)
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

    # 放入 1920x1080 黑色画布中央
    canvas = np.zeros((OUT_HEIGHT, OUT_WIDTH, 3), dtype=np.uint8)
    h_offset = (OUT_HEIGHT - img.shape[0]) // 2
    w_offset = (OUT_WIDTH - img.shape[1]) // 2
    canvas[h_offset:h_offset+img.shape[0], w_offset:w_offset+img.shape[1]] = img

    return canvas

# ---------------- ROS 回调 ----------------
def map_callback(msg):
    global latest_map
    with map_lock:
        latest_map = map_to_image(msg)

# ---------------- FFmpeg 推流线程 ----------------
def streaming_loop():
    global latest_map
    # 等待第一帧地图
    while latest_map is None and not rospy.is_shutdown():
        time.sleep(0.05)

    height, width, _ = latest_map.shape

    # FFmpeg 命令
    ffmpeg_cmd = [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}',
        '-r', str(FPS),
        '-i', '-',                 # 从 stdin 输入
        '-c:v', 'libx264',
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-profile:v', 'baseline',
        '-pix_fmt', 'yuv420p',
        '-f', 'rtsp',
        '-rtsp_transport', 'tcp',
        RTSP_URL
    ]

    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    while not rospy.is_shutdown():
        with map_lock:
            frame = latest_map.copy()

        try:
            proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            rospy.logerr("FFmpeg pipe broken")
            break

        time.sleep(1.0 / FPS)

    proc.stdin.close()
    proc.wait()

# ---------------- 主程序 ----------------
if __name__ == "__main__":
    rospy.init_node("map_rtsp_publisher")
    rospy.Subscriber("/projected_map", OccupancyGrid, map_callback, queue_size=1)

    t = threading.Thread(target=streaming_loop)
    t.start()

    rospy.spin()
    t.join()
