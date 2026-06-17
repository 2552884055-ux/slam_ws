#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RTSP视频流服务器
订阅ROS图像话题并通过RTSP协议传输
"""

import sys
import rospy
from sensor_msgs.msg import Image
import cv2
import subprocess
import numpy as np
import threading
import queue

# 尝试导入cv_bridge，如果失败则使用手动转换
try:
    from cv_bridge import CvBridge
    USE_CV_BRIDGE = True
except Exception as e:
    rospy.logwarn(f"cv_bridge导入失败，使用手动转换: {e}")
    USE_CV_BRIDGE = False

class RTSPStreamServer:
    def __init__(self):
        rospy.init_node('rtsp_stream_server', anonymous=True)
        
        # 参数配置
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.rtsp_port = rospy.get_param('~rtsp_port', 8554)
        self.stream_name = rospy.get_param('~stream_name', 'mapping')
        self.fps = rospy.get_param('~fps', 30)
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.bitrate = rospy.get_param('~bitrate', '2M')
        
        # CV Bridge
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
        
        # 图像队列
        self.frame_queue = queue.Queue(maxsize=10)
        
        # FFmpeg进程
        self.ffmpeg_process = None
        
        # 启动FFmpeg RTSP服务器
        self.start_ffmpeg_server()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback,
            queue_size=1
        )
        
        # 启动发送线程
        self.send_thread = threading.Thread(target=self.send_frames)
        self.send_thread.daemon = True
        self.send_thread.start()
        
        rospy.loginfo(f"RTSP服务器已启动: rtsp://localhost:{self.rtsp_port}/{self.stream_name}")
        rospy.loginfo(f"订阅图像话题: {self.image_topic}")
    
    def start_ffmpeg_server(self):
        """启动FFmpeg RTSP服务器"""
        # FFmpeg命令：从stdin读取原始视频并输出为RTSP流
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', '-',  # 从stdin读取
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-b:v', self.bitrate,
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            f'rtsp://localhost:{self.rtsp_port}/{self.stream_name}'
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            rospy.loginfo("FFmpeg RTSP服务器进程已启动")
        except Exception as e:
            rospy.logerr(f"启动FFmpeg失败: {e}")
            rospy.logerr("请确保已安装FFmpeg: sudo apt-get install ffmpeg")
    
    def image_callback(self, msg):
        """ROS图像回调函数"""
        try:
            # 将ROS图像转换为OpenCV格式
            if USE_CV_BRIDGE and self.bridge:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # 手动转换ROS Image到OpenCV
                cv_image = self.ros_image_to_cv2(msg)
            
            # 调整图像大小
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # 添加时间戳和信息叠加
            timestamp = rospy.Time.now().to_sec()
            cv2.putText(cv_image, f"Time: {timestamp:.2f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 将帧放入队列
            if not self.frame_queue.full():
                self.frame_queue.put(cv_image)
            
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")
    
    def ros_image_to_cv2(self, msg):
        """手动将ROS Image消息转换为OpenCV图像"""
        if msg.encoding == 'rgb8':
            channels = 3
            dtype = np.uint8
        elif msg.encoding == 'bgr8':
            channels = 3
            dtype = np.uint8
        elif msg.encoding == 'mono8':
            channels = 1
            dtype = np.uint8
        elif msg.encoding == '16UC1':
            channels = 1
            dtype = np.uint16
        else:
            rospy.logerr(f"不支持的图像编码: {msg.encoding}")
            return None
        
        # 将数据转换为numpy数组
        img_array = np.frombuffer(msg.data, dtype=dtype)
        
        if channels == 1:
            img = img_array.reshape((msg.height, msg.width))
        else:
            img = img_array.reshape((msg.height, msg.width, channels))
        
        # 如果是RGB，转换为BGR
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        return img
    
    def send_frames(self):
        """发送帧到FFmpeg"""
        while not rospy.is_shutdown():
            try:
                if self.ffmpeg_process and self.ffmpeg_process.poll() is None:
                    # 从队列获取帧
                    frame = self.frame_queue.get(timeout=1.0)
                    
                    # 写入FFmpeg stdin
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    self.ffmpeg_process.stdin.flush()
                else:
                    rospy.logwarn("FFmpeg进程未运行，尝试重启...")
                    self.start_ffmpeg_server()
                    rospy.sleep(1.0)
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"发送帧错误: {e}")
    
    def cleanup(self):
        """清理资源"""
        if self.ffmpeg_process:
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()
        rospy.loginfo("RTSP服务器已关闭")

if __name__ == '__main__':
    try:
        server = RTSPStreamServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'server' in locals():
            server.cleanup()
