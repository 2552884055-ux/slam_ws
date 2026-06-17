#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RViz窗口捕获节点
捕获RViz窗口并发布为ROS图像话题
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mss
import mss.tools

class RVizCapture:
    def __init__(self):
        rospy.init_node('rviz_capture', anonymous=True)
        
        # 参数配置
        self.output_topic = rospy.get_param('~output_topic', '/rviz/image')
        self.fps = rospy.get_param('~fps', 30)
        self.monitor_index = rospy.get_param('~monitor_index', 1)  # 0=全屏, 1=主显示器
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 图像发布器
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        
        # 屏幕捕获
        self.sct = mss.mss()
        
        # 定时器
        self.rate = rospy.Rate(self.fps)
        
        rospy.loginfo(f"RViz捕获节点已启动，发布到: {self.output_topic}")
        rospy.loginfo(f"帧率: {self.fps} FPS")
    
    def capture_and_publish(self):
        """捕获屏幕并发布"""
        while not rospy.is_shutdown():
            try:
                # 捕获屏幕
                monitor = self.sct.monitors[self.monitor_index]
                screenshot = self.sct.grab(monitor)
                
                # 转换为numpy数组
                img = np.array(screenshot)
                
                # 转换颜色空间 BGRA -> BGR
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                
                # 转换为ROS消息
                ros_image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = 'rviz_capture'
                
                # 发布
                self.image_pub.publish(ros_image)
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"捕获错误: {e}")
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        capture = RVizCapture()
        capture.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
