#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread

class RTSPPublisher:
    def __init__(self):
        rospy.init_node('rtsp_publisher', anonymous=True)

        # 获取参数
        self.rtsp_url_1 = rospy.get_param('~rtsp_url_1', 'rtsp://admin:Admin@123@192.168.1.165:554/Streaming/Channels/101')
        self.rtsp_url_2 = rospy.get_param('~rtsp_url_2', 'rtsp://admin:Admin@123@192.168.1.64:554/h264/ch1/main/av_stream')
        
        self.image_topic_1 = rospy.get_param('~image_topic_1', '/rtsp_image_1')
        self.image_topic_2 = rospy.get_param('~image_topic_2', '/rtsp_image_2')

        self.frame_rate = rospy.get_param('~frame_rate', 20)  # 设定为20Hz
        self.frame_width = rospy.get_param('~frame_width', 1920)
        self.frame_height = rospy.get_param('~frame_height', 1080)

        # 创建两个发布者
        self.image_pub_1 = rospy.Publisher(self.image_topic_1, Image, queue_size=10)
        self.image_pub_2 = rospy.Publisher(self.image_topic_2, Image, queue_size=10)

        # 初始化 CvBridge
        self.bridge = CvBridge()

        # 打开两个 RTSP 流
        self.cap_1 = cv2.VideoCapture(self.rtsp_url_1)
        self.cap_2 = cv2.VideoCapture(self.rtsp_url_2)

        if not self.cap_1.isOpened():
            rospy.logerr("无法打开 RTSP 流: %s" % self.rtsp_url_1)
            rospy.signal_shutdown("无法打开 RTSP 流 1")
        else:
            rospy.loginfo("成功连接到 RTSP 流: %s" % self.rtsp_url_1)

        if not self.cap_2.isOpened():
            rospy.logerr("无法打开 RTSP 流: %s" % self.rtsp_url_2)
            rospy.signal_shutdown("无法打开 RTSP 流 2")
        else:
            rospy.loginfo("成功连接到 RTSP 流: %s" % self.rtsp_url_2)

    def publish_stream_1(self):
        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            ret, frame = self.cap_1.read()
            if not ret:
                rospy.logwarn("无法读取 RTSP 流 1 帧")
                continue
            try:
                # 调整图像大小
                frame_resized = cv2.resize(frame, (self.frame_width, self.frame_height))

                # 将 OpenCV 图像转换为 ROS 图像消息
                image_msg = self.bridge.cv2_to_imgmsg(frame_resized, "bgr8")
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = "rtsp_camera_1"

                # 发布图像消息
                self.image_pub_1.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge 错误 (流1): {0}".format(e))
            rate.sleep()

    def publish_stream_2(self):
        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            ret, frame = self.cap_2.read()
            if not ret:
                rospy.logwarn("无法读取 RTSP 流 2 帧")
                continue
            try:
                # 调整图像大小
                frame_resized = cv2.resize(frame, (self.frame_width, self.frame_height))

                # 将 OpenCV 图像转换为 ROS 图像消息
                image_msg = self.bridge.cv2_to_imgmsg(frame_resized, "bgr8")
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = "rtsp_camera_2"

                # 发布图像消息
                self.image_pub_2.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge 错误 (流2): {0}".format(e))
            rate.sleep()

    def start(self):
        # 使用两个线程同时发布两个流
        thread_1 = Thread(target=self.publish_stream_1)
        thread_2 = Thread(target=self.publish_stream_2)
        thread_1.start()
        thread_2.start()

        thread_1.join()
        thread_2.join()

        # 释放资源
        self.cap_1.release()
        self.cap_2.release()

if __name__ == '__main__':
    try:
        publisher = RTSPPublisher()
        publisher.start()
    except rospy.ROSInterruptException:
        pass

