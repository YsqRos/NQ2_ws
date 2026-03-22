#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, Imu, NavSatFix
from cv_bridge import CvBridge
import os
import time
import rosbag
import threading


class DataSaver:
    def __init__(self):
        rospy.init_node('makedata', anonymous=True)

        # 参数
        self.rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://admin:123456@192.168.0.125:554/stream1')
        self.image_topic = rospy.get_param('~image_topic', '/rtsp_image')
        self.imu_topic = rospy.get_param('~imu_topic', '/wit/imu')
        self.gps_topic = rospy.get_param('~gps_topic', '/gps_bd09/fix')

        self.bag_path = rospy.get_param('~bag_path', '/tmp/rosbags/')  # rosbag存储路径
        if not os.path.exists(self.bag_path):
            os.makedirs(self.bag_path)
        self.bag_name = time.strftime("%Y%m%d-%H%M%S")  # 使用时间戳命名
        self.bag_file = os.path.join(self.bag_path, "{}.bag".format(self.bag_name))

        # 创建rosbag文件
        self.bag = rosbag.Bag(self.bag_file, 'w')

        # 创建CvBridge实例
        self.bridge = CvBridge()

        # 为每个话题创建独立的订阅者
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)

        # 确保在节点关闭时关闭 bag 文件
        rospy.on_shutdown(self.close)

        rospy.loginfo("开始保存数据到 {}".format(self.bag_file))

        # 创建一个锁以确保线程安全
        self.lock = threading.Lock()
        # 存储最新时间戳
        self.current_timestamp = None
        # 存储等待同一时间戳的消息
        self.image_cache = []
        self.imu_cache = []
        self.gps_cache = []

    def image_callback(self, image_msg):
        def log_info():
            if image_msg.header.seq % 10 == 0:
                rospy.loginfo("保存图像数据，时间戳: {:.6f}".format(image_msg.header.stamp.to_sec()))
        thread = threading.Thread(target=log_info)
        thread.start()
        with self.lock:
            if self.current_timestamp is None:
                self.current_timestamp = image_msg.header.stamp
                self.image_cache.append(image_msg)
            elif self.current_timestamp == image_msg.header.stamp:
                self.image_cache.append(image_msg)
            else:
                self.write_to_bag()
                self.current_timestamp = image_msg.header.stamp
                self.image_cache = [image_msg]

    def imu_callback(self, imu_msg):
        def log_info():
            if imu_msg.header.seq % 100 == 0:
                rospy.loginfo("保存IMU数据，时间戳: {:.6f}".format(imu_msg.header.stamp.to_sec()))
        thread = threading.Thread(target=log_info)
        thread.start()
        with self.lock:
            if self.current_timestamp is None:
                self.current_timestamp = imu_msg.header.stamp
                self.imu_cache.append(imu_msg)
            elif self.current_timestamp == imu_msg.header.stamp:
                self.imu_cache.append(imu_msg)
            else:
                self.write_to_bag()
                self.current_timestamp = imu_msg.header.stamp
                self.imu_cache = [imu_msg]

    def gps_callback(self, gps_msg):
        def log_info():
            if gps_msg.header.seq % 10 == 0:
                rospy.loginfo("保存GPS数据，时间戳: {:.6f}".format(gps_msg.header.stamp.to_sec()))
        thread = threading.Thread(target=log_info)
        thread.start()
        with self.lock:
            if self.current_timestamp is None:
                self.current_timestamp = gps_msg.header.stamp
                self.gps_cache.append(gps_msg)
            elif self.current_timestamp == gps_msg.header.stamp:
                self.gps_cache.append(gps_msg)
            else:
                self.write_to_bag()
                self.current_timestamp = gps_msg.header.stamp
                self.gps_cache = [gps_msg]

    def write_to_bag(self):
        try:
            for image_msg in self.image_cache:
                self.bag.write(self.image_topic, image_msg, t=self.current_timestamp)
            for imu_msg in self.imu_cache:
                self.bag.write(self.imu_topic, imu_msg, t=self.current_timestamp)
            for gps_msg in self.gps_cache:
                self.bag.write(self.gps_topic, gps_msg, t=self.current_timestamp)
            self.image_cache = []
            self.imu_cache = []
            self.gps_cache = []
        except Exception as e:
            rospy.logerr("保存数据时出错: {}".format(e))

    def run(self):
        """
        运行ROS节点，保持节点活跃直到被关闭。
        """
        rospy.spin()

    def close(self):
        """
        关闭rosbag文件。
        """
        with self.lock:
            self.write_to_bag()
            self.bag.close()
            rospy.loginfo("数据保存完成，rosbag已关闭: {}".format(self.bag_file))


if __name__ == '__main__':
    try:
        saver = DataSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
