#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
from sensor_msgs.msg import Image, Imu, MagneticField
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading

class ImageIMUSyncSaver:
    def __init__(self):
        # 初始化ROS节点 
        rospy.init_node('image_imu_sync_saver', anonymous=True)

        # 创建CvBridge对象
        self.bridge = CvBridge()

        # 获取参数
        self.save_dir = rospy.get_param('~save_dir', '/home/qxy/catkin_ws/src/rtsp_camera/saved_images')
        self.save_frequency = rospy.get_param('~save_frequency', 20)  # 每20帧保存一次 (20Hz)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)

        # 确保保存目录存在
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo("创建保存数据的目录: {}".format(self.save_dir))
        else:
            rospy.loginfo("数据将保存到: {}".format(self.save_dir))

        # 初始化订阅者
        image_topic = rospy.get_param('~image_topic', "/rtsp_image")
        imu_topic = rospy.get_param('~imu_topic', "/wit/imu")
        mag_topic = rospy.get_param('~mag_topic', "/wit/mag")

        rospy.loginfo("订阅的话题: %s, %s, %s" % (image_topic, imu_topic, mag_topic))

        # 使用message_filters订阅图像和IMU数据
        self.image_sub = Subscriber(image_topic, Image)
        self.imu_sub = Subscriber(imu_topic, Imu)
        self.mag_sub = Subscriber(mag_topic, MagneticField)

        # 设置同步器，使用 ApproximateTimeSynchronizer 进行近似同步
        ts = ApproximateTimeSynchronizer([self.image_sub, self.imu_sub, self.mag_sub], queue_size=20, slop=0.2)
        ts.registerCallback(self.sync_callback)

        # 初始化计数器
        self.frame_count = 0

        # 创建显示窗口
        self.window_name = "RTSP Stream"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.frame_width, self.frame_height)

        rospy.loginfo("Image and IMU Sync Saver Node 已启动。")

    def sync_callback(self, image_msg, imu_msg, mag_msg):
        rospy.loginfo("接收到同步消息")
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            rospy.loginfo("图像转换成功")
        except CvBridgeError as e:
            rospy.logerr("CvBridge 错误: {0}".format(e))
            return

        # 调整图像大小
        try:
            resized_image = cv2.resize(cv_image, (self.frame_width, self.frame_height))
            rospy.loginfo("图像调整大小成功")
        except Exception as e:
            rospy.logerr("图像调整大小失败: {0}".format(e))
            return

        # 显示图像
        try:
            cv2.imshow(self.window_name, resized_image)
            cv2.waitKey(1)  # 必须有，才能刷新窗口
            rospy.loginfo("图像显示成功")
        except Exception as e:
            rospy.logerr("图像显示失败: {0}".format(e))
            return

        # 增加帧计数
        self.frame_count += 1

        # 判断是否需要保存
        if self.frame_count % self.save_frequency == 0:
            rospy.loginfo("开始保存数据，帧数: {}".format(self.frame_count))
            # 获取时间戳
            stamp = image_msg.header.stamp
            time_fmt = datetime.fromtimestamp(stamp.to_sec()).strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 精确到毫秒

            # 生成文件路径
            image_filename = "image_{}.jpg".format(time_fmt)
            image_path = os.path.join(self.save_dir, image_filename)
            imu_filename = "imu_{}.txt".format(time_fmt)
            imu_path = os.path.join(self.save_dir, imu_filename)

            # 数据字典
            data = {
                'image': resized_image,
                'imu': imu_msg,
                'mag': mag_msg,
                'time_fmt': time_fmt
            }

            rospy.loginfo("生成保存数据的文件路径: {}".format(image_path))
            rospy.loginfo("生成保存IMU数据的文件路径: {}".format(imu_path))

            # 启动新线程保存数据
            save_thread = threading.Thread(target=self.save_data, args=(image_path, imu_path, data))
            save_thread.start()

    def save_data(self, image_path, imu_path, data):
        try:
            # 保存图像
            cv2.imwrite(image_path, data['image'])
            rospy.loginfo("保存图像到: {}".format(image_path))
        except Exception as e:
            rospy.logerr("保存图像失败: {}".format(e))

        try:
            # 保存 IMU 数据
            with open(imu_path, 'w') as f:
                f.write("Timestamp: {}\n".format(data['time_fmt']))

                # 方向（四元数）
                f.write("Orientation: x={}, y={}, z={}, w={}\n".format(
                    data['imu'].orientation.x,
                    data['imu'].orientation.y,
                    data['imu'].orientation.z,
                    data['imu'].orientation.w
                ))

                # 角速度
                f.write("Angular Velocity: x={}, y={}, z={}\n".format(
                    data['imu'].angular_velocity.x,
                    data['imu'].angular_velocity.y,
                    data['imu'].angular_velocity.z
                ))

                # 线性加速度
                f.write("Linear Acceleration: x={}, y={}, z={}\n".format(
                    data['imu'].linear_acceleration.x,
                    data['imu'].linear_acceleration.y,
                    data['imu'].linear_acceleration.z
                ))

                # 磁场
                f.write("Magnetic Field: x={}, y={}, z={}\n".format(
                    data['mag'].magnetic_field.x,
                    data['mag'].magnetic_field.y,
                    data['mag'].magnetic_field.z
                ))

            rospy.loginfo("保存 IMU 数据到: {}".format(imu_path))
        except Exception as e:
            rospy.logerr("保存 IMU 数据失败: {}".format(e))

    def shutdown_hook(self):
        rospy.loginfo("关闭 Image and IMU Sync Saver Node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        saver = ImageIMUSyncSaver()
        rospy.on_shutdown(saver.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

