#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import rospy
import math
import platform
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler
from threading import Lock

def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

class ImuPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("imu_publisher", anonymous=True)
        
        # 获取参数
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baud", 9600)
        rospy.loginfo("IMU Publisher Node: Port=%s Baud=%d", self.port, self.baudrate)
        
        # 初始化串口
        try:
            rospy.loginfo("尝试打开串口: %s at %d baud.", self.port, self.baudrate)
            self.wt_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.wt_imu.isOpen():
                rospy.loginfo("\033[32m串口已成功打开...\033[0m")
            else:
                self.wt_imu.open()
                rospy.loginfo("\033[32m串口已成功打开...\033[0m")
        except Exception as e:
            rospy.logerr("串口打开失败: %s", str(e))
            rospy.signal_shutdown("串口打开失败")
            return
        
        # 初始化发布者
        self.imu_pub = rospy.Publisher("/wit/imu", Imu, queue_size=20)
        self.mag_pub = rospy.Publisher("/wit/mag", MagneticField, queue_size=20)
        
        # 初始化消息
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        
        # 初始化全局变量
        self.key = 0
        self.buff = {}
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.latest_data = {
            'acceleration': list(self.acceleration),  # 使用 list() 替代 copy()
            'angularVelocity': list(self.angularVelocity),
            'angle_degree': list(self.angle_degree),
            'magnetometer': list(self.magnetometer)
        }
        self.data_lock = Lock()
        
        # 确定Python版本
        self.python_version = platform.python_version()[0]
        rospy.loginfo("Python版本: %s", self.python_version)
        
        # 设置发布频率
        self.rate = rospy.Rate(15)  # 20 Hz
        
        rospy.loginfo("IMU Publisher Node 已启动，准备发布数据。")
        
    def handle_serial_data(self, raw_byte):
        angle_flag = False
        with self.data_lock:
            try:
                if self.python_version == '2':
                    byte_val = ord(raw_byte)
                elif self.python_version == '3':
                    byte_val = raw_byte
                else:
                    rospy.logwarn("未知的Python版本: %s", self.python_version)
                    return
            except Exception as e:
                rospy.logerr("处理字节时出错: %s", e)
                return
            
            self.buff[self.key] = byte_val
            self.key += 1
            rospy.logdebug("接收字节: 0x{:02X}, key: {}".format(byte_val, self.key))
            
            # 检查帧头
            if self.buff.get(0) != 0x55:
                if self.key > 0:
                    rospy.logwarn("帧头不正确: 0x{:02X}".format(self.buff.get(0)))
                self.key = 0
                self.buff.clear()
                return
            
            # 检查数据长度
            if self.key < 11:
                rospy.logdebug("数据未完整，当前长度: {}".format(self.key))
                return
            
            # 数据长度满足，解析数据
            data_buff = list(self.buff.values())
            msg_type = self.buff.get(1)
            rospy.logdebug("消息类型: 0x{:02X}".format(msg_type))
            
            if msg_type in [0x51, 0x52, 0x53, 0x54]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    rospy.logdebug("校验成功，开始解析数据")
                    parsed_data = hex_to_short(data_buff[2:10])
                    
                    if msg_type == 0x51:
                        # 加速度数据
                        self.acceleration = [
                            parsed_data[i] / 32768.0 * 16 * 9.8 for i in range(3)
                        ]
                        rospy.loginfo("加速度更新: x={:.3f}, y={:.3f}, z={:.3f}".format(
                            self.acceleration[0], self.acceleration[1], self.acceleration[2]))
                    
                    elif msg_type == 0x52:
                        # 角速度数据
                        self.angularVelocity = [
                            parsed_data[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)
                        ]
                        rospy.loginfo("角速度更新: x={:.3f}, y={:.3f}, z={:.3f}".format(
                            self.angularVelocity[0], self.angularVelocity[1], self.angularVelocity[2]))
                    
                    elif msg_type == 0x53:
                        # 角度数据
                        self.angle_degree = [
                            parsed_data[i] / 32768.0 * 180 for i in range(3)
                        ]
                        angle_flag = True
                        rospy.loginfo("角度更新: roll={:.2f}, pitch={:.2f}, yaw={:.2f}".format(
                            self.angle_degree[0], self.angle_degree[1], self.angle_degree[2]))
                    
                    elif msg_type == 0x54:
                        # 磁力计数据
                        self.magnetometer = parsed_data[0:3]
                        rospy.loginfo("磁力计更新: x={}, y={}, z={}".format(
                            self.magnetometer[0], self.magnetometer[1], self.magnetometer[2]))
                    
                    # 更新最新数据缓存
                    self.latest_data['acceleration'] = list(self.acceleration)  # 使用 list() 替代 copy()
                    self.latest_data['angularVelocity'] = list(self.angularVelocity)
                    self.latest_data['angle_degree'] = list(self.angle_degree)
                    self.latest_data['magnetometer'] = list(self.magnetometer)
                    
                else:
                    rospy.logwarn('消息类型 0x{:02X} 校验失败'.format(msg_type))
            else:
                rospy.logwarn("未知的消息类型: 0x{:02X}".format(msg_type))
            
            # 清空缓冲区
            self.buff.clear()
            self.key = 0
        
        if angle_flag:
            rospy.loginfo("角度数据已更新，准备发布IMU和磁力计数据。")
    
    def publish_data(self):
        with self.data_lock:
            stamp = rospy.get_rostime()
            
            # 设置IMU消息头
            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = "base_link"
            
            # 设置磁力计消息头
            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = "base_link"
            
            # 计算四元数
            angle_radian = [math.radians(angle) for angle in self.latest_data['angle_degree']]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
            
            # 设置IMU方向
            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]
            
            # 设置角速度
            self.imu_msg.angular_velocity.x = self.latest_data['angularVelocity'][0]
            self.imu_msg.angular_velocity.y = self.latest_data['angularVelocity'][1]
            self.imu_msg.angular_velocity.z = self.latest_data['angularVelocity'][2]
            
            # 设置线性加速度
            self.imu_msg.linear_acceleration.x = self.latest_data['acceleration'][0]
            self.imu_msg.linear_acceleration.y = self.latest_data['acceleration'][1]
            self.imu_msg.linear_acceleration.z = self.latest_data['acceleration'][2]
            
            # 设置磁力计数据
            self.mag_msg.magnetic_field.x = self.latest_data['magnetometer'][0]
            self.mag_msg.magnetic_field.y = self.latest_data['magnetometer'][1]
            self.mag_msg.magnetic_field.z = self.latest_data['magnetometer'][2]
            
            # 发布消息
            self.imu_pub.publish(self.imu_msg)
            self.mag_pub.publish(self.mag_msg)
            
            rospy.loginfo("发布 IMU 和磁场数据: 时间戳 {}".format(stamp))
    
    def run(self):
        rospy.loginfo("IMU Publisher Node 开始运行。")
        while not rospy.is_shutdown():
            try:
                buff_count = self.wt_imu.inWaiting()
                rospy.logdebug("串口缓存字节数: {}".format(buff_count))
            except Exception as e:
                rospy.logerr("读取串口数据失败: %s", str(e))
                rospy.logerr("IMU 连接断开")
                rospy.signal_shutdown("串口读取失败")
                break
            else:
                if buff_count > 0:
                    try:
                        buff_data = self.wt_imu.read(buff_count)
                        # 在Python 2中使用 encode('hex') 替代 .hex()
                        hex_data = buff_data.encode('hex')
                        rospy.logdebug("读取到 {} 字节的数据: {}".format(buff_count, hex_data))
                        for byte in buff_data:
                            self.handle_serial_data(byte)
                    except Exception as e:
                        rospy.logerr("读取串口数据时出错: %s", str(e))
                
            # 发布数据
            self.publish_data()
            
            self.rate.sleep()  # 等待直到下一次循环
    
        # 关闭串口
        self.wt_imu.close()
        rospy.loginfo("串口已关闭，节点退出。")

if __name__ == "__main__":
    try:
        imu_publisher = ImuPublisher()
        imu_publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断.")
    except Exception as e:
        rospy.logerr("发生异常: %s", str(e))

