#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import struct
import rospy
import math
import platform
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf.transformations import quaternion_from_euler
import threading
import pynmea2

class GPSPublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('gps_publisher', anonymous=True)

        # 创建一个发布者，用于发布 BD-09 坐标到 /gps_bd09/fix 话题
        self.bd09_pub = rospy.Publisher('/gps_bd09/fix', NavSatFix, queue_size=10)

        # 从 ROS 参数服务器获取串口参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 9600)

        # 尝试打开串口
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo("成功连接到串口: {}".format(self.port))
        except serial.SerialException as e:
            rospy.logerr("无法打开串口 {}: {}".format(self.port, e))
            rospy.signal_shutdown("串口连接失败")

        # 初始化缓冲区
        self.buffer = ""

        # 初始化 GPS_MAP 参数
        self.PI = 3.1415926535897932384626
        self.A = 6378245.0
        self.EE = 0.00669342162296594323

        # 初始化上一个已知的 GPS 数据
        self.last_fix = None
        self.fix_lock = threading.Lock()

        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        rospy.loginfo("GPS Publisher Node 已启动。")

    def str_To_Gps84(self, in_data1, in_data2):
        len_data1 = len(in_data1)
        str_data2 = "%05d" % int(in_data2)
        temp_data = int(in_data1)
        symbol = 1
        if temp_data < 0:
            symbol = -1
        degree = int(temp_data / 100.0)
        # 处理可能的索引超出范围
        if len_data1 < 2:
            rospy.logwarn("转换时 in_data1 长度无效: {}".format(in_data1))
            return 0.0
        str_decimal = in_data1[-2:] + '.' + str_data2
        try:
            f_degree = float(str_decimal)/60.0
        except ValueError:
            rospy.logwarn("in_data1 和 in_data2 转换无效: {}, {}".format(in_data1, in_data2))
            f_degree = 0.0
        if symbol > 0:
            result = degree + f_degree
        else:
            result = degree - f_degree
        return result

    def outOfChina(self, lat, lon):
        if (lon < 72.004 or lon > 137.8347):
            return True
        if (lat < 0.8293 or lat > 55.8271):
            return True
        return False

    def transformLat(self, x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * self.PI) + 20.0 * math.sin(2.0 * x * self.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * self.PI) + 40.0 * math.sin(y / 3.0 * self.PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * self.PI) + 320 * math.sin(y * self.PI / 30.0)) * 2.0 / 3.0
        return ret

    def transformLon(self, x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * self.PI) + 20.0 * math.sin(2.0 * x * self.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * self.PI) + 40.0 * math.sin(x / 3.0 * self.PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * self.PI) + 300.0 * math.sin(x / 30.0 * self.PI)) * 2.0 / 3.0
        return ret

    def gps84_to_gcj02(self, lat, lon):
        if self.outOfChina(lat, lon):
            return [lat, lon]
        dLat = self.transformLat(lon - 105.0, lat - 35.0)
        dLon = self.transformLon(lon - 105.0, lat - 35.0)
        radLat = lat / 180.0 * self.PI
        magic = math.sin(radLat)
        magic = 1 - self.EE * magic * magic
        sqrtMagic = math.sqrt(magic)
        dLat = (dLat * 180.0) / ((self.A * (1 - self.EE)) / (magic * sqrtMagic) * self.PI)
        dLon = (dLon * 180.0) / (self.A / sqrtMagic * math.cos(radLat) * self.PI)
        mgLat = lat + dLat
        mgLon = lon + dLon
        return [mgLat, mgLon]

    def gcj02_to_bd09(self, gg_lat, gg_lon):
        x = gg_lon
        y = gg_lat
        z = math.sqrt(x * x + y * y) + 0.00002 * math.sin(y * self.PI)
        theta = math.atan2(y, x) + 0.000003 * math.cos(x * self.PI)
        bd_lon = z * math.cos(theta) + 0.0065
        bd_lat = z * math.sin(theta) + 0.006
        return [bd_lat, bd_lon]

    def parse_gps_data(self, data):
        """
        解析 NMEA GPS 数据并发布为 NavSatFix 消息。
        """
        try:
            msg = pynmea2.parse(data)
            if isinstance(msg, pynmea2.types.talker.GGA):
                # 将 NMEA 坐标转换为十进制度
                latitude = self.convert_to_decimal_degrees(msg.lat, msg.lat_dir)
                longitude = self.convert_to_decimal_degrees(msg.lon, msg.lon_dir)

                # 进行坐标转换
                gcj02_data = self.gps84_to_gcj02(latitude, longitude)
                bd09_data = self.gcj02_to_bd09(gcj02_data[0], gcj02_data[1])

                # 创建 NavSatFix 消息用于 BD-09 坐标
                bd_fix = NavSatFix()
                bd_fix.header.stamp = rospy.Time.now()
                bd_fix.header.frame_id = "gps_bd09"

                # 设置 GPS 状态
                bd_fix.status.status = NavSatStatus.STATUS_FIX
                bd_fix.status.service = NavSatStatus.SERVICE_GPS

                # 设置 BD-09 的纬度、经度和高度（假设高度不变）
                bd_fix.latitude = bd09_data[0]
                bd_fix.longitude = bd09_data[1]
                bd_fix.altitude = float(msg.altitude)

                # 设置协方差（可选）
                bd_fix.position_covariance = [0.5, 0, 0,
                                             0, 0.5, 0,
                                             0, 0, 0.5]
                bd_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                # 发布 BD-09 消息
                with self.fix_lock:
                    self.last_fix = bd_fix

                rospy.loginfo("BD-09: Latitude=%.6f, Longitude=%.6f, Altitude=%.2f 米" %
                              (bd_fix.latitude, bd_fix.longitude, bd_fix.altitude))
        except pynmea2.ParseError:
            rospy.logwarn("解析 GPS 数据失败: {}".format(data))
        except Exception as e:
            rospy.logerr("处理 GPS 数据时出错: {}".format(e))

    def convert_to_decimal_degrees(self, raw_value, direction):
        """
        将原始 NMEA 纬度或经度转换为十进制度。
        """
        try:
            if not raw_value:
                return 0.0
            # raw_value 格式为 "ddmm.mmmm" 或 "dddmm.mmmm"
            degrees = int(float(raw_value) / 100)
            minutes = float(raw_value) - degrees * 100
            decimal_degrees = degrees + minutes / 60
            if direction in ['S', 'W']:
                decimal_degrees = -decimal_degrees
            return decimal_degrees
        except ValueError:
            rospy.logwarn("转换时原始值无效: {}".format(raw_value))
            return 0.0

    def read_serial(self):
        """
        读取串口数据并处理。
        """
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting:
                    # 读取所有传入数据
                    incoming_data = self.ser.read(self.ser.in_waiting)
                    # 将传入的字节解码为字符串，忽略错误
                    self.buffer += incoming_data.decode('utf-8', errors='ignore')

                    # 处理缓冲区中的所有完整行
                    while '\n' in self.buffer:
                        # 将缓冲区分割为一行和剩余缓冲区
                        line, self.buffer = self.buffer.split('\n', 1)
                        line = line.strip()

                        if line:
                            # 确保行以 '$' 开头，表示有效的 NMEA 句子
                            if not line.startswith('$'):
                                rospy.logwarn("接收到无效的 GPS 数据（不以 $ 开头）: {}".format(line))
                                continue

                            rospy.logdebug("接收到的 GPS 数据: {}".format(line))
                            self.parse_gps_data(line)
            except serial.SerialException as e:
                rospy.logerr("串口通信错误: {}".format(e))
                rospy.signal_shutdown("串口通信错误")
                break
            except Exception as e:
                rospy.logerr("未知错误: {}".format(e))
            rospy.sleep(0.001)  # Slight delay to prevent CPU hogging

    def publish_fix(self, event):
        """
        定时发布最新的 GPS 数据。
        """
        with self.fix_lock:
            if self.last_fix is not None:
                self.bd09_pub.publish(self.last_fix)
                rospy.logdebug("发布 GPS 数据，时间戳: {}".format(self.last_fix.header.stamp.to_sec()))
            else:
                rospy.logwarn("尚未接收到任何 GPS 数据，无法发布")

    def run(self):
        """
        运行 ROS 节点，启动定时发布器。
        """
        # 使用 Timer 每 0.05 秒（20Hz）发布一次 GPS 数据
        rospy.Timer(rospy.Duration(0.5), self.publish_fix)
        rospy.spin()

if __name__ == "__main__":
    try:
        gps_publisher = GPSPublisher()
        gps_publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS 发布节点已关闭。")
        pass

