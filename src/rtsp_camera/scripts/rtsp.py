#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from urllib.parse import quote

class DualChannelRtspPublisher:
    def __init__(self):
        rospy.init_node("rtsp_publisher_single_ip")

        # ===== 基础参数（单 IP）=====
        self.ip = rospy.get_param("~ip", "192.168.1.64")
        self.port = int(rospy.get_param("~rtsp_port", 554))
        self.username = rospy.get_param("~username", "admin")
        self.password = rospy.get_param("~password", "Admin@123")  # 可含 @，会自动 URL 编码
        # 常见：可见光=101，热成像=201 或 301（视机型而定；可在相机 Web 管理页→视频→通道里查看）
        self.visible_channel = str(rospy.get_param("~visible_channel", "101"))
        self.thermal_channel = str(rospy.get_param("~thermal_channel", "201"))

        # 使能（如果暂时没有热成像通道可先关掉）
        self.enable_visible = bool(rospy.get_param("~enable_visible", True))
        self.enable_thermal = bool(rospy.get_param("~enable_thermal", True))

        # 输出话题
        self.topic_visible = rospy.get_param("~image_topic_visible", "/visible/image_raw")
        self.topic_thermal = rospy.get_param("~image_topic_thermal", "/thermal/image_raw")

        # 频率/尺寸（<=0 表示不 resize，按原始分辨率发布）
        self.frame_rate = float(rospy.get_param("~frame_rate", 20.0))
        self.out_w = int(rospy.get_param("~frame_width", -1))
        self.out_h = int(rospy.get_param("~frame_height", -1))

        self.bridge = CvBridge()
        self.pub_visible = None
        self.pub_thermal = None

        if self.enable_visible:
            self.pub_visible = rospy.Publisher(self.topic_visible, Image, queue_size=10)
        if self.enable_thermal:
            self.pub_thermal = rospy.Publisher(self.topic_thermal, Image, queue_size=10)

        if not (self.enable_visible or self.enable_thermal):
            rospy.logerr("两个通道都被禁用，直接退出。请至少开启一个通道。")
            rospy.signal_shutdown("no channels enabled")
            return

        # 线程
        self.threads = []
        if self.enable_visible:
            self.threads.append(threading.Thread(
                target=self._stream_loop,
                args=("visible", self.visible_channel, self.pub_visible, "visible_frame"),
                daemon=True))
        if self.enable_thermal:
            self.threads.append(threading.Thread(
                target=self._stream_loop,
                args=("thermal", self.thermal_channel, self.pub_thermal, "thermal_frame"),
                daemon=True))

    def _build_rtsp_url(self, channel: str) -> str:
        # 对密码做 URL 编码，避免 @ 等字符导致解析错误
        enc_pwd = quote(self.password, safe="")
        return f"rtsp://{self.username}:{enc_pwd}@{self.ip}:{self.port}/Streaming/Channels/{channel}"

    @staticmethod
    def _open_capture(url: str):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)  # 显式使用 FFMPEG 后端更稳
        # 降低延迟：让内部缓冲尽量小
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap if cap.isOpened() else None

    def _stream_loop(self, tag: str, channel: str, pub: rospy.Publisher, frame_id: str):
        """
        tag: 日志标识（'visible' / 'thermal'）
        channel: 通道号（如 '101', '201', '301'）
        """
        rate = rospy.Rate(self.frame_rate)
        url = self._build_rtsp_url(channel)
        cap = self._open_capture(url)
        last_retry = rospy.Time.now()

        if cap:
            rospy.loginfo(f"[{tag}] 打开成功：{url}")
        else:
            rospy.logwarn(f"[{tag}] 初次打开失败，将周期性重连：{url}")

        while not rospy.is_shutdown():
            if cap:
                ok, frame = cap.read()
                if ok:
                    # 可选 resize（保持算法输入一致；注意这会改变像素尺寸）
                    if self.out_w > 0 and self.out_h > 0:
                        frame = cv2.resize(frame, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = frame_id
                    pub.publish(msg)
                    rate.sleep()
                    continue

                # 连续失败：释放等待重连
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None

            # 重连（例如每 3 秒一次）
            if (rospy.Time.now() - last_retry).to_sec() >= 3.0:
                rospy.logwarn(f"[{tag}] 尝试重连：{url}")
                cap = self._open_capture(url)
                last_retry = rospy.Time.now()

            rate.sleep()

    def run(self):
        for th in self.threads:
            th.start()
        rospy.loginfo("rtsp_publisher_single_ip 启动完成。按 Ctrl+C 退出。")
        try:
            for th in self.threads:
                th.join()
        except KeyboardInterrupt:
            pass
        finally:
            rospy.loginfo("节点退出。")

if __name__ == "__main__":
    node = DualChannelRtspPublisher()
    if not rospy.is_shutdown():
        node.run()
