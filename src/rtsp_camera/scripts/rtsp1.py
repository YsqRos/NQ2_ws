#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from threading import Thread
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# ---- URL 编码（Py3）----
try:
    from urllib.parse import quote as urlquote
except Exception:
    from urllib import quote as urlquote

def url_encode_pwd(pwd: str) -> str:
    return urlquote(str(pwd), safe='')

def build_ffmpeg_url(base_rtsp: str, ch: str, use_udp: bool) -> str:
    """
    为 OpenCV-FFmpeg 后端追加低延迟参数
    """
    sep = '&' if '?' in base_rtsp else '?'
    trans = 'udp' if use_udp else 'tcp'
    return (f"{base_rtsp.format(ch=ch)}{sep}"
            f"rtsp_transport={trans}"
            f"&stimeout=5000000"
            f"&max_delay=0"
            f"&buffer_size=102400")

def open_with_ffmpeg(base_rtsp: str, channels, try_udp_first=False):
    """
    按通道和传输方式尝试 RTSP，返回 (cap, chosen_channel, chosen_transport)
    """
    order = [True, False] if try_udp_first else [False, True]  # True=UDP, False=TCP
    for ch in channels:
        for use_udp in order:
            url = build_ffmpeg_url(base_rtsp, ch, use_udp)
            rospy.loginfo("尝试 RTSP：%s", url)
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 降低内部缓冲
                except Exception:
                    pass
                return cap, ch, ("udp" if use_udp else "tcp")
        rospy.logwarn("通道 %s 未连上，继续尝试…", ch)
    return None, None, None

class DualFFmpegRTSPNode:
    def __init__(self):
        rospy.init_node("ffmpeg_rtsp_dual_topics", anonymous=True)
        self.bridge = CvBridge()

        # ===== 发布样式：默认发 Image（方便 rviz） =====
        self.use_compress = rospy.get_param("~use_compressed", False)
        self.jpeg_q       = rospy.get_param("~jpeg_quality", 70)

        # ===== 相机1 参数（可见光） =====
        self.cam1_enable  = rospy.get_param("~cam1_enable", True)
        self.cam1_ip      = rospy.get_param("~cam1_ip", "192.168.1.64")
        self.cam1_user    = rospy.get_param("~cam1_user", "admin")
        self.cam1_pass    = rospy.get_param("~cam1_pass", "Admin@123")
        self.cam1_channels= rospy.get_param("~cam1_channels", ["101","102"])
        self.cam1_try_udp_first = rospy.get_param("~cam1_try_udp_first", False)
        self.topic1       = rospy.get_param("~image_topic_1", "/cam1/image")

        # ===== 相机2 参数（红外） =====
        self.cam2_enable  = rospy.get_param("~cam2_enable", True)
        self.cam2_ip      = rospy.get_param("~cam2_ip", "192.168.1.165")
        self.cam2_user    = rospy.get_param("~cam2_user", "admin")
        self.cam2_pass    = rospy.get_param("~cam2_pass", "Admin@123")
        # 🔑 全部尝试红外可能用到的通道
        self.cam2_channels= rospy.get_param("~cam2_channels", ["201","202","301","302","401","402"])
        self.cam2_try_udp_first = rospy.get_param("~cam2_try_udp_first", False)
        self.topic2       = rospy.get_param("~image_topic_2", "/cam2/image")

        # ===== Publisher =====
        if self.use_compress:
            self.pub1 = rospy.Publisher(self.topic1 + "/compressed", CompressedImage, queue_size=1)
            self.pub2 = rospy.Publisher(self.topic2 + "/compressed", CompressedImage, queue_size=1)
        else:
            self.pub1 = rospy.Publisher(self.topic1, Image, queue_size=1)
            self.pub2 = rospy.Publisher(self.topic2, Image, queue_size=1)

        # ===== 打开相机 =====
        self.cap1 = None; self.cap2 = None
        if self.cam1_enable:
            base1 = self._build_base_rtsp(self.cam1_user, self.cam1_pass, self.cam1_ip)
            rospy.loginfo("尝试连接相机1：%s", base1)
            self.cap1, ch1, tr1 = open_with_ffmpeg(base1, self.cam1_channels, self.cam1_try_udp_first)
            if self.cap1 is None:
                rospy.logerr("相机1连接失败：检查 IP/账号/密码/通道/网络。")
            else:
                rospy.loginfo("✅ 相机1连接成功：channel=%s transport=%s", ch1, tr1)

        if self.cam2_enable:
            base2 = self._build_base_rtsp(self.cam2_user, self.cam2_pass, self.cam2_ip)
            rospy.loginfo("尝试连接相机2：%s", base2)
            self.cap2, ch2, tr2 = open_with_ffmpeg(base2, self.cam2_channels, self.cam2_try_udp_first)
            if self.cap2 is None:
                rospy.logerr("相机2连接失败：请检查 IP=%s 通道=%s 用户=%s", self.cam2_ip, self.cam2_channels, self.cam2_user)
            else:
                rospy.loginfo("✅ 相机2连接成功：channel=%s transport=%s", ch2, tr2)

        if (not self.cam1_enable or self.cap1 is None) and (not self.cam2_enable or self.cap2 is None):
            rospy.signal_shutdown("no camera opened")

    @staticmethod
    def _build_base_rtsp(user, pwd, ip):
        pwd_enc = url_encode_pwd(pwd)
        return f"rtsp://{user}:{pwd_enc}@{ip}:554/Streaming/Channels/{{ch}}?transportmode=unicast"

    def _publish_one(self, frame, pub, frame_id):
        stamp = rospy.Time.now()
        if self.use_compress:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.format = "jpeg"
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_q)])
            if ok:
                msg.data = enc.tobytes()
                pub.publish(msg)
        else:
            img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img.header.stamp = stamp
            img.header.frame_id = frame_id if frame_id else "camera"
            pub.publish(img)

    def _loop(self, cap, pub, frame_id):
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                continue
            self._publish_one(frame, pub, frame_id)

    def start(self):
        threads = []
        if self.cap1 is not None:
            t1 = Thread(target=self._loop, args=(self.cap1, self.pub1, "camera1"))
            t1.daemon = True; t1.start(); threads.append(t1)
        if self.cap2 is not None:
            t2 = Thread(target=self._loop, args=(self.cap2, self.pub2, "camera2"))
            t2.daemon = True; t2.start(); threads.append(t2)

        for t in threads:
            t.join()

        if self.cap1 is not None: self.cap1.release()
        if self.cap2 is not None: self.cap2.release()

if __name__ == "__main__":
    try:
        node = DualFFmpegRTSPNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
