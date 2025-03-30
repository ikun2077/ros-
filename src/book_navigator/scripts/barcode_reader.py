#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
from std_msgs.msg import String

class BarcodeReader:
    def __init__(self):
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 获取摄像头参数
        self.cam_device = rospy.get_param('~camera_device', '/dev/video0')
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.cam_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        
        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open camera device {self.cam_device}")
            raise RuntimeError("Camera initialization failed")
            
        # 设置发布者/订阅者
        self.barcode_pub = rospy.Publisher("/detected_barcode", String, queue_size=1)
        self.last_barcode = None
        self.barcode_timeout = rospy.Duration(5)  # 5秒内不重复检测相同条码

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame from camera")
            return
            
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 条码检测
            barcodes = pyzbar.decode(gray)
            
            current_time = rospy.Time.now()
            for barcode in barcodes:
                barcode_data = barcode.data.decode("utf-8")
                
                # 去重检测
                if self.last_barcode and \
                   (barcode_data == self.last_barcode[0]) and \
                   (current_time - self.last_barcode[1] < self.barcode_timeout):
                    continue
                    
                rospy.loginfo(f"New barcode detected: {barcode_data}")
                self.barcode_pub.publish(barcode_data)
                self.last_barcode = (barcode_data, current_time)
                
        except Exception as e:
            rospy.logerr(f"Barcode processing error: {str(e)}")

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            try:
                self.process_frame()
                rate.sleep()
            except KeyboardInterrupt:
                break
                
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('barcode_reader')
    reader = BarcodeReader()
    reader.run()