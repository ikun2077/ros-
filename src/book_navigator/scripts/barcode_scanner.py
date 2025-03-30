#!/usr/bin/env python3
import rospy
import cv2
import sqlite3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from book_navigator.msg import Barcode
from book_navigator.srv import GetTarget, GetTargetResponse

class BarcodeScanner:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)
        self.barcode_detector = cv2.barcode_BarcodeDetector()
        
        # 数据库连接
        self.db_conn = sqlite3.connect(rospy.get_param('~db_path'))
        self.cursor = self.db_conn.cursor()
        
        # 发布识别结果
        self.barcode_pub = rospy.Publisher('/detected_barcode', Barcode, queue_size=10)
        
        # 目标查询服务
        self.target_service = rospy.Service('/get_target', GetTarget, self.handle_target_query)

    def scan_loop(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()
            if ret:
                # 条码检测
                _, decoded, _, _ = self.barcode_detector.detectAndDecode(frame)
                if decoded:
                    barcode_msg = Barcode()
                    barcode_msg.data = decoded[0]
                    barcode_msg.header.stamp = rospy.Time.now()
                    self.barcode_pub.publish(barcode_msg)
            rate.sleep()

    def handle_target_query(self, req):
        res = GetTargetResponse()
        self.cursor.execute("SELECT x,y FROM locations WHERE barcode=?", (req.barcode,))
        result = self.cursor.fetchone()
        if result:
            res.target_x, res.target_y = result
            res.exists = True
        else:
            res.exists = False
        return res

if __name__ == '__main__':
    rospy.init_node('barcode_scanner')
    scanner = BarcodeScanner()
    scanner.scan_loop()