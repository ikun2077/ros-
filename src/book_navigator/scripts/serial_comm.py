#!/usr/bin/env python3
import rospy
import serial
from serial.serialutil import SerialException
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class SerialCommunicator:
    def __init__(self):
        # 串口参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        self.timeout = rospy.get_param('~timeout', 1.0)
        
        # 路径配置
        self.precision = rospy.get_param('~coord_precision', 3)  # 小数点后位数
        
        # 初始化串口连接
        self.ser = None
        self.connect_serial()
        
        # ROS订阅
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            rospy.loginfo(f"Connected to serial port {self.port}")
        except SerialException as e:
            rospy.logerr(f"Serial connection failed: {str(e)}")
            raise

    def path_callback(self, msg):
        if not self.ser or not self.ser.is_open:
            rospy.logwarn("Serial port not available, skipping path transmission")
            return

        try:
            # 构建传输协议
            data = "BEGIN_PATH\n"
            for idx, pose in enumerate(msg.poses):
                x = round(pose.pose.position.x, self.precision)
                y = round(pose.pose.position.y, self.precision)
                data += f"{idx},{x},{y}\n"
            data += "END_PATH\n"
            
            self.ser.write(data.encode('utf-8'))
            rospy.loginfo(f"Sent {len(msg.poses)} path points to STM32")
            
        except SerialException as e:
            rospy.logerr(f"Serial write error: {str(e)}")
            self.reconnect_serial()

    def reconnect_serial(self):
        rospy.loginfo("Attempting serial reconnection...")
        try:
            if self.ser:
                self.ser.close()
            self.connect_serial()
        except SerialException as e:
            rospy.logerr(f"Reconnection failed: {str(e)}")

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    rospy.init_node('serial_communicator')
    try:
        sc = SerialCommunicator()
        rospy.spin()
    except SerialException as e:
        rospy.logerr(f"Serial node failed: {str(e)}")