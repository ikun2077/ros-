#!/usr/bin/env python3
import rospy
import requests
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class NavigationController:
    def __init__(self):
        # Flask服务器配置
        self.flask_url = rospy.get_param('~flask_url', 'http://192.168.1.100:5000/arrived')
        
        # 状态跟踪
        self.current_goal = None
        self.last_status = None
        
        # ROS订阅/发布
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        
        # 语音反馈
        self.tts_pub = rospy.Publisher("/speech_feedback", String, queue_size=1)

    def goal_callback(self, msg):
        self.current_goal = msg.pose
        rospy.loginfo(f"New navigation goal received: {self.current_goal}")

    def status_callback(self, msg):
        if not msg.status_list:
            return
            
        status = msg.status_list[-1]
        
        # 状态变化检测
        if self.last_status != status.status:
            if status.status == GoalStatus.SUCCEEDED:
                self.handle_success()
            elif status.status == GoalStatus.ABORTED:
                self.handle_failure()
            elif status.status == GoalStatus.PREEMPTED:
                self.handle_cancel()
                
            self.last_status = status.status

    def handle_success(self):
        rospy.loginfo("Navigation succeeded! Notifying Flask server...")
        
        try:
            response = requests.post(self.flask_url, timeout=3)
            if response.status_code == 200:
                self.tts_pub.publish("目标已到达")
            else:
                rospy.logwarn(f"Flask server returned {response.status_code}")
        except Exception as e:
            rospy.logerr(f"Failed to contact Flask server: {str(e)}")
            self.tts_pub.publish("到达目标但通知失败")

    def handle_failure(self):
        error_msg = "导航失败，请检查路径或环境"
        rospy.logerr(error_msg)
        self.tts_pub.publish(error_msg)

    def handle_cancel(self):
        info_msg = "导航任务被取消"
        rospy.logwarn(info_msg)
        self.tts_pub.publish(info_msg)

if __name__ == '__main__':
    rospy.init_node('navigation_controller')
    nc = NavigationController()
    rospy.spin()