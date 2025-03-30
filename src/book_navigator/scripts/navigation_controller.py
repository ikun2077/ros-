#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from book_navigator.srv import GetTarget, GetTargetRequest
from tf.transformations import quaternion_from_euler

class NavigationController:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # 获取当前位置（需AMCL运行）
        self.current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        
        # 订阅条码结果
        rospy.Subscriber('/detected_barcode', Barcode, self.navigate_to_target)

    def navigate_to_target(self, msg):
        # 查询数据库
        try:
            get_target = rospy.ServiceProxy('/get_target', GetTarget)
            response = get_target(msg.data)
            
            if response.exists:
                goal = self.create_goal(response.target_x, response.target_y)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                self.trigger_completion()
        except Exception as e:
            rospy.logerr(f"导航错误: {str(e)}")

    def create_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, 0)  # 保持默认朝向
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def trigger_completion(self):
        # 触发语音通知
        rospy.ServiceProxy('/voice_notify', SetVoice)("到达目标位置")

if __name__ == '__main__':
    rospy.init_node('navigation_controller')
    NavigationController()
    rospy.spin()