#!/usr/bin/env python3
import rospy
import sqlite3
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class DatabaseHandler:
    def __init__(self):
        # 数据库配置
        self.db_path = rospy.get_param('~database_path', 'database/book_db.sqlite3')
        self.conn = None
        self.cursor = None
        
        # 导航参数
        self.start_pose = self.load_start_pose()
        self.map_frame = rospy.get_param('~map_frame', 'map')
        
        # 初始化数据库连接
        self.connect_database()
        self.create_table()
        
        # ROS通信设置
        self.barcode_sub = rospy.Subscriber("/detected_barcode", String, self.handle_barcode)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    def load_start_pose(self):
        return {
            'x': rospy.get_param('~start_x', 0.0),
            'y': rospy.get_param('~start_y', 0.0),
            'z': rospy.get_param('~start_z', 0.0),
            'yaw': rospy.get_param('~start_yaw', 0.0)
        }

    def connect_database(self):
        try:
            self.conn = sqlite3.connect(self.db_path)
            self.cursor = self.conn.cursor()
            rospy.loginfo(f"Connected to database: {self.db_path}")
        except sqlite3.Error as e:
            rospy.logerr(f"Database connection error: {str(e)}")
            raise

    def create_table(self):
        try:
            self.cursor.execute('''CREATE TABLE IF NOT EXISTS books
                                (id TEXT PRIMARY KEY, 
                                 end_x REAL NOT NULL,
                                 end_y REAL NOT NULL,
                                 end_z REAL DEFAULT 0.0,
                                 end_yaw REAL DEFAULT 0.0)''')
            self.conn.commit()
        except sqlite3.Error as e:
            rospy.logerr(f"Table creation error: {str(e)}")

    def handle_barcode(self, msg):
        book_id = msg.data
        rospy.loginfo(f"Processing barcode: {book_id}")
        
        try:
            self.cursor.execute("SELECT end_x, end_y, end_z, end_yaw FROM books WHERE id=?", (book_id,))
            result = self.cursor.fetchone()
        except sqlite3.Error as e:
            rospy.logerr(f"Database query error: {str(e)}")
            return

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame
        
        if result:
            end_x, end_y, end_z, end_yaw = result
            q = quaternion_from_euler(0, 0, end_yaw)
            
            goal.pose.position.x = end_x
            goal.pose.position.y = end_y
            goal.pose.position.z = end_z
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            
            rospy.loginfo(f"Publishing goal: ({end_x}, {end_y})")
        else:
            q = quaternion_from_euler(0, 0, self.start_pose['yaw'])
            
            goal.pose.position.x = self.start_pose['x']
            goal.pose.position.y = self.start_pose['y']
            goal.pose.position.z = self.start_pose['z']
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            
            rospy.logwarn(f"Barcode {book_id} not found, returning to start")

        self.goal_pub.publish(goal)

    def __del__(self):
        if self.conn:
            self.conn.close()
            rospy.loginfo("Database connection closed")

if __name__ == '__main__':
    rospy.init_node('database_handler')
    try:
        dh = DatabaseHandler()
        rospy.spin()
    except sqlite3.Error as e:
        rospy.logerr(f"Database handler failed: {str(e)}")