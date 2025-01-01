# topic 으로 출발 도착
import cv2
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from rospy.executors import MultiThreadedExecutor
class Mamot_navigator:
    def __init__(self):
        rospy.init_node('mamot_navigator')

        self.plate_sub = rospy.Subscriber('/topic_name', String, self.plate_callback)
        self.tire_sub = rospy.Subscriber('/topic_name', String, self.thread_callback)
        self.test_sub = rospy.Subscriber('/test_topic', String, self.thread_callback)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.plate_sub)
        self.executor.add_node(self.tire_sub)
        self.executor.add_node(self.test_sub)



    def plate_callback(self, msg):
        rospy.loginfo("plate done")
        self.send_goal(1)

    def tire_callback(self, msg):
        rospy.loginfo("tire done")
        self.send_goal(2)

    def test_callback(self, msg):
        rospy.loginfo("test done")
        self.send_goal(0)

    def send_goal(self, pos_id):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if pos_id == 1:
            goal.target_pose.pose.position.x = 1.0  
            goal.target_pose.pose.position.y = 2.0  
            goal.target_pose.pose.orientation.w = 1.0
            rospy.loginfo("Sending AGV to location 1")

        elif pos_id == 2:
            goal.target_pose.pose.position.x = 3.0 
            goal.target_pose.pose.position.y = 4.0  
            goal.target_pose.pose.orientation.w = 1.0
            rospy.loginfo("Sending AGV to location 2")

        elif pos_id == 0:
            goal.target_pose.pose.position.x = 3.0 
            goal.target_pose.pose.position.y = 4.0  
            goal.target_pose.pose.orientation.w = 1.0
            rospy.loginfo("Send Goal to ")

        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(f"Mamot reached {pos_id}")

    def start(self):
        try:
            self.executor.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation interrupted")

if __name__ == '__main__':
    mamot_navigator = Mamot_navigator()
    mamot_navigator.start()