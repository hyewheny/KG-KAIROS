#### plate bridge will combined
#!/usr/bin/env python
import rospy
import time
import actionlib
import tf
import numpy as np
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import *

def movebase_client(pos_id):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    def feedback_callback(msg):
        rospy.loginfo("[Feedback] going to Goal Pose...")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'  
    goal.target_pose.header.stamp = rospy.Time.now()
    
    if pos_id == 0:
            goal.target_pose.pose.position.x = 0.41 
            goal.target_pose.pose.position.y = -1.41
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo("pos_id:1  plate ")

    elif pos_id == 1:
        goal.target_pose.pose.position.x = 3.0 
        goal.target_pose.pose.position.y = 4.0  
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("pos_id: 2 _tire ")

    elif pos_id == 2:
        goal.target_pose.pose.position.x = 3.0 
        goal.target_pose.pose.position.y = 4.0  
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Send Goal to ")
   
    client.send_goal(goal=goal, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available")
        rospy.signal_shutdown("Action server not available")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done")
    except:
        rospy.loginfo("Navigation Finished")