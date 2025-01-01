#!/usr/bin/env python
import cv2
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import String

def feedback_callback(msg):
    rospy.loginfo("[Feedback] going to Goal Pose...")

rospy.init_node('move_base_action_client')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'  
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 2.0  
goal.target_pose.pose.position.y = -1.8 
goal.target_pose.pose.position.z = 0.0

goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.0
goal.target_pose.pose.orientation.w = 1.0

client.send_goal(goal=goal, feedback_cb=feedback_callback)
client.wait_for_result()

rospy.loginfo(f"[Result] State: {client.get_state()}")