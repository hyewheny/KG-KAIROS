#!/usr/bin/env python
import cv2
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import String

rospy.init_node('move_base_action_client')
move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_client.wait_for_server()
position_fg = 0

def feedback_callback(msg):
    rospy.loginfo(f'Moving ..')

def send_goal(pose_id):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if pose_id == 1:
        # go to plate (2.00, -0.30)
        goal.target_pose.pose.position.x = 1.45 
        goal.target_pose.pose.position.y = -0.39 
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

    elif pose_id == 2:
        # go to tire (2.34, 0.20)
        goal.target_pose.pose.position.x = 3.25
        goal.target_pose.pose.position.y = 0.41 
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

    elif pose_id == 3:
        # go to Aruco place
        goal.target_pose.pose.position.x = 1.27 
        goal.target_pose.pose.position.y = -0.60  
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = -0.7
        goal.target_pose.pose.orientation.w = 0.7
    
    elif pose_id == 4:
        # not using
        goal.target_pose.pose.position.x = 1.27 
        goal.target_pose.pose.position.y = -0.65  
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

    move_client.send_goal(goal=goal, feedback_cb=feedback_callback)
    move_client.wait_for_result()
    rospy.loginfo(f"Mamot reached {pose_id}")

time.sleep(5)
for i in range(1,4):
    send_goal(i)
    time.sleep(3)
