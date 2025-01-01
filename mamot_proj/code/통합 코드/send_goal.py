#!/usr/bin/env python
import rospy
import roslibpy
import os
import subprocess
import cv2
import cv2.aruco as aruco
import numpy as np
import base64
import time
import threading
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *

ros_client = roslibpy.Ros(host='172.30.1.100', port=9090)
ros_client.run()

agv_talker = roslibpy.Topic(ros_client, '/position_set', 'std_msgs/String')
agv_listener = roslibpy.Topic(ros_client, '/position_cmd', 'std_msgs/String')

exit_fg =0
status_fg = 0
position_fg = 0
move_fg = 0

def bridge_thread():
    global exit_fg, status_fg, position_fg
    def agv_listener_callback(msg):
        global move_fg
        if msg['data'] == '1':
            print('1. Mamot Start, Moving to photo spot')
            move_fg = 1

        elif msg['data'] == '2':
            print("2. Moving next to tire position")
            move_fg = 2
    
        elif msg['data'] == '3':
            print("3. Moving next to tire position")
            move_fg = 3

    agv_listener.subscribe(agv_listener_callback)
    while not exit_fg:
        # 1. Mamot Go
        while status_fg == 0:
            if position_fg == 1:
                print("Mamot at plate position and wait next cmd")
                agv_talker.publish(roslibpy.Message({'data': '1'}))
                status_fg = 1
            else:
                pass
        while status_fg == 1:
            if position_fg == 2:
                print("Mamot at tire position and wait next cmd")
                agv_talker.publish(roslibpy.Message({'data': '2'}))
                status_fg = 2
            else:
                pass
        while status_fg == 2:
            if position_fg == 3:
                print("Mamot at charge position ")
                exit_fg = True
                break
            else:
                pass


def move_thread():
    global exit_fg, status_fg, position_fg, move_fg
    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_client.wait_for_server()

    def feedback_callback(msg):
        rospy.loginfo(f'Moving ..')

    def send_goal(pose_id):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if pose_id == 1:
            # go to plate (2.00, -0.39)
            goal.target_pose.pose.position.x = 0.76 
            goal.target_pose.pose.position.y = -0.01 
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = -0.707
            goal.target_pose.pose.orientation.w = 0.707

        elif pose_id == 2:
            # go to tire (2.34, 0.20)
            goal.target_pose.pose.position.x = 1.58
            goal.target_pose.pose.position.y = -0.65
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = -0.707
            goal.target_pose.pose.orientation.w = 0.700

        elif pose_id == 3:
            # go to Aruco place
            goal.target_pose.pose.position.x = 0.76
            goal.target_pose.pose.position.y = 0.0  
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.99
            goal.target_pose.pose.orientation.w = 0.02

        move_client.send_goal(goal=goal, feedback_cb=feedback_callback)
        move_client.wait_for_result()
        rospy.loginfo(f"Mamot reached {pose_id}")
        
    while not exit_fg:
        if move_fg == 1:
            # time.sleep(1)
            send_goal(1)
            position_fg = 1
            move_fg = None
        elif move_fg == 2:
            # time.sleep(1)
            send_goal(2)
            position_fg = 2
            move_fg = None
        elif move_fg == 3:
            # time.sleep(1)
            send_goal(3)
            position_fg = 3
            move_fg = None

if __name__ == '__main__':
    rospy.init_node('move_test', anonymous=True)
    bridge_thread = threading.Thread(target=bridge_thread)
    move_thread = threading.Thread(target=move_thread)

    bridge_thread.start()
    move_thread.start()

    bridge_thread.join()
    move_thread.join()
        
