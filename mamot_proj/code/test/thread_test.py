#!/usr/bin/env python
import rospy
import threading
import time
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import *
from pymycobot.myagv import MyAgv
move_fg = 0
exit_fg = False
rospy.init_node('move_base_action_client', anonymous=False)

def fir_thread():
    global move_fg
    global exit_fg
    while not exit_fg:
        print("11111")
        time.sleep(3)
        move_fg=1
        break
    print("1_thread end")


def sec_thread():
    global exit_fg  
    while not exit_fg:
        print("22222")
        time.sleep(1)
    print("2_thread end")

def thr_thread():
    global move_fg
    global exit_fg

    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_client.wait_for_server()

    def feedback_callback(msg):
        pass

    def send_goal(pose_id): #x,y,z,w
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if pose_id == 1:
            goal.target_pose.pose.position.x = 2.00
            goal.target_pose.pose.position.y = 0.37
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

        elif pose_id == 2:
            # (,)
            goal.target_pose.pose.position.x = 2.27
            goal.target_pose.pose.position.y = 0.37 
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 1.0
            goal.target_pose.pose.orientation.w = 0.0

        move_client.send_goal(goal=goal, feedback_cb=feedback_callback)
        move_client.wait_for_result()
        rospy.loginfo(f"Mamot reached {pose_id}")
        
    while not exit_fg:
        if move_fg == 1:
            send_goal(1)
            time.sleep(1)
            send_goal(2)
            time.sleep(1)
            print("slam end")
            exit_fg=True
    print("3_thread end")

def manual_move():
    agv =MyAgv("/dev/ttyAMA2", 115200)
    print("manual start")
    time.sleep(1)
    agv.go_ahead(3)
    time.sleep(1)
    agv.retreat(3)
    time.sleep(1)
    agv.pan_right(3)
    time.sleep(2)
    agv.restore()
    print("manual end")

        
fir_thread = threading.Thread(target=fir_thread)
sec_thread = threading.Thread(target=sec_thread)
thr_thread = threading.Thread(target=thr_thread)

fir_thread.start()
sec_thread.start()
thr_thread.start()

fir_thread.join()
sec_thread.join()
thr_thread.join()

print("MANUAL moad")
manual_move()
print("All finished")


