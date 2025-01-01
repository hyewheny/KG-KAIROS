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
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import *
from pymycobot.myagv import MyAgv

lidar_launch = "/home/er/myagv_ros/src/myagv_odometry/launch/myagv_active.launch"
navigation_launch = "/home/er/myagv_ros/src/myagv_navigation/launch/navigation_active.launch"

script_path = os.path.expanduser("/home/er/myagv_ros/src/myagv_odometry/scripts/close_ydlidar.sh")
ros_client = roslibpy.Ros(host='172.30.1.100', port=9090)
ros_client.run()

agv_talker = roslibpy.Topic(ros_client, '/plate_agv', 'std_msgs/String')
agv_listener = roslibpy.Topic(ros_client, '/plate_pc', 'std_msgs/String')
audio_talker = roslibpy.Topic(ros_client, '/audio_topic', 'std_msgs/String')

kill_nodes=['/move_base','/rviz','/ydlidar_lidar_publisher','/amcl','/base2imu_link','/base2camera_link','/base_link_to_imu_link','/base_link_to_laser_link','joint_state_publisher','map_server_for_test',
'/myagv_odometry_node','/robot_pose_ekf','robot_state_publisher','/rosapi']
class Thread_mode():
    def __init__(self):
        rospy.init_node('agv_node', anonymous=False)
        self.exit_fg = False 
        self.position_fg = 0
        self.status_fg = 0
        self.move_fg = 0
    def kill_node(self, kill_nodes):
        for node in kill_nodes:
            try:
                subprocess.run(['rosnode','kill',node], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")

    def cam_thread(self):   
        print(f'cam thread : {self.status_fg}')
        cap = cv2.VideoCapture(0)
        while not self.exit_fg:
            if self.status_fg == 1:
                ret, frame = cap.read()
                if not ret:
                    print("ret error")
                    break
                time.sleep(3)
                img_name = f"/home/er/catkin_ws/src/mamot/src/image/plate_img.png"
                cv2.imwrite(img_name, frame)
                self.status_fg = 2
                print('cam thread : Captured img from agv cam, changed fg')
                break
        print("cam thread end")
        cap.release()
        cv2.destroyAllWindows()    

    def bridge_thread(self):
        def agv_listener_callback(msg):
                if msg['data'] == '1':
                    # [Rin] Start SLAM Nav Code
                    print('1. Mamot Start, Moving to photo spot')
                    self.move_fg = 1
                    audio_talker.publish(roslibpy.Message({'data':'1'}))

                elif msg['data'] == '2':
                    # [Rin] Move next to tire SLAM Nav Code
                    print("2. Moving next to tire position")
                    self.move_fg = 2
            
                elif msg['data'] == '3':
                    # [Rin] Move next to home
                    print("3. Moving next to tire position")
                    self.move_fg = 3

        while not self.exit_fg:
            agv_listener.subscribe(agv_listener_callback)
            # 1. Mamot Go
            while self.status_fg == 0:
                if self.position_fg == 1:
                    self.status_fg = 1
            while self.status_fg == 1:
                pass
            # 2. Get Plate Img
            # [Rin] Camera On, Take Photo & Save
            while self.status_fg == 2:
                plate_img = cv2.imread('/home/er/catkin_ws/src/mamot/src/image/plate_img.png')
                _, buffer = cv2.imencode('.jpg', plate_img)
                plate_msg = 'aaa' + base64.b64encode(buffer).decode('utf-8')
                agv_talker.publish(roslibpy.Message({'data': plate_msg}))
                audio_talker.publish(roslibpy.Message({'data':'2'}))
                print('2. Published plate image')
                self.status_fg = 3

            # 3. Confirm Plate Num & move tire position
            while self.status_fg == 3:
                if self.position_fg == 2:
                    agv_talker.publish(roslibpy.Message({'data':'bbb'}))
                    audio_talker.publish(roslibpy.Message({'data':'3'}))
                    time.sleep(5)        
                    self.status_fg = 4

            # 4. Request Service to Cobot
            while self.status_fg == 4:
                if self.move_fg == 3:
                    audio_talker.publish(roslibpy.Message({'data':'4'}))
                    self.status_fg = 5

            # 5. 
            while self.status_fg == 5:
                if self.position_fg == 3:
                    audio_talker.publish(roslibpy.Message({'data':'5'}))
                    self.kill_node(kill_nodes)
                    # lidar_process = subprocess.Popen(["roslaunch",lidar_launch])
                    # lidar_process.kill()
                    # print("close lidar launch")
                    # subprocess.run(['bash', script_path], check=True)
                    # print("close ydlidar")
                    # navigation_process = subprocess.Popen(["roslaunch",navigation_launch])
                    # navigation_process.kill()
                    print("close navigation launch")
                    time.sleep(1)        
                    self.exit_fg = True
                    break
                

    def move_thread(self):
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
                goal.target_pose.pose.position.x = 1.45 
                goal.target_pose.pose.position.y = -0.48 
                goal.target_pose.pose.position.z = 0.0

                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0

            elif pose_id == 2:
                # go to tire (2.34, 0.20)
                goal.target_pose.pose.position.x = 3.25
                goal.target_pose.pose.position.y = 0.38 
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

            move_client.send_goal(goal=goal, feedback_cb=feedback_callback)
            move_client.wait_for_result()
            rospy.loginfo(f"Mamot reached {pose_id}")
            
        while not self.exit_fg:
            if self.move_fg == 1:
                time.sleep(1)
                send_goal(1)
                self.position_fg = 1
                self.move_fg = None
            elif self.move_fg == 2:
                time.sleep(1)
                send_goal(2)
                self.position_fg = 2
                self.move_fg = None
            elif self.move_fg == 3:
                time.sleep(1)
                send_goal(3)
                self.position_fg = 3
                self.move_fg = None


            
                def manual_mode():
                    agv = MyAgv("/dev/ttyAMA2", 115200)
                    camera_matrix = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/camera_matrix.npy")
                    dist_coeffs = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/dist_coeffs.npy")
                    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
                    parameters = aruco.DetectorParameters()
                    marker_length = 0.0195


                    def adjust_distance(distance):
                        if distance < 1.25:
                            agv.retreat(3,0.5)
                        elif distance > 1.45:
                            agv.go_ahead(3,0.5)

                    def go_charging():
                        agv.counterclockwise_rotation(2,5.5)
                        agv.retreat(15,4)
                        agv.stop()
                        agv.restore()

                    def adjust_offset(offset):
                        if offset > 20:
                            agv.pan_left(3,1)
                            print("Go to Left")
                            # offset = None
                        elif offset < -20:
                            agv.pan_right(3,1)
                            print("Go to Right")
                            # offset = None

                    cap = cv2.VideoCapture(0) 
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
                    frame_count = 0
                    center = 160

                    while( cap.isOpened() ): 
                        ret, frame = cap.read() 
                        if not ret:
                            print("failed to capture frame")
                            break
                        frame_count += 1
                        if frame_count % 5 == 0:
                            offset = None
                            frame = cv2.resize(frame, (320, 240))
                            cv2.imshow('cam',frame)
                            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            _, thresholded_frame = cv2.threshold(gray_frame, 90, 255, cv2.THRESH_BINARY)
                            
                            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                            thresholded_frame, aruco_dict, parameters=parameters)
                            
                            if ids is not None:
                                for i in range(len(ids)):
                                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                                        corners[i], marker_length, camera_matrix, dist_coeffs)
                                    distance = np.linalg.norm(tvec) 
                                    distance = round(distance, 2)
                            
                                    # aruco.drawDetectedMarkers(thresholded_frame, corners, ids)
                                    corner_points = corners[i][0]
                                    x_coords = corner_points[:,0]
                                    aruco_x = np.mean(x_coords)
                                    offset = center - aruco_x

                                    if (-20 < offset < 20):
                                        if 1.25 <= distance <= 1.45:
                                            print("Ready to Charge")
                                            cap.release()
                                            cv2.destroyAllWindows()
                                            go_charging()
                                            time.sleep(5)
                                            res = agv.get_battery_info()
                                            charge_check = res[0]
                                            if charge_check == '011001':
                                                audio_talker.publish(roslibpy.Message({'data':'6'}))
                                                print("Finished")
                                            return
                                        else:
                                            adjust_distance(distance)
                                            # print(f"dis: {distance}")
                                            frame_count = 0
                                    else:
                                        # print(offset)
                                        adjust_offset(offset)
                                        frame_count = 0
                
                        if cv2.waitKey(1) == ord('q'):
                            break

    # cap.release()
    # cv2.destroyAllWindows()
thread_mode = Thread_mode()
cam_thread = threading.Thread(target=thread_mode.cam_thread)
bridge_thread = threading.Thread(target=thread_mode.bridge_thread)
move_thread = threading.Thread(target=thread_mode.move_thread)

cam_thread.start()
bridge_thread.start()
move_thread.start()

cam_thread.join()
bridge_thread.join()
move_thread.join()