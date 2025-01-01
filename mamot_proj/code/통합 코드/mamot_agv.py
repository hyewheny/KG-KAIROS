#!/usr/bin/env python
import rospy
import roslibpy
import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import base64
import time
import threading
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import *
from pymycobot.myagv import MyAgv

ros_client = roslibpy.Ros(host='172.30.1.41', port=9090)
ros_client.run()

agv_talker = roslibpy.Topic(ros_client, '/plate_agv', 'std_msgs/String')
agv_listener = roslibpy.Topic(ros_client, '/plate_pc', 'std_msgs/String')

exit_fg = False 
position_fg = 0
status_fg = 0
move_fg = 0
rospy.init_node('move_base_action_client', anonymous=False)

def cam_thread():   
    global exit_fg
    global status_fg
    print(f'cam thread : {status_fg}')
    cap = cv2.VideoCapture(0)
    while not exit_fg:
        if status_fg == 1:
            ret, frame = cap.read()
            if not ret:
                print("ret error")
                break
            time.sleep(3)
            img_name = f"/home/er/catkin_ws/src/mamot/src/image/plate_img.png"
            cv2.imwrite(img_name, frame)
            status_fg=2
            print('cam thread : Captured img from agv cam, changed fg')
            break
    print("cam thread end")
    cap.release()
    cv2.destroyAllWindows()    

def bridge_thread():
    global exit_fg
    global position_fg
    global status_fg

    def agv_listener_callback(msg):
            global move_fg
            if msg['data'] == '1':
                # [Rin] Start SLAM Nav Code
                print('1. Mamot Start, Moving to photo spot')
                move_fg = 1

            elif msg['data'] == '2':
                # [Rin] Move next to tire SLAM Nav Code
                print("3. Moving next to tire position")
                # send_goal(1)
                # time.sleep(3)
                # send_goal(2)
                move_fg = 2

    while not exit_fg:
        # 1. Mamot Go
        agv_listener.subscribe(agv_listener_callback)
        while status_fg == 0:
            if position_fg == 1:
                status_fg = 1
        while status_fg == 1:
            pass
        # 2. Get Plate Img
        # [Rin] Camera On, Take Photo & Save
        while status_fg == 2:
            plate_img = cv2.imread('/home/er/catkin_ws/src/mamot/src/image/plate_img.png')
            _, buffer = cv2.imencode('.jpg', plate_img)
            plate_msg = 'aaa' + base64.b64encode(buffer).decode('utf-8')
            agv_talker.publish(roslibpy.Message({'data': plate_msg}))
            print('2. Published plate image')
            status_fg = 3

        # 3. Confirm Plate Num
        while status_fg == 3:
            if position_fg == 2:
                agv_listener.unsubscribe()
                status_fg = 4

        # 4. Send Depth Info
        # [Rin] Receive depth (photo?) data from Depth cam
        while status_fg == 4:
            depth = 'bbb' + '4' # sample
            agv_talker.publish(roslibpy.Message({'data': depth}))
            print('4. published depth data')
            status_fg = 5

        while status_fg == 5:
            exit_fg = True
            break
            

def move_thread():
    global exit_fg
    global position_fg
    global move_fg

    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_client.wait_for_server()

    def feedback_callback(msg):
        rospy.loginfo(f'[FEEDBACK MSG] :{msg}')

    def send_goal(pose_id): #x,y,z,w
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if pose_id == 1:
            # (2.00, -0.30)
            goal.target_pose.pose.position.x = 2.00 
            goal.target_pose.pose.position.y = -0.30 
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

        elif pose_id == 2:
            # (2.34, 0.20)
            goal.target_pose.pose.position.x = 2.64 
            goal.target_pose.pose.position.y = 0.42 
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.269
            goal.target_pose.pose.orientation.w = 0.962

        elif pose_id == 3:
            # (3.68, 0.39)
            goal.target_pose.pose.position.x = 3.25 
            goal.target_pose.pose.position.y = 0.30  
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

        move_client.send_goal(goal=goal, feedback_cb=feedback_callback)
        move_client.wait_for_result()
        rospy.loginfo(f"Mamot reached {pose_id}")
        
    while not exit_fg:
        if move_fg == 1:
            send_goal(1)
            position_fg = 1
            move_fg = None
        elif move_fg == 2:
            send_goal(2)
            time.sleep(1)
            send_goal(3)
            position_fg = 2
            move_fg = None
            break

def manual_mode():
    agv = MyAgv("/dev/ttyAMA2", 115200)
    camera_matrix = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/camera_matrix.npy")
    dist_coeffs = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/dist_coeffs.npy")
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    marker_length = 0.0195

    def keep_distance(distance):
        if distance < 1.3:
            agv.retreat(1,0.5)
        else:
            agv.go_ahead(1,0.5)

    def go_charging():
        agv.counterclockwise_rotation(2,6)
        agv.retreat(2,2)
        agv.stop()
        agv.restore()

    def aruco_move(offset):
        if offset > 0:
            agv.pan_left(1,1)
            print("Go to Left")
            offset = None
        elif offset < 0:
            # time.sleep(0.5)
            agv.pan_right(1,1)
            print("Go to Right")
            offset = None

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
            
            # Aruco 留덉빱 ?먯?
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
            thresholded_frame, aruco_dict, parameters=parameters)
            
            if ids is not None:
                # 媛?留덉빱 猷⑦봽
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], marker_length, camera_matrix, dist_coeffs)
                    distance = np.linalg.norm(tvec) 
                    distance = round(distance, 2)
                    # 留덉빱??寃쎄퀎? 異?                    aruco.drawDetectedMarkers(thresholded_frame, corners, ids)
                    corner_points = corners[i][0]
                    x_coords = corner_points[:,0]
                    aruco_x = np.mean(x_coords)
                    offset = center - aruco_x

                    if (-20 < offset < 20):
                        if distance == 1.3:
                            print("Ready to Charge")
                            go_charging()
                            time.sleep(3)
                            print("Finished")
                            return
                        else:
                            keep_distance(distance)
                            frame_count = 0
                    else:
                        # print(offset)
                        aruco_move(offset)
                        frame_count = 0
   
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

cam_thread = threading.Thread(target=cam_thread)
bridge_thread = threading.Thread(target=bridge_thread)
move_thread = threading.Thread(target=move_thread)

cam_thread.start()
bridge_thread.start()
move_thread.start()

cam_thread.join()
bridge_thread.join()
move_thread.join()

print("ARUCO MODE START")
manual_mode()
print("ARUCO MODE END")
