# Image Topic 버전 > 잘 안됐음
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.exit_fg = False 
        self.position_fg = 0
        self.status_fg = 0
        self.move_fg = 0
        self.cmd_fg = 0
        self.camera_matrix = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/camera_matrix.npy")
        self.dist_coeffs = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/dist_coeffs.npy")
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_length = 0.0195
        self.cvbridge = CvBridge()
        self.center = 320
        self.dis_fg = 0
        self.cv_img = None

    def kill_node(self, kill_nodes):
        for node in kill_nodes:
            try:
                subprocess.run(['rosnode','kill',node], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")
    

    def cam_thread(self):
        def rotate_agv(pos, offset):
            print("offset : ", offset)
            msg = Twist()
            if pos == 2:
                if offset < -30:
                    msg.linear.x = 0.0
                    msg.angular.z = -0.2
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.2)
                    msg.angular.z=0.0
                    self.vel_pub.publish(msg)
                elif offset > 20:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.2
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.2)
                    msg.angular.z= 0.0
                    self.vel_pub.publish(msg)
                else:
                    print("Rotation Done")
                    msg.angular.z= 0.0
                    self.vel_pub.publish(msg)
                    self.dis_fg = 1
            if pos == 3:
                if offset < 0.49:
                    msg.linear.x = 0.0
                    msg.angular.z = -0.2
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.2)
                    msg.angular.z=0.0
                    self.vel_pub.publish(msg)
                elif offset > 0.53:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.2
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.2)
                    msg.angular.z= 0.0
                    self.vel_pub.publish(msg)
                else:
                    print("Rotation Done")
                    msg.angular.z= 0.0
                    self.vel_pub.publish(msg)
                    self.dis_fg = 1

        def fwd_back(pos, distance):
            print('distance: ', distance)
            msg = Twist()
            if pos == 2:
                if distance < 0.30:
                    msg.linear.x = -0.1
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.1)
                    msg.angular.x=0.0
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                elif distance > 0.35:
                    msg.linear.x = 0.1
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.1)
                    msg.angular.x=0.0
                    self.vel_pub.publish(msg)
                else:
                    self.dis_fg = 2
                    msg.angular.x= 0.0
                    self.vel_pub.publish(msg)
                    print("Dist Done")

            if pos == 3:
                if distance < 1.25:
                    msg.linear.x = -0.1
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.1)
                    msg.angular.x=0.0
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                elif distance > 1.45:
                    msg.linear.x = 0.1
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.1)
                    msg.angular.x=0.0
                    self.vel_pub.publish(msg)
                else:
                    self.dis_fg = 2
                    msg.angular.x= 0.0
                    self.vel_pub.publish(msg)
                    print("Dist Done")

        def aruco_process(frame):
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresholded_frame = cv2.threshold(gray_frame, 90, 255, cv2.THRESH_BINARY)
            
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
            thresholded_frame, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                    distance = np.linalg.norm(tvec) 
                    distance = round(distance, 2)
                    corner_points = corners[i][0]
                    x_coords = corner_points[:,0]
                    aruco_x = np.mean(x_coords)
                    offset = self.center - aruco_x
                    if self.dis_fg == 0 :
                        print('offset: ', offset)
                        rotate_agv(self.position_fg, offset)
                    elif self.dis_fg == 1 :
                        print('distance :', distance)
                        fwd_back(self.position_fg, distance)
                    elif self.dis_fg == 2:
                        self.status_fg = self.status_fg + 1
                        print('Status_fg : ', self.status_fg)
                        self.dis_fg = 0
                        self.position_fg = 0
                    else:
                        pass
            else:
                print("Can't detect Aruco Marker")

        def img_callback(data):
            try:
                if self.position_fg != 0 :
                    self.cv_img = self.cvbridge.imgmsg_to_cv2(data, 'bgr8')
                    if self.position_fg == 1:
                        audio_talker.publish(roslibpy.Message({'data':'2'}))
                        img_name = f"/home/er/catkin_ws/src/mamot/src/image/plate_img.png"
                        cv2.imwrite(img_name, self.cv_img)
                        self.status_fg = self.status_fg + 1
                        self.position_fg = 0           
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")   

        cam_sub = rospy.Subscriber('/camera_image', Image, img_callback)
    
        while not self.exit_fg:
            if self.position_fg != 0: 
                if self.position_fg == 2 or self.position_fg == 3:
                    if self.cv_img is not None:
                        aruco_process(self.cv_img)
             
    def bridge_thread(self):
        def agv_listener_callback(msg):
                if msg['data'] == '1':
                    # Move to plate position
                    print('1. Mamot Start, Moving to photo spot')
                    self.move_fg = 1
                    audio_talker.publish(roslibpy.Message({'data':'1'}))

                elif msg['data'] == '2':
                    # Move to tire position
                    print("2. Moving next to tire position")
                    self.move_fg = 2
                    audio_talker.publish(roslibpy.Message({'data':'3'}))

                elif msg['data'] == '3':
                    # Move to charge position
                    print("3. Moving next to tire position")
                    audio_talker.publish(roslibpy.Message({'data':'4'}))
                    self.move_fg = 3

        while not self.exit_fg:
            agv_listener.subscribe(agv_listener_callback)
            
            # 0. initial state
            while self.status_fg == 0:
                pass

            # 1. pub plate img 
            while self.status_fg == 1:
                plate_img = cv2.imread('/home/er/catkin_ws/src/mamot/src/image/plate_img.png')
                _, buffer = cv2.imencode('.jpg', plate_img)
                plate_msg = 'aaa' + base64.b64encode(buffer).decode('utf-8')
                agv_talker.publish(roslibpy.Message({'data': plate_msg}))
                print(" published IMG ")
                self.status_fg = 2

            # 2. waiting plate confirm
            while self.status_fg == 2:
                pass

            # 3. pub 'cmd depth'
            while self.status_fg == 3:
                agv_talker.publish(roslibpy.Message({'data':'bbb'}))
                self.status_fg = 4

            # 4. waiting tire confirm
            while self.status_fg == 4:
                    pass
                    
            # 5. arrived at charging position / close navigation
            while self.status_fg == 5:
                audio_talker.publish(roslibpy.Message({'data':'5'}))
                self.kill_node(kill_nodes)
                print("Killed all nodes")
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
                # go to plate (1.14, -0.39)
                goal.target_pose.pose.position.x = 1.23 
                goal.target_pose.pose.position.y = -0.39 
                goal.target_pose.pose.position.z = 0.0

                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = -0.04
                goal.target_pose.pose.orientation.w = 0.99

            elif pose_id == 21:
                # go to tire (3.25, 0.38)
                goal.target_pose.pose.position.x = 2.00
                goal.target_pose.pose.position.y = 0.32 
                goal.target_pose.pose.position.z = 0.0

                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.11
                goal.target_pose.pose.orientation.w = 0.99
            elif pose_id == 2:
                # go to tire (3.25, 0.38)
                goal.target_pose.pose.position.x = 2.30
                goal.target_pose.pose.position.y = 0.20 
                goal.target_pose.pose.position.z = 0.0

                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0

            elif pose_id == 3:
                # go to Aruco place (1.27, -0.42)
                goal.target_pose.pose.position.x = 1.27 
                goal.target_pose.pose.position.y = -0.42  
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
                send_goal(1)
                self.position_fg = 1
                self.move_fg = 0
            elif self.move_fg == 2:
                # send_goal(21)
                send_goal(2)
                self.position_fg = 2
                self.move_fg = 0
            elif self.move_fg == 3:
                send_goal(3)
                self.position_fg = 3
                self.move_fg = 0

            
def manual_mode():
    agv = MyAgv("/dev/ttyAMA2", 115200)
    def go_charging():
        agv.counterclockwise_rotation(2,5.5)
        agv.retreat(15,4)
        agv.stop()
        agv.restore()

    go_charging()
    res = agv.get_battery_info()
    charge_check = res[0]
    if charge_check == '011001':
        audio_talker.publish(roslibpy.Message({'data':'6'}))
        print("Finished")


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


print("Move to Charge")
manual_mode()
print("All finished")
