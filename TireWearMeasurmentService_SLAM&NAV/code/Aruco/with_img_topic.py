#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
import threading

class Thread_mode():
    def __init__(self):
        rospy.init_node('agv_node', anonymous=False)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.exit_fg = False 
        self.position_fg = 2
        self.status_fg = 2
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
    

    def cam_thread(self):
        
        def rotate_agv(pos, offset):
            # print("offset : ", offset)
            msg = Twist()
            if pos == 2:
                if offset < 60:
                    msg.linear.x = 0.0
                    msg.angular.z = -0.2
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.2)
                    msg.angular.z=0.0
                    self.vel_pub.publish(msg)
                elif offset > 80:
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
                if distance < 0.36:
                    msg.linear.x = -0.1
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    rospy.sleep(0.1)
                    msg.angular.x=0.0
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                elif distance > 0.38:
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

        def img_callback(data):
            try:
                if self.position_fg != 0 :
                    self.cv_img = self.cvbridge.imgmsg_to_cv2(data, 'bgr8')
                    if self.position_fg == 1:
                        img_name = f"/home/er/catkin_ws/src/mamot/src/image/plate_img.png"
                        cv2.imwrite(img_name, self.cv_img)
                        self.status_fg = self.status_fg + 1
                        self.position_fg = 0
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")   
                    
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
                        print('distance : ', distance)
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

        cam_sub = rospy.Subscriber('/camera_image', Image, img_callback)
        while not self.exit_fg:
            if self.position_fg != 0 and self.cv_img is not None:
                aruco_process(self.cv_img)
            


thread_mode = Thread_mode()
cam_thread = threading.Thread(target=thread_mode.cam_thread)
cam_thread.start()
cam_thread.join()