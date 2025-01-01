#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import cv2.aruco as aruco

camera_matrix = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/camera_matrix.npy")
dist_coeffs = np.load(r"/home/er/catkin_ws/src/mamot/src/aruco/dist_coeffs.npy")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.0195

cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

frame_count = 0
center = 320
dis_fg = 0

rospy.init_node('vel_node')
vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

def rotate_agv(offset):
    global dis_fg 
    print("offset : ", offset)

    msg = Twist()
    if offset < 27.75:
        msg.linear.x = 0.0
        msg.angular.z = -0.2
        vel_pub.publish(msg)
        rospy.sleep(0.2)
        msg.angular.z=0.0
        vel_pub.publish(msg)
    elif offset > 57:
        msg.linear.x = 0.0
        msg.angular.z = 0.2
        vel_pub.publish(msg)
        rospy.sleep(0.2)
        msg.angular.z= 0.0
        vel_pub.publish(msg)

    else:
        print("Rotation Done")
        msg.angular.z= 0.0
        vel_pub.publish(msg)
        dis_fg = 1


def fwd_back(distance):
    global dis_fg
    print('distance: ',distance)

    msg = Twist()
    if distance < 0.50:
        msg.linear.x = -0.1
        msg.angular.z = 0.0
        vel_pub.publish(msg)
        rospy.sleep(0.1)
        msg.angular.x=0.0
        msg.angular.z = 0.0
        vel_pub.publish(msg)

    elif distance > 0.54:
        msg.linear.x = 0.1
        vel_pub.publish(msg)
        rospy.sleep(0.1)
        msg.angular.x=0.0
        vel_pub.publish(msg)
    else:
        dis_fg = 2
        msg.angular.x=0.0
        vel_pub.publish(msg)
        print("Dist Done")

while True:
    ret, frame = cap.read() 
    if not ret:
        print("failed to capture frame")
        break
    frame_count += 1
    if frame_count % 11 == 0:

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
                corner_points = corners[i][0]
                x_coords = corner_points[:,0]
                aruco_x = np.mean(x_coords)
                offset = center - aruco_x
                # print('offset: ', offset)
                print('distance: ', distance)
                
                # if dis_fg == 0 :
                #     print('offset: ', offset)
                #     rotate_agv(offset)
                # elif dis_fg == 1 :
                #     print('distance: ', distance)
                #     fwd_back(distance)
                # else:
                #     pass
                    
        else:
            print('No Marker')
        cv2.imshow('cam', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
