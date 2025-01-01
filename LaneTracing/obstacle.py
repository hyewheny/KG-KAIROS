import rospy
from sensor_msgs.msg import LaserScan
import socket
import base64
import signal
import sys

HOST = '172.30.1.24'
PORT = 9999
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

distancelimit = 0.3 #Ben
sidedistancelimit = 0.3 #Ben

status = 0
status_send = False

def scan_callback(data):
   # rospy.loginfo('Lidar Sub > LaserScan -----------------')
   RF = LF = RB = LB = LS =0 #Ben
   RightFront = LeftFront = RightBack = LeftBack = LeftSide = 0 #Ben

   Ranges = data.ranges #Ben -->

   for i in range(len(Ranges)): 
      if 0<i<60: 
         if 0<Ranges[i]<distancelimit:
            LF +=1
      elif 60<i<120:
         if 0<Ranges[i]<1:
            LS +=1
      elif 430<i<490:
         if 0<Ranges[i]<distancelimit:
            RF +=1
      elif 120<i<180:
         if 0<Ranges[i]<sidedistancelimit:
            LB +=1
      elif 310<i<370:
         if 0<Ranges[i]<sidedistancelimit:
            RB +=1

   if RF>8: RightFront=1
   if LF>8: LeftFront=1
   if RB>8: RightBack=1
   if LB>8: LeftBack=1
   if LS>8: LeftSide=1
   if RF>8 and LF>8:
      if  RF>LF:
         RightFront=1
         LeftFront=0
      else:
         LeftFront=1
         RightFront=0
   # rospy.loginfo(f'LF:{LeftFront} RF:{RightFront} LB:{LeftBack} RB:{RightBack}')

   # sending on conditon 
   global status
   global status_send

   if RightFront or LeftFront:
      if status != 1:
         status = 1
         status_send = True
   if status == 1 and not RightFront and not LeftFront:
      if status != 2:
         status = 2
         status_send = True
   if status == 2 and (RightBack or LeftBack):
      if status != 3:
         status = 3
         status_send = True
   if status == 3 and not RightBack and not LeftBack:
      if status != 4:
         status = 4
         status_send = True
   if status == 4 and (LeftSide or LeftFront or LeftBack):
      if status != 5:
         status = 5
         status_send = True

   # Client part EJ      
   if status_send:
      msg = f'{LeftFront},{RightFront},{LeftBack},{RightBack},{LeftSide}'
      try:
         client_socket.sendall(msg.encode())
         rospy.loginfo(f'Status: {status}, Sent:({LeftFront, RightFront, LeftBack, RightBack})')
         status_send = False
      except Exception as e:
         rospy.logerr(f"Socket communication failed: {e}")

   pass

def listener():
   rospy.init_node('obstacle_test', anonymous=True)
   # rospy.loginfo('Start subscriber')
   rospy.Subscriber('scan', LaserScan, scan_callback)

   # scanServer = rospy.ServiceProxy('obstacle_pos', ObstacleService)
   # result = scanServer(True, True, False, False)

   rospy.spin()

def talker():
   rospy.init

if __name__ == '__main__':
   listener()