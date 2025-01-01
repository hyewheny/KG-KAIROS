import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import numpy as np
import cv2
import time
import roslibpy

client = roslibpy.Ros(host='192.168.64.3',port=9090)
client.run()
talker = roslibpy.Topic(client, '/talker','std_msgs/Float32MultiArray')
while client.is_connected:
    data_array = [1.0,2.0,3.0]
    talker.publish(roslibpy.Message({'data': data_array}))
    print('Sending message...')
    time.sleep(1)
talker.unadvertise()
client.terminate()