# ROS <-> Window Python , Vision => threading
import threading
import cv2
import numpy as np
import time
import roslibpy
from __future__ import print_function
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot

model = YOLO(r'C:\Users\snug1\Desktop\2차프로젝트\양불판정\best.pt')
mc = MyCobot('COM5',115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
cap = cv2.VideoCapture(0)
true_detect = True
step_detect = True

class Movecobot():
    def __init__(self, mc):
        self.mc = mc

    def true_move(self):
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(1)
        mc.send_angles([10.09,(-43.5),(-11.25),(-30.67),89.29,15.82],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([10.09,(-26.13),(-11.25),(-30.67),89.12,15.82],50)
        time.sleep(1)

        # place 
        mc.send_angles([55.81,(-26.13),(-13.62),(-29.53),89.64,-29.17],40)
        time.sleep(1.5)
        mc.send_angles([55.81,(-41.68),(-13.62),(-29.53),89.64,-29.17],40)
        time.sleep(1.5)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([55.81,(-26.13),(-10),(-29.44),89.12,-36],50)
        time.sleep(1.5)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(3)

    def false_move(self):
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(1)
        mc.send_angles([10.09,(-43.5),(-11.25),(-30.67),89.29,15.82],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([10.09,(-26.13),(-11.25),(-30.67),89.12,15.82],50)
        time.sleep(1)
        # place 
        mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],50)
        time.sleep(2)
        mc.send_angles([-26,(-42),(-10),(-29.44),89.12,-39],50)
        time.sleep(2)
        mc.set_gripper_value(40,50)
        time.sleep(2)
        mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],50)
        time.sleep(3)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(3)

    def step_move(self):
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(1)
        mc.send_angles([10.09,(-43.5),(-11.25),(-30.67),89.29,15.82],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([10.09,(-26.13),(-11.25),(-30.67),89.12,15.82],50)
        time.sleep(1)

        # place 
        mc.send_angles([55.81,(-26.13),(-13.62),(-29.53),89.64,-29.17],40)
        time.sleep(1.5)
        mc.send_angles([55.81,(-41.68),(-13.62),(-29.53),89.64,-29.17],40)
        time.sleep(1.5)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([55.81,(-26.13),(-10),(-29.44),89.12,-36],50)
        time.sleep(1.5)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(3)


        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(1)
        mc.send_angles([10.09,(-43.5),(-11.25),(-30.67),89.29,15.82],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([10.09,(-26.13),(-11.25),(-30.67),89.12,15.82],50)
        time.sleep(1)
        # place 
        mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],50)
        time.sleep(2)
        mc.send_angles([-26,(-42),(-10),(-29.44),89.12,-39],50)
        time.sleep(2)
        mc.set_gripper_value(40,50)
        time.sleep(2)
        mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],50)
        time.sleep(3)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(3)

class Depthdata:
    def __init__(self):
        self.mc = mc
        self.step_detect = True

    def depth_thread(self):
        client = roslibpy.Ros(host='192.168.64.3',port=9090)
        client.run()
        listener = roslibpy.Topic(client,'/z_value','std_msgs/Float32')
        
        def received(message): # 콜백함수
            data_array = message['data']
            z_value = data_array[2]
            if self.step_detect:
                if z_value>=210:    
                    self.step_detect=False
                    print(f'Z value: {z_value}')    
                    self.mc.step_move()
                    self.step_detect=True
                
        listener.subscribe(received)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            client.terminate()
class VisionData(Movecobot):
    def __init__(self):
        self.mc = mc
    def vision_thread():
    while True:
        ret, frame = cap.read()
        height,width,_ = frame.shape
        roi = frame[:,width//4:3*(width//4)]
        cv2.rectangle(frame,(width//4,0),(3*(width//4),height),(0,0,255),2)
        cv2.imshow('Detection', frame)

        results = model(roi)
        annotated_frame = results[0].plot()
        name = results[0].names
        cls_inds = results[0].boxes.cls

        if true_detect == False:
            for cls in cls_inds:
                label = name[int(cls)]
                if label == "true":
                    true_detect = True
                    print("It's true")
                    true_move()
                    true_detect = False

                elif label == "false":
                    print("It's false")
                    true_detect = True
                    print("It's true")
                    false_move()
                    true_detect = False
                else:
                    print("Detect Waiting")
        
        if cv2.waitKey(1) == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()

if __name__ == "__main__":
    vision_thread = threading.Thread(target=vision_thread)
    depth_thread = threading.Thread(target=depth_thread)

    vision_thread.start()
    depth_thread.start()

    vision_thread.join()
    depth_thread.join()