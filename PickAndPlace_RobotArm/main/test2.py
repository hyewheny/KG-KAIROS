# ROS <-> Window Python :: rosbridge && Depth, Vision threading
# future 은 맨 위에 있어야 된다고 함.
from __future__ import print_function
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot
import threading
import cv2
import time
import roslibpy

model = YOLO(r'C:\Users\snug1\Desktop\2차프로젝트\양불판정\best.pt')
mc = MyCobot('COM5',115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
cap = cv2.VideoCapture(0)

class Movecobot():
    def __init__(self, mc, vision_data, depth_data):
        self.mc = mc
        self.vision_data = vision_data
        self.depth_data = depth_data

    def botton_true_move(self):
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
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],60)
        time.sleep(2)

    def bottom_false_move(self):
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,9.75],40)
        time.sleep(2)
        mc.send_angles([10.09,(-45.12),(-11.25),(-30.67),89.29,9.75],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([10.09,(-0.87),(-11.25),(-30.67),89.12,9.75],40)
        time.sleep(2)
        mc.send_angles([129.55,(-17.87),(-37.25),(-30.67),89.29,9,75],40)
        time.sleep(3)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([129.55,(-0.87),(-11.25),(-30.67),89.12,9.75],40)
        time.sleep(2)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(2)

    def top_true_move(self):
        # top pick

        # top place
        mc.send_angles([55.81,(-26.13),(-13.62),(-29.53),95,-29.17],40)
        time.sleep(2)
        mc.send_angles([55.81,(-41.68),(-13.62),(-29.53),95,-29.17],40)
        time.sleep(2)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([55.81,(-26.13),(-13.62),(-29.53),95,-29.17],40)
        time.sleep(2)
    def top_false_move(self):
        # top pick

        # top place
        mc.send_angles([129.55,(-17.87),(-37.25),(-30.67),89.29,9,75],40)
        time.sleep(3)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([129.55,(-0.87),(-11.25),(-30.67),89.12,9.75],40)
        time.sleep(2)
        # back
        mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
        time.sleep(2)

    def cobot_thread(self):
        while True:
            if self.depth_data == "one" and self.vision_data == "True":
                #self.bottom_true_move()
                print("true moving")
            elif self.depth_data == "one" and self.vision_data == "False":
                #self.bottom_false_move()
                print("false moving")
            elif self.depth_data == "two" and self.vision_data == "True":
                print("step moving")
                #self.step_move()
                self.step_result = "to_one"
                if self.step_result =="to_one" and self.vision_data == "True":
                    print("to_one_true moving")
                    #self.botton_true_move()
                elif self.step_result == "to_one" and self.vision_data == "False":
                    print("to_one_false moving")
                    #self.bottom_false_move()

        while True:
            if self.vision_data == "True":
                print("True")
            elif self.vision_data == "False":
                print("False")


            
class DepthData:
    def __init__(self):
        self.mc = mc
        self.step_detect = False
        self.step_result = "None"

    def depth_thread(self):
        client = roslibpy.Ros(host='172.30.1.40',port=9090)
        client.run()
        listener = roslibpy.Topic(client,'/z_value','std_msgs/Float32')
        
        def received(message): 
            z_value = message['data']
            if self.step_detect == False:
                self.step_result = "None"
                print(f'Z value: {z_value}')
                if z_value == 212:    
                    self.step_detect = True
                    self.step_result = "one"
                    print('one block')
                    self.step_detect = False
                elif (z_value > 0) and (z_value == 191):
                    self.step_detect = True
                    print('two block')
                    self.step_result = "two"
                    self.step_detect = False                
        listener.subscribe(received)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            client.terminate()

class VisionData():
    def __init__(self):
        self.mc = mc
        self.vison_detect = False
        self.vision_result = "None"
    def vision_thread(self):
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

            if self.vison_detect == False:
                for cls in cls_inds:
                    label = name[int(cls)]
                    if label == "true":
                        self.vison_detect = True
                        self.vision_result = "True"
                        print("It's true")

                    elif label == "false":
                        self.vison_detect = True
                        self.vison_result = "False"
                        print("It's false")
                    else:
                        print("Detect Waiting")
            
            if cv2.waitKey(1) == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

depth_data = DepthData()
vision_data = VisionData()
operate_mc = Movecobot(mc, depth_data, vision_data)

depth_thread = threading.Thread(target=depth_data.depth_thread)
vision_thread = threading.Thread(target=vision_data.vision_thread)
cobot_thread = threading.Thread(target=operate_mc.cobot_thread)

depth_thread.start()
vision_thread.start()
cobot_thread.start()

depth_thread.join()
vision_thread.join()
cobot_thread.join()

