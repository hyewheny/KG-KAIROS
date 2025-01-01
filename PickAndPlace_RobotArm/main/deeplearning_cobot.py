# 1106
from __future__ import print_function
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot
import cv2
import threading
import time
import roslibpy

initial_position = [13.09, 0, 0, 0, 87.18, 14.85] # [0,0,0,0,0,0] 부터 시작할 때

ready_position = [13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85]
one_pick_position = [13.09, (-48), 6.54, (-45.79), 86.92, 14.85]
two_pick_position =  [13.09, (-41.39), (-1.23), (-45.61), 87.09, 14.85]
pick_up_position = [13.09, (-10.56), 2.54, (-45.79), 86.92, 14.85]

one_true_stop = [40.68, (-26.12),(-24.52),(-29.7),87.18,62.57]
one_true_place = [64.68, (-34.12),(-24.52),(-29.7),87.18,(-32.87)]

move_false_place = [129.55,(-6.76),(-12.48),(-42.53),86.83,14.85]
two_false_one_check =[13.09, 5.97,(-31.81),(46.14),88.33, 14.85]

model = YOLO(r'C:\Users\snug1\Desktop\second_proj\best.pt',verbose=False)
mc = MyCobot('COM5',115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(1)
cap = cv2.VideoCapture(0)

class Movecobot():
    def __init__(self, mc, detect):
        self.mc = mc
        self.detect = detect
        self.cobot_moving = False
        self.step_result = "None"

    def one_true_move(self):
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(3)
        mc.send_angles([13.09, (-52.08), 10.17, (-46.93), 87.45, 14.85],40)
        time.sleep(2)
        mc.set_gripper_value(0,50)
        time.sleep(1)
        mc.send_angles([13.09, (-10.56), 2.54, (-45.79), 86.92, 14.85],50)
        time.sleep(1)

        # place
        mc.send_angles([56, 0, (-13.6), -30, 71, 60],50)
        time.sleep(1)
        mc.send_angles([60, 0, -60, (-30), 71, 60],50)
        time.sleep(1)
        mc.send_angles([60, (-40), (-17), (-30), 90, 60],60)
        time.sleep(1)
        mc.set_gripper_value(60,50)
        time.sleep(1)
        mc.send_angles([60, (-36), (-17), (-30), 90, 60],60)
        time.sleep(1)
        mc.send_angles([60, 22, -60, (-30), 71, 60],50)
        time.sleep(1)
        mc.send_angles([56, 0, (-13.6), -30, 71, 60],50)
        time.sleep(1)
        mc.send_angles([13.1, (-20.68), (-11.25), (-40.67), 89.29, 9.75],50)
        time.sleep(1)
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(3)

    def one_false_move(self):
        #pick
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(1)
        mc.send_angles([13.09, (-53.08), 8.17, (-46.93), 87.45, 14.58],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(1)
        mc.send_angles([13.09, (-10.56), 2.54, (-45.79), 86.92, 14.85],50)
        time.sleep(3)

        # place
        mc.send_angles([129.55,(0.00),(-11.25),(-70.67),89.12,9.75],30)
        time.sleep(3)
        mc.send_angles([129.55,(-24.87),(-11.25),(-50.67),89.12,9.75],30)
        time.sleep(3)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([129.55,(0.00),(-11.25),(-70.67),89.12,9.75],30)
        time.sleep(3)
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(4)

    def two_true_move(self):
        # pick
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(1)
        mc.send_angles([13.09, (-41.39), (-1.23), (-45.61), 87.09, 14.85],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([13.09, (-10.56), 2.54, (-45.79), 86.92, 14.85],50)
        time.sleep(3)

        # place
        mc.send_angles([56, 0, (-13.6), -30, 71, 60],50)
        time.sleep(1)
        mc.send_angles([60, 0, -60, (-30), 71, 60],50)
        time.sleep(1)
        mc.send_angles([60, (-40), (-17), (-30), 90, 60],60)
        time.sleep(1)
        mc.set_gripper_value(60,50)
        time.sleep(1)
        mc.send_angles([60, (-36), (-17), (-30), 90, 60],60)
        time.sleep(1)
        mc.send_angles([60, 22, -60, (-30), 71, 60],50)
        time.sleep(1)
        mc.send_angles([56, 0, (-13.6), -30, 71, 60],50)
        time.sleep(1)
        mc.send_angles([13.1, (-20.68), (-11.25), (-40.67), 89.29, 9.75],50)
        time.sleep(1)
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(3)

    def two_false_move(self):
        mc.send_angles([13.09, (-20.12), (-14.58), (-45.52), 87.18, 14.85],40)
        time.sleep(2)
        mc.send_angles([13.09, (-41.39), (-1.23), (-45.61), 87.09, 14.85],40)
        time.sleep(3)
        mc.set_gripper_value(0,50)
        time.sleep(2)
        mc.send_angles([13.09, (-10.56), 2.54, (-45.79), 86.92, 14.85],50)
        time.sleep(3)

        mc.send_angles([129.55,(0.00),(-11.25),(-70.67),89.12,9.75],30)
        time.sleep(3)
        mc.send_angles([129.55,(-24.87),(-11.25),(-50.67),89.12,9.75],30)
        time.sleep(3)
        mc.set_gripper_value(40,50)
        time.sleep(1)
        mc.send_angles([129.55,(0.00),(-11.25),(-70.67),89.12,9.75],30)
        time.sleep(2)
        mc.send_angles([13.09,(0.00),(-11.25),(-70.67),89.12,9.75],50)
        time.sleep(4)

    def cobot_thread(self):
        while True:
            if not self.cobot_moving:
                print(f'블록 상태 : {self.detect.old_z} ------- 양불판정 : {self.detect.vision_result}')
                # Case : 1층 블록 
                if self.detect.old_z == "one_floor" and self.detect.vision_result == "True":
                    self.cobot_moving = True
                    print(f'-------------------------------------------------1 층 + 양 품')
                    self.one_true_move()
                    self.detect.old_z = "None"
                    self.cobot_moving = False
                

                elif self.detect.old_z == "one_floor" and self.detect.vision_result == "False":
                    self.cobot_moving = True
                    print(f'-------------------------------------------------1 층 + 불량품')
                    self.one_false_move()
                    self.detect.old_z = "None"
                    self.cobot_moving = False

                # Case : 2층 블록
                elif self.detect.old_z == "two_floor" and self.detect.vision_result == "True":
                    self.cobot_moving = True
                    print(f'-------------------------------------------------2 층 + 양 품')
                    self.two_true_move()
                    self.step_result = "to_one"

                    if self.step_result =="to_one" and self.detect.vision_result == "True":
                        print("--------------------------------------------- 2 > 1 층 양품")
                        self.one_true_move()
                        self.detect.old_z = "None"
                        self.step_result = "None"
                        
                        self.cobot_moving = False

                    elif self.step_result == "to_one" and self.detect.vision_result == "False":
                        print("---------------------------------------------2 > 1 층 불량품")
                        self.one_false_move()
                        self.detect.old_z = "None"
                        self.step_result = "None"
                        self.cobot_moving = False



                elif self.detect.old_z == "two_floor" and self.detect.vision_result == "False":
                    self.cobot_moving = True
                    print(f'------------------------------------------------2 층 + 불량품')
                    self.two_false_move()
                    self.step_result = "to_one"

                    if self.step_result =="to_one" and self.detect.vision_result == "True":
                        print("---------------------------------------------2 > 1 층 양품")
                        self.one_true_move()
                        self.detect.old_z = "None"
                        self.step_result = "None"
                        self.cobot_moving = False


                        
                    elif self.step_result == "to_one" and self.detect.vision_result == "False":
                        print("---------------------------------------------2 > 1 층 불량품")
                        self.one_false_move()
                        self.detect.old_z = "None"
                        self.step_result = "None"
                        self.cobot_moving = False
                else:
                    print(f'old_z: {self.detect.old_z} ------ result: {self.detect.vision_result}')
                    pass

class Detect():
    def __init__(self):
        self.vision_result = "None"
        self.old_z = "None"
        self.new_z = "None"
        client = roslibpy.Ros(host='172.30.1.40',port=9090)
        client.run()
        self.listener = roslibpy.Topic(client,'/coordinates','std_msgs/Float32')

    def detect_thread(self):
        def received(message):  
            z_value = message['data']
            z_value =int(z_value)
            if 266 < z_value <= 290:
                self.new_z = "one_floor"
                if(self.old_z!=self.new_z):
                    self.old_z = self.new_z
                    print(f"old_z: {self.old_z}")

            elif  235 <= z_value <= 266:
                self.new_z = "two_floor"
                if(self.old_z!=self.new_z):
                    self.old_z = self.new_z
                    print(f"old_z: {self.old_z}")
            else:
                self.old_z = "None"
                

        while True:
            self.listener.subscribe(received)
            ret, frame = cap.read()
            if not ret:
                break
            height,width,_ = frame.shape
            roi = frame[:,width//4:3*(width//4)]
            cv2.rectangle(frame,(width//4,0),(3*(width//4),height),(0,0,255),2)

            results = model(roi, conf=0.6)
            annotated_frame = results[0].plot()
            boxes = results[0].boxes
            name = results[0].names
            cls_inds = boxes.cls
            #cv2.imshow('Detection', frame)

            if len(cls_inds):
                for cls in cls_inds:
                    label = name[int(cls)]
                    if label == "true":
                        self.vision_result = "True"
                        

                    elif label == "false":
                        self.vision_result = "False"

            else:
                self.vision_result = "None"
                print("Not detected ")



        #     if cv2.waitKey(1) == ord('q'):
        #         break
        # cap.release()
        # cv2.destroyAllWindows()

detect = Detect()
operate_mc = Movecobot(mc, detect)

detect_thread = threading.Thread(target=detect.detect_thread)
cobot_thread = threading.Thread(target=operate_mc.cobot_thread)

detect_thread.start()
cobot_thread.start()

detect_thread.join()
cobot_thread.join()