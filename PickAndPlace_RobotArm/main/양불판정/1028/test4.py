import numpy as np
import cv2
import time
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot

model = YOLO(r'C:\Users\snug1\Desktop\2차프로젝트\양불판정\best.pt')
cap = cv2.VideoCapture(0)
mc = MyCobot('COM5',115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
true_detect = False
# default pose : mc.send_angles([15,(-35),(-10),(-29.44),89.12,16],20)

def true_move():
    mc.send_angles([18,(-25),(-10),(-29.44),89.12,16],20)
    time.sleep(1)
    mc.send_angles([18,(-43.5),(-13),(-29.44),89.12,16],20)
    time.sleep(3)
    mc.set_gripper_value(0,50)
    time.sleep(2)
    mc.send_angles([18,(-25),(-10),(-29.44),89.12,16],50)
    time.sleep(1)
    # place 
    mc.send_angles([56,(-35),(-13),(-29.44),89.12,-36],40)
    time.sleep(1)
    mc.send_angles([56,(-45),(-10),(-26),89.12,-36],40)
    time.sleep(1)
    mc.set_gripper_value(40,50)
    time.sleep(1)
    mc.send_angles([56,(-35),(-10),(-29.44),89.12,-36],50)
    time.sleep(1)
    # back
    mc.send_angles([15,(-35),(-10),(-29.44),89.12,16],40)
    time.sleep(3)

def false_move():
    mc.send_angles([15,(-25),(-10),(-29.44),89.12,16],20)
    time.sleep(1)
    mc.send_angles([15,(-43.5),(-13),(-29.44),89.12,16],20)
    time.sleep(3)
    mc.set_gripper_value(0,50)
    time.sleep(2)
    mc.send_angles([15,(-25),(-10),(-29.44),89.12,16],20)
    time.sleep(2)
    # place 
    mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],30)
    time.sleep(2)
    mc.send_angles([-26,(-42),(-10),(-29.44),89.12,-39],20)
    time.sleep(2)
    mc.set_gripper_value(40,50)
    time.sleep(2)
    mc.send_angles([-26,(-35),(-10),(-29.44),89.12,-39],20)
    time.sleep(3)
    # back
    mc.send_angles([15,(-35),(-10),(-29.44),89.12,16],30)
    time.sleep(3)
while True:
    ret, frame = cap.read()
    height,width,_ = frame.shape
    roi = frame[:,width//4:3*(width//4)]
    cv2.rectangle(frame,(width//4,0),(3*(width//4),height),(0,0,255),2)

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
    cv2.imshow('Detection', annotated_frame)
cap.release()
cv2.destroyAllWindows()