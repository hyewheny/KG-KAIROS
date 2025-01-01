# 기존 움직임 코드
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

def true_move():
    mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,10],40)
    time.sleep(2)
    mc.send_angles([10.09,(-39.9),(-11.25),(-30.67),89.29,10],40)
    time.sleep(3)
    mc.set_gripper_value(0,50)
    time.sleep(1)
    mc.send_angles([10.09,(-26.13),(-11.25),(-30.67),89.12,15.82],40)
    time.sleep(2)

    # place 
    mc.send_angles([55.81,(-26.13),(-13.62),(-29.53),89.64,-29.17],50)
    time.sleep(2)
    mc.send_angles([55.81,(-41.68),(-13.62),(-29.53),89.64,-29.17],50)
    time.sleep(2)
    mc.set_gripper_value(40,50)
    time.sleep(1)
    mc.send_angles([55.81,(-26.13),(-10),(-29.44),89.12,-36],50)
    time.sleep(2)
    # back
    mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,15.82],50)
    time.sleep(2)

def false_move():
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
    mc.send_angles([10.09,(-24.87),(-11.25),(-30.67),89.29,9.75],40)
    time.sleep(2)

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