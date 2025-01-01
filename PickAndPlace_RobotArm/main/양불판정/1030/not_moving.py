# 딥러닝 , 카메라만 있는 코드
import numpy as np
import cv2
import time
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot

model = YOLO(r'C:\Users\snug1\Desktop\2차프로젝트\양불판정\best.pt')
cap = cv2.VideoCapture(0)
mc = MyCobot('COM5',115200)
while True:
    ret, frame = cap.read()
    height,width,_ = frame.shape
    roi = frame[height//2:,width//4:3*(width//4)]
    cv2.rectangle(frame,(width//4,height//2),(3*(width//4),height),(0,0,255),2)

    results = model(roi)

    results = model(roi)
    annotated_frame = results[0].plot()
    name = results[0].names
    cls_inds = results[0].boxes.cls
    for cls in cls_inds:
        label = name[int(cls)]
        if label == "true":
            print("It's true")

        elif label == "false":
            print("It's false")
        else:
            print("Detect Waiting")
    if cv2.waitKey(1) == ord('q'):
        break
    cv2.imshow('Detection', frame)
cap.release()
cv2.destroyAllWindows()