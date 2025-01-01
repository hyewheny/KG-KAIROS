# 's' 누르면 카메라 캡쳐 
import cv2
import os

dir = 'images'
os.makedirs(dir, exist_ok=True)
cap = cv2.VideoCapture(1)   
count = 0

while True:
    ret, frame = cap.read()
    key = cv2.waitKey(1) & 0xff

    if cv2.waitKey(1) == ord('q'):
        break

    elif key == ord('s'):
        print('Saved img')
        print(count)
        PATH = os.path.join(dir,f'image_{count}.jpg')
        cv2.imwrite(PATH,frame)
        count+=1
    cv2.imshow('vision cam',frame)
cap.release()
cv2.destroyAllwindows()