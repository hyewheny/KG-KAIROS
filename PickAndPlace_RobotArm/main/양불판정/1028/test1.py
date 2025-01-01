# vision cam ON
import cv2

cap = cv2.VideoCapture(1)   
while True:
    ret, frame = cap.read()
    #frame = cv2.flip(frame, 0)
    if cv2.waitKey(1) == ord('q'):
        break
    cv2.imshow('vision cam',frame)
cap.release()
cv2.destroyAllwindows()