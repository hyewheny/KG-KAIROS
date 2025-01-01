import cv2
import numpy as np
import threading
import time
from pymycobot.myagv import MyAgv
import socket

agv = MyAgv("/dev/ttyAMA2", 115200)

HOST = '172.30.1.24'
PORT = 9999
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for a connection...")
conn, addr = server_socket.accept()
print(f"Connected by {addr}")

is_red_on = False
is_line_tracing_active = True  
line_result = None
obstacle_fg = "None"
obstacle_check = "None"

def obstacle_thread():
    global obstacle_fg, obstacle_check
    while obstacle_check != "finish":
        try:
            data = conn.recv(1024)
            if not data:
                print("Connection closed by client.")
            received_msg = data.decode()
            print(received_msg)
            
            # obstacle found
            if received_msg:
                obstacle_pos = list(map(int, received_msg.split(',')))
                # print(obstacle_pos)
                if obstacle_check == "None" and (obstacle_pos[0] == 1 or obstacle_pos[1] == 1):
                    obstacle_fg = "ob_left"
                    obstacle_check = "RIGHT"
                    print('1-1 RIGHT')
                if obstacle_fg == "ob_left":
                    # 1-2. [0, 0, 0, 0]
                    if obstacle_check == "RIGHT" and obstacle_pos[0] == 0 and obstacle_pos[1] == 0:
                        obstacle_check = "FORWARD"
                        print('1-2 FWD')
                    # 1-3. [0, 0, 1, 0]
                    if obstacle_pos[2] == 1:
                        obstacle_fg = "ob_left_3"
                        print('1-3 FWD')
                if obstacle_fg == "ob_left_3" or obstacle_fg == "ob_left_4":
                    # 1-4. [0, 0, 0, 0]
                    if obstacle_pos[2] ==0:
                        # obstaclecheck = "FORWARD"
                        obstacle_check = "PAN"
                        obstacle_fg = "ob_left_4"
                        print('1-4 PAN LEFT')
                if obstacle_fg == "ob_left_4" and (1 in obstacle_pos):
                    obstacle_fg = "ob_left_5"
                    obstacle_check = "FORWARD"
                    print('1-5 FWD')


        except Exception as e:
            print(f"An error occurred: {e}")
            conn.close()
            server_socket.close()

def process_traffic_light(traffic_roi):
    hsv = cv2.cvtColor(traffic_roi, cv2.COLOR_BGR2HSV)
    
    lower_red_on = np.array([160, 50, 50], dtype=np.uint8)
    upper_red_on = np.array([180, 255, 255], dtype=np.uint8)
    lower_green_on = np.array([40, 150, 150], dtype=np.uint8)
    upper_green_on = np.array([80, 255, 255], dtype=np.uint8)

    red_mask_on = cv2.inRange(hsv, lower_red_on, upper_red_on)
    green_mask_on = cv2.inRange(hsv, lower_green_on, upper_green_on)

    contours_red_on, _ = cv2.findContours(red_mask_on, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if contours_red_on:
    #     cv2.drawContours(traffic_roi, contours_red_on, -1, (0, 0, 255), 2)  

    contours_green_on, _ = cv2.findContours(green_mask_on, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if contours_green_on:
    #     cv2.drawContours(traffic_roi, contours_green_on, -1, (0, 255, 0), 2)  
 

    #print(f"Red mask count: {cv2.countNonZero(red_mask_on)}, Green mask count: {cv2.countNonZero(green_mask_on)}")

    if cv2.countNonZero(red_mask_on) > 500:
        return "RedON"
    elif cv2.countNonZero(green_mask_on) > 600:
        return "GreenON"
    return None


def process_line_tracing(line_roi):
    gray = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        #qqcv2.drawContours(line_roi, [c], -1, (0, 255, 0), 2) qq
        M = cv2.moments(c)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            center = line_roi.shape[1] // 2
            if cx < center - 25:
                return "LEFT"
            elif cx > center + 25:
                return "RIGHT"
            else:
                return "FORWARD"
    return None

# AGV
def control_agv():
    global is_red_on, line_result, is_line_tracing_active, obstacle_check, obstacle_fg
    while True:
        if is_red_on:
            print("AGV state: red, stop!")
            agv.stop()

        if obstacle_check != "None" and obstacle_check != "finish":
            if obstacle_fg == "ob_left_5" and line_result:
                obstacle_check = "finish"
                conn.close()
                server_socket.close()
                print(line_result)
                print('ENDING Obstacle Mode')
            else:
                line_result = obstacle_check

        if is_line_tracing_active and line_result:
            if obstacle_check == "None":
                print(f"AGV state: line mode! - {line_result}")
            else:
                # print(f'Obstacle Mode - {line_result}')
                pass

            if line_result == "LEFT":
                agv.counterclockwise_rotation(1)
                time.sleep(0.1)
            elif line_result == "RIGHT":
                agv.clockwise_rotation(1)
                time.sleep(0.1)
            elif line_result == "FORWARD":
                agv.go_ahead(1)
                time.sleep(0.1)
            elif line_result == "PAN":
                agv.counterclockwise_rotation(1)
                time.sleep(0.05)
                agv.pan_left(1)
                time.sleep(0.1)
        # else:
        #     agv.stop()
        #     time.sleep(0.1)

def camera_thread():
    global is_red_on, line_result, is_line_tracing_active
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        height, width, _ = frame.shape
        
        line_tracing_roi = frame[height // 2:, :]
        traffic_light_roi = frame[:height // 2, :]

        cv2.rectangle(frame, (0, height // 2), (width, height), (255, 0, 0), 2) 
        cv2.rectangle(frame, (0, 0), (width, height // 2), (0, 255, 0), 2)  

        # light
        traffic_light = process_traffic_light(traffic_light_roi)
        if traffic_light == "RedON":
            print("Red_Light!")
            is_red_on = True
            is_line_tracing_active = False

        elif traffic_light == "GreenON":
            print("Green_Light!")
            is_red_on = False
            #time.sleep(1)
            is_line_tracing_active = True


        # LineTracing
        if not is_red_on and is_line_tracing_active:
            line_result = process_line_tracing(line_tracing_roi)
            # if line_result:
            #     print(f"LineTracing: {line_result}")

        cv2.imshow('AGV Camera', frame)
        if cv2.waitKey(1) == ord('q'):
            threading.Timer(0.3, agv.stop).start()  
            is_line_tracing_active = False
            agv.restore()  
            conn.close()
            server_socket.close()
            break

    cap.release()
	  cv2.destroyAllWindows()

agv_thread = threading.Thread(target=control_agv)
camera_thread = threading.Thread(target=camera_thread)
obstacle_thread = threading.Thread(target=obstacle_thread)

agv_thread.start()
camera_thread.start()
obstacle_thread.start()

camera_thread.join()
agv_thread.join()
obstacle_thread.join()