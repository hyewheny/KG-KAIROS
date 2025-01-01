import time
import roslibpy

client = roslibpy.Ros(host='172.30.1.100',port=9090)
client.run()

listener = roslibpy.Topic(client, '/position_set','std_msgs/String')
talker = roslibpy.Topic(client, '/position_cmd','std_msgs/String')
status_fg = 0

def listener_callback(msg):
    print("Received message:", msg)
    signal = msg['data']
    if signal == '1':
        print("Received pose 1")
        time.sleep(2)
        print(" Go AGV ! pose 2")
        talker.publish(roslibpy.Message({'data':'2'}))

    elif signal == '2':
        print("Received pose 2")
        time.sleep(2)
        print(" Go AGV ! pose 3")
        talker.publish(roslibpy.Message({'data':'3'}))
        print("cmd end")

try:
    while True:
        listener.subscribe(listener_callback)
        if status_fg == 0:
            time.sleep(3)
            print(" Go AGV ! pose 1")
            talker.publish(roslibpy.Message({'data':'3'}))
            status_fg = 1
        elif status_fg == 1:
            pass

except KeyboardInterrupt:
    client.terminate()