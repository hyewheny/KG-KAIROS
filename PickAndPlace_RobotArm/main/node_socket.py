# talker (sub node and pub msg)
from __future__ import print_function
import roslibpy

client = roslibpy.Ros(host='192.168.64.3',port=9090)
client.run()
listener = roslibpy.Topic(client,'/x_value','std_msgs/Float32')
step_flag = True
def step_move():
    print("this have two blocks")
def normal_move():
    print("this have one block")
def received(message):
    global step_flag
    x_value = message['data']
    # print(f'Heard talking :   {x_value}')
    if step_flag:
        if x_value <5:
            step_flag=False
            step_move()
            normal_move()
            step_flag=True
        elif x_value >5:
            step_flag=False
            normal_move()
            step_flag=True
        else:
            pass
listener.subscribe(received)


try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()