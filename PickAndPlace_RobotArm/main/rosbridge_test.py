# Float32MultiArray Z값만 받기
from __future__ import print_function
import roslibpy

client = roslibpy.Ros(host='192.168.64.3',port=9090)
client.run()

listener = roslibpy.Topic(client,'/talker','std_msgs/Float32MultiArray')
def received(message):
    data_array = message['data']
    data = data_array[2]
    print(f'Heard talking :   {data}')
listener.subscribe(received)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()