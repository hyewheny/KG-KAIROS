# ROS에서 depth cam 토픽 구독 후 z값만 발행하는 토픽을 rosbridge 를 통해 윈도우에서 구독
## 와이 파이 연결 주의
from __future__ import print_function
import roslibpy
# 브릿지와 연결
client = roslibpy.Ros(host='172.30.1.40', port=9090)
client.run()
# 토픽 생성
listener = roslibpy.Topic(client, '/z_value', 'std_msgs/Float32')
# 구독 콜백 함수 정의
def received(message):
    z_value = message['data']
    print(f'Received: {z_value}')
# 구독 콜백 함수 실행
listener.subscribe(received)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
