# Sub Node > Pub msg
import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import roslibpy
import time

class Testsub(Node):
	def __init__(self):
		super().__init__('testsub')
		self.subscription = self.create_subscription(
			Pose,
			'/turtle1/pose',
			self.callback,
			10)
		self.subscription # prevent unused variable warning
		self.client = roslibpy.Ros(host='192.168.64.3',port=9090)
		self.client.run()
		self.talker = roslibpy.Topic(self.client, '/x_value', 'std_msgs/Float32')
	def callback(self,msg):
		# 브릿지와 연결이 되어있으면 pub을 한다
		if self.client.is_connected:
			x_value=msg.x
			print(f"X value: {x_value}")
			self.talker.publish(roslibpy.Message({'data': x_value}))
			

def main(args=None):
	rp.init(args=args)

	testsub = Testsub()
	rp.spin(testsub)
	# 등록된 토픽을 제거
	testsub.talker.unadvertise()
	# 내부 이벤트 종료
	testsub.client.terminate()
	testsub.destroy_node()
	rp.shutdown()


if __name__ =='__main__':
	main()