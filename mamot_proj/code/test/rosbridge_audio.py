import rospy
import roslibpy
from std_msgs.msg import String
import time

ros_client = roslibpy.Ros(host='172.30.1.41', port=9090)
ros_client.run()

# rospy.init_node('audio_topic')
# rospy.spin()
pub = roslibpy.Topic(ros_client, 'audio_topic', 'std_msgs/String')
while True:
    if ros_client.is_connected:
        msg = roslibpy.Message({'data':'1'})
        print("I pubed")
        pub.publish(msg)
    
    time.sleep(10)

