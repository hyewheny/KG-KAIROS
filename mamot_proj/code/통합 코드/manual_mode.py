import subprocess
import time
from pymycobot.myagv import MyAgv

kill_nodes=['/move_base','/rviz','/ydlidar_lidar_publisher','/amcl','/base2imu_link',
            '/base2camera_link','/base_link_to_imu_link','/base_link_to_laser_link','joint_state_publisher',
            'map_server_for_test','/myagv_odometry_node','/robot_pose_ekf','robot_state_publisher','/rosapi']

def kill_node(kill_nodes):
        for node in kill_nodes:
            try:
                subprocess.run(['rosnode','kill',node], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")

def manual_mode():
    agv = MyAgv("/dev/ttyAMA2", 115200)
    def go_charging():
        agv.counterclockwise_rotation(15,6)
        agv.retreat(15,3)
        agv.stop()
        agv.restore()
    go_charging()
    time.sleep(2)
    res = agv.get_battery_info()
    charge_check = res[0]
    if charge_check == '011001':
        # audio_talker.publish(roslibpy.Message({'data':'6'}))
        print("Finished")

