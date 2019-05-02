#!/usr/bin/env python
# quat_to_euler_imu.py
# This node converts UUV sim simulated quaternion imu values to equivalent euler values
# as defined in remap_thrust_cfg.yml

import rospy
import rospkg
import yaml
from tf import transformations
from sensor_msgs.msg import Imu as SimImu
from riptide_msgs.msg import Imu as RealImu

rpack = rospkg.RosPack()
config_path = rpack.get_path('riptide_simulation') + "/cfg/quat_to_euler_imu_cfg.yml"

def callback(msg):
       angles = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

def main():
        with open(config_path, 'r') as stream:
                cfg = yaml.load(stream)
        pub = rospy.Publisher(cfg['real_topic'], RealImu, queue_size=10)
        sub = rospy.Subscriber(cfg['sim_topic'], SimImu, callback)
        rospy.spin()

if __name__ == "__main__":
        rospy.init_node("quat_to_euler_imu")
        main()