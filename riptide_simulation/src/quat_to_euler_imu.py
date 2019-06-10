#!/usr/bin/env python
# quat_to_euler_imu.py
# This node converts UUV sim simulated quaternion imu values to euler values from the sim_topic
# then publishes the new values to the real_topic as defined by quat_to_euler_imu_cfg.yml

import rospy
import rospkg
import yaml
import tf
import math
from sensor_msgs.msg import Imu as SimImu
from imu_3dm_gx4.msg import FilterOutput as RealImu
from geometry_msgs.msg import Vector3

rpack = rospkg.RosPack()
config_path = rpack.get_path('riptide_simulation') + "/cfg/quat_to_euler_imu_cfg.yml"

def callback(msg):
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        eulerAnglesList = tf.transformations.euler_from_quaternion([w, x, y, z])

        eulerAngles_rad = Vector3()
        eulerAngles_rad.x = eulerAnglesList[0]
        eulerAngles_rad.y = eulerAnglesList[1]
        eulerAngles_rad.z = eulerAnglesList[2]
        real_topic = RealImu()

        real_topic.header = msg.header
        # Gyroscope Euler angles roll, pitch, yaw, and status
        real_topic.euler_rpy = eulerAngles_rad 
        # Heading Update Data
        real_topic.heading_update_alt = eulerAngles_rad.z #Heading in [radians]
        real_topic.heading_update_LORD = eulerAngles_rad.z #Heading in [radians]
        # Linear accelerations along x,y,z axes, and status
        real_topic.linear_acceleration = msg.linear_acceleration #X, Y, and Z axes in [m/s^2]
        # Angular rates along x,y,z axes, and status
        real_topic.angular_velocity = msg.angular_velocity #X, Y, and Z axes in [radians/s]
        
        pub.publish(real_topic)

def main():
        global cfg
        global pub
        with open(config_path, 'r') as stream:
                cfg = yaml.load(stream)
        sim_topic = cfg['sim_topic']
        real_topic = cfg['real_topic']
        sub = rospy.Subscriber(sim_topic, SimImu, callback)
        pub = rospy.Publisher(real_topic, RealImu, queue_size=10)
        rospy.spin()

if __name__ == "__main__":
        rospy.init_node("quat_to_euler_imu")
        main()