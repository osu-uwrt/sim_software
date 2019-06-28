#!/usr/bin/env python
# quat_to_euler_imu.py
# This node converts UUV sim simulated quaternion imu values to euler values from the sim_topic
# then publishes the new values to the real_topic as defined by quat_to_euler_imu_cfg.yml

import rospy
import rospkg
import yaml
import tf
import math
from sensor_msgs.msg import FluidPressure as SimDepth
from riptide_msgs.msg import Depth as RealDepth


def callback(msg):
    real_topic = RealDepth()

    real_topic.header = msg.header
    real_topic.depth = msg.fluid_pressure / (9.81 * 1000)

    pub.publish(real_topic)


def main():
    global cfg
    global pub

    sim_topic = rospy.get_param('~sim_topic')
    real_topic = rospy.get_param('~real_topic')
    sub = rospy.Subscriber(sim_topic, SimDepth, callback)
    pub = rospy.Publisher(real_topic, RealDepth, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("depth_formatting")
    main()
