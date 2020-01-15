#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import math

class depthConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("depth/pressure", FluidPressure, self.depthCb)
        self.pub = rospy.Publisher("depth/pose", PoseWithCovarianceStamped, queue_size=10)

    def depthCb(self, msg):
        outMsg = PoseWithCovarianceStamped()
        outMsg.header = msg.header
        outMsg.header.frame_id = "odom"
        outMsg.pose.pose.position.z = (101.325 - msg.fluid_pressure)/9.80638
        outMsg.pose.covariance[0] = -1
        outMsg.pose.covariance[7] = -1
        outMsg.pose.covariance[14] = msg.variance / (9.80638 ** 2)
        outMsg.pose.covariance[21] = -1
        outMsg.pose.covariance[28] = -1
        outMsg.pose.covariance[35] = -1
        self.pub.publish(outMsg)

if __name__ == '__main__':
    rospy.init_node('depth_converter')
    depthConverter()
    rospy.spin()
    