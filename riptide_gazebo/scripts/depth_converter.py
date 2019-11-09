#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from riptide_msgs.msg import Depth
from geometry_msgs.msg import Vector3, Quaternion
import math
import tf

class depthConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("/sensors/pressure", FluidPressure, self.depthCb)
        self.pub = rospy.Publisher("/state/depth", Depth, queue_size=10)

    def depthCb(self, msg):
        outMsg = Depth()
        outMsg.depth = (msg.fluid_pressure - 101.325)/9.80638
        outMsg.pressure = msg.fluid_pressure
        self.pub.publish(outMsg)

if __name__ == '__main__':
    rospy.init_node('depth_converter')
    depthConverter()
    rospy.spin()
    