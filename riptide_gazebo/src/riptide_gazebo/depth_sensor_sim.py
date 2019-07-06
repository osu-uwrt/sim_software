#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import FluidPressure
from riptide_msgs.msg import Depth

class DepthSensor(object):

    def __init__(self):
        self._uuv_topic = rospy.get_param('~sim_topic')
        self._depth_topic = rospy.get_param('~real_topic')
        self._fluid_density = rospy.get_param('~fluid_density', 1000)
        self._sub = rospy.Subscriber(self._uuv_topic, FluidPressure, self.uuv_pressure_cb)
        self._pub = rospy.Publisher(self._depth_topic, Depth, queue_size=1)
        rospy.loginfo("Depth Sensor node initialized")

    def uuv_pressure_cb(self, data):
        """Callback for UUV Simulator pressure message. Publish same message as actual Depth Sensor"""
        depth_msg = Depth()
        depth_msg.header = data.header
        depth_msg.depth = data.fluid_pressure / (9.81 * self._fluid_density)
        self._pub.publish(depth_msg)
