#!/usr/bin/env python

import rospy
import yaml
from uuv_sensor_ros_plugins_msgs.msg import DVL as UUVDvl
from nortek_dvl.msg import Dvl as NortekDvlMsg

class NortekDvl(object):

    def __init__(self):
        self._uuv_topic = rospy.get_param('~sim_topic')
        self._nortek_topic = rospy.get_param('~real_topic')
        self._sub = rospy.Subscriber(self._uuv_topic, UUVDvl, self.uuv_dvl_cb)
        self._pub = rospy.Publisher(self._nortek_topic, NortekDvlMsg, queue_size=1)
        rospy.loginfo("Nortek DVL node initialized")
        

    def uuv_dvl_cb(self, data):
        """Callback for UUV Simulator DVL message. 
        Publish same message as Nortek DVL, but in NED convention."""
        nortek_msg = NortekDvlMsg()
        nortek_msg.header = data.header
        nortek_msg.time = 0  # ping time
        nortek_msg.dt1 = 0  # time from trigger to center of water track cell
        nortek_msg.dt2 = 0  # time from start of output message to center of water track cell
        nortek_msg.velocity.x = data.velocity.x
        nortek_msg.velocity.y = -data.velocity.y
        nortek_msg.velocity.z = -data.velocity.z
        nortek_msg.figureOfMerit = 0
        nortek_msg.beamDistance = [data.beams[0].range, data.beams[1].range, data.beams[2].range, data.beams[3].range]
        nortek_msg.batteryVoltage = 0
        nortek_msg.speedSound = 0
        nortek_msg.pressure = 0
        nortek_msg.temp = 0

        self._pub.publish(nortek_msg)
