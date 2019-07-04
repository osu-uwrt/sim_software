#!/usr/bin/env python
# dvl_formatting.py
# This node converts simulation dvl values to nortek formatted dvl values and publishes it to the topic riptide expects

import rospy
import rospkg
import yaml
import tf
import math
from uuv_sensor_ros_plugins_msgs.msg import DVL as UUVDvl
from nortek_dvl.msg import Dvl as NortekDvl
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

class DVLFormatter(object):

    def __init__(self):
        self.main()

    def main(self):

def callback(msg):
    real_topic = RealDvl()

    real_topic.header = msg.header
    real_topic.time = 0  # ping time
    real_topic.dt1 = 0  # time from trigger to center of water track cell
    real_topic.dt2 = 0  # time from start of output message to center of water track cell
    real_topic.velocity = msg.velocity
    real_topic.figureOfMerit = 0
    real_topic.beamDistance = [
        msg.beams[0].range, msg.beams[1].range, msg.beams[2].range, msg.beams[3].range]
    real_topic.batteryVoltage = 0
    real_topic.speedSound = 0
    real_topic.pressure = 0
    real_topic.temp = 0

    pub.publish(real_topic)


def main():
    global cfg
    global pub

    sim_topic = rospy.get_param('~sim_topic')
    real_topic = rospy.get_param('~real_topic')
    sub = rospy.Subscriber(sim_topic, SimDvl, callback)
    pub = rospy.Publisher(real_topic, RealDvl, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("dvl_formatting")
    main()
