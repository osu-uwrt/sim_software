#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import riptide_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
from nortek_dvl.msg import Dvl
from gazebo_msgs.msg import ModelStates
import math
import tf

class dvlConverter():
    def __init__(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/8), self.getVelocity)
        self.pub = rospy.Publisher("/dvl", Dvl, queue_size=1)

    def getVelocity(self, event):
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        index = msg.name.index("puddles")
        velocity = msg.twist[index].linear
        pass


if __name__ == '__main__':
    rospy.init_node('dvl_converter')
    dvlConverter()
    rospy.spin()
    