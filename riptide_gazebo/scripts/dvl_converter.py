#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import riptide_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
from nortek_dvl.msg import Dvl
from gazebo_msgs.msg import ModelStates
import math
from tf.transformations import quaternion_conjugate, quaternion_multiply

class dvlConverter():
    def __init__(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/8), self.getVelocity)
        self.pub = rospy.Publisher("/state/dvl", Dvl, queue_size=1)

    def getVelocity(self, event):
        # Get state of the vehicle
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        index = msg.name.index("puddles")

        # Find orientation and world frame velocity
        velocity = msg.twist[index].linear
        velocity = [velocity.x, velocity.y, velocity.z, 0]
        orientation = msg.pose[index].orientation
        quat = quaternion_conjugate((orientation.x, orientation.y, orientation.z, orientation.w))

        # Rotate to body frame
        velocity = quaternion_multiply(
            quaternion_multiply(quat, velocity), 
            quaternion_conjugate(quat)
        )[:3]

        # Publish as NED
        outMsg = Dvl()
        outMsg.header = rospy.Time.now
        outMsg.velocity.x = velocity[0]
        outMsg.velocity.y = -velocity[1]
        outMsg.velocity.z = -velocity[2]
        self.pub.publish(outMsg)
        pass


if __name__ == '__main__':
    rospy.init_node('dvl_converter')
    dvlConverter()
    rospy.spin()
    