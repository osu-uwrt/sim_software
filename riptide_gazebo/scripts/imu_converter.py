#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import riptide_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
import math
from tf.transformations import quaternion_conjugate, quaternion_multiply, quaternion_from_euler

class imuConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("/sensors/imu", sensor_msgs.msg.Imu, self.imuCb)
        self.pub = rospy.Publisher("/imu/data", sensor_msgs.msg.Imu, queue_size=10)

    def convert2NED(self, src):
        src.x = src.x
        src.y = -src.y
        src.z = -src.z

    def quaternion2NED(self, q):
        [q.x, q.y, q.z, q.w] = quaternion_multiply(
            quaternion_conjugate(quaternion_from_euler(0, 0, 1.5708)),
            [q.x, q.y, q.z, q.w]
        )
        q.y = -q.y
        q.z = -q.z

    def imuCb(self, msg):
        self.convert2NED(msg.angular_velocity)
        self.convert2NED(msg.linear_acceleration)
        self.quaternion2NED(msg.orientation)
    
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_converter')
    imuConverter()
    rospy.spin()
    