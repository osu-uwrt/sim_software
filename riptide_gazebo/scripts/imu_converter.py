#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import riptide_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
import math
from tf.transformations import quaternion_conjugate, quaternion_multiply, quaternion_from_euler, euler_from_quaternion

class imuConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("/sensors/imu", sensor_msgs.msg.Imu, self.imuCb)
        self.pub = rospy.Publisher("/state/imu", riptide_msgs.msg.Imu, queue_size=10)

    def copyNED(self, src, dst, scale = 1):
        dst.x = src.x * scale
        dst.y = -src.y * scale
        dst.z = -src.z * scale
    
    def copy(self, src, dst, scale = 1):
        dst.x = src.x * scale
        dst.y = src.y * scale
        dst.z = src.z * scale
        try:
            dst.w = src.w * scale
        except:
            pass

    def quaternion2NED(self, q):
        NEDQ = Quaternion()
        [NEDQ.x, NEDQ.y, NEDQ.z, NEDQ.w] = quaternion_multiply(
            quaternion_conjugate(quaternion_from_euler(0, 0, 1.5708)),
            [q.x, q.y, q.z, q.w]
        )
        NEDQ.y = -NEDQ.y
        NEDQ.z = -NEDQ.z
        return NEDQ

    def xyzw_array(self, o):
        return [o.x, o.y, o.z, o.w]

    def subtract(self, a, b):
        a.x -= b[0]
        a.y -= b[1]
        a.z -= b[2]
        return a

    def imuCb(self, msg):
        outMsg = riptide_msgs.msg.Imu()
        self.copyNED(msg.angular_velocity, outMsg.ang_vel_rad)
        self.copyNED(msg.angular_velocity, outMsg.ang_vel_deg, 180 / math.pi)
        NEDQ = self.quaternion2NED(msg.orientation) 
        self.copy(NEDQ, outMsg.quaternion)
        
        [outMsg.rpy_rad.x, outMsg.rpy_rad.y, outMsg.rpy_rad.z] = euler_from_quaternion((NEDQ.x, NEDQ.y, NEDQ.z, NEDQ.w))
        self.copy(outMsg.rpy_rad, outMsg.rpy_deg, 180/math.pi)

        gravityVector = quaternion_multiply(
            quaternion_conjugate(self.xyzw_array(msg.orientation)),
            quaternion_multiply([0,0,9.8,0], self.xyzw_array(msg.orientation))
        )
        realAccel = self.subtract(msg.linear_acceleration, gravityVector)
        self.copyNED(realAccel, outMsg.linear_accel)
        
        self.pub.publish(outMsg)

if __name__ == '__main__':
    rospy.init_node('imu_converter')
    imuConverter()
    rospy.spin()
    