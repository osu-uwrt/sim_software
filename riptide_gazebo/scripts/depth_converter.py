#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import tf2_ros
from tf.transformations import quaternion_multiply, unit_vector, vector_norm, quaternion_conjugate, quaternion_matrix
from tf2_geometry_msgs import from_msg_msg
import math
import numpy as np
class depthConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("depth/pressure", FluidPressure, self.depthCb)
        self.pub = rospy.Publisher("depth/pose", PoseWithCovarianceStamped, queue_size=10)
        self.namespace = rospy.get_namespace()
        self.surfacePressure = 101.325
        self.kPaPerM = 9.80638
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def depthCb(self, msg):
        try:
            # Rotation from base frame to odom
            b2oOrientation = self.tfBuffer.lookup_transform('odom', self.namespace+'/base_link', rospy.Time()).transform.rotation
            b2oMatrix = quaternion_matrix([b2oOrientation.x, b2oOrientation.y, b2oOrientation.z, b2oOrientation.w])[:3,:3]

            # Offset to pressure sensor
            pressureOffset = self.tfBuffer.lookup_transform(self.namespace+'/pressure_link', self.namespace+'/base_link', rospy.Time()).transform.translation
            b2pVector = [pressureOffset.x, pressureOffset.y, pressureOffset.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.loginfo(ex)
            return

        # Rotate pressure sensor offset into odom frame and get additional depth from offset
        # TODO: Calculate uncertainty of this measure
        addedDepth = np.dot(b2oMatrix, b2pVector)[2]

        # Publish z offset from odom to base_link (depth)
        outMsg = PoseWithCovarianceStamped()
        outMsg.header = msg.header
        outMsg.header.frame_id = "odom"
        outMsg.pose.pose.position.z = (self.surfacePressure - msg.fluid_pressure)/self.kPaPerM + addedDepth
        outMsg.pose.covariance[0] = -1
        outMsg.pose.covariance[7] = -1
        outMsg.pose.covariance[14] = msg.variance / (self.kPaPerM ** 2)
        outMsg.pose.covariance[21] = -1
        outMsg.pose.covariance[28] = -1
        outMsg.pose.covariance[35] = -1
        self.pub.publish(outMsg)

if __name__ == '__main__':
    rospy.init_node('depth_converter')
    depthConverter()
    rospy.spin()
    