#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import tf2_ros
from tf.transformations import quaternion_multiply, unit_vector, vector_norm, quaternion_conjugate, quaternion_matrix
from tf2_geometry_msgs import from_msg_msg
import math
import numpy as np
class dvlConverter():
    def __init__(self):
        self.dvlSub = rospy.Subscriber("dvl_twist", TwistWithCovarianceStamped, self.dvlCb)
        self.odomSub = rospy.Subscriber("odometry/filtered", Odometry, self.odomCb)
        self.pub = rospy.Publisher("dvl/twist", TwistWithCovarianceStamped, queue_size=10)
        self.namespace = rospy.get_param("~namespace", "puddles")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def odomCb(self, msg):
        self.odomTwist = msg.twist.twist

    def dvlCb(self, msg):
        twist = msg.twist.twist
        try:
            # Transform from dvl to base
            d2bTransform = self.tfBuffer.lookup_transform(self.namespace+'/base_link', self.namespace+'/dvl_link', rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.loginfo(ex)
            return

        d2bRotation = d2bTransform.rotation
        d2bMatrix = quaternion_matrix([d2bRotation.x, d2bRotation.y, d2bRotation.z, d2bRotation.w])[:3,:3]
        d2bOffset = d2bTransform.translation
        d2bVector = [d2bOffset.x, d2bOffset.y, d2bOffset.z]

        # Compute added velocity from angular velocity
        base_rot = [self.odomTwist.angular.x, self.odomTwist.angular.y, self.odomTwist.angular.z]
        out_vel = np.dot(d2bMatrix, [twist.linear.x, twist.linear.y, twist.linear.z])
        out_vel += np.cross(d2bVector, base_rot)

        # TODO: Rotate covariance
        msg.header.frame_id = self.namespace+"/base_link"
        twist.linear.x, twist.linear.y, twist.linear.z = out_vel
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dvl_converter')
    dvlConverter()
    rospy.spin()
    