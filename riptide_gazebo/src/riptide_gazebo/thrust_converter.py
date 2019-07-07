#!/usr/bin/env python

import rospy
import yaml
import math
import numpy
from riptide_msgs.msg import ThrustStamped, Thrust
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrustConverter(object):

    def __init__(self):
        self._input_topic = rospy.get_param('~input_topic')
        self._prefix = rospy.get_param('~topic_prefix')
        self._suffix = rospy.get_param('~topic_suffix')
        self._thruster_config_file = rospy.get_param("~thruster_config")
        self._common_config_file = rospy.get_param("~common_config")
        self._model = rospy.get_param("~thruster_model")
        self._config = {}
        self._model_config = {}
        self._pubs = {}
        self._thrusters = []
        
        with open(self._thruster_config_file, 'r') as stream:
            self._config = yaml.safe_load(stream)
        with open(self._common_config_file, 'r') as stream:
            common_config = yaml.safe_load(stream)
            self._model_config = common_config[self._model]

        self.init_thruster_pubs()
        self._sub = rospy.Subscriber(self._input_topic, ThrustStamped, self.command_cb)

        rospy.loginfo("Thrust Converter initialized")

    def init_thruster_pubs(self):
        """Initialize thruster publishers"""
        self._thrusters = self._config.keys()
        for thruster in self._thrusters:
            input_topic = self._prefix + str(self._config[thruster]['id']) + self._suffix
            pub = rospy.Publisher(input_topic, FloatStamped, queue_size=1)
            self._pubs[thruster] = pub

    def command_cb(self, msg):
        """Read in thruster commands from main vehicle, then convert to angular velocity commands for uuv_thruster_ros_plugin"""
        for thruster in self._thrusters:
            cmd = FloatStamped()
            force = 0
            code = "force = msg.force." + thruster
            exec(code) # Run the command, get thruster force

            ang_vel = self.get_angular_velocity(force)
            cmd.header = msg.header
            code = "cmd.data = " + str(ang_vel)
            exec(code) # Run the command, populate message
            self._pubs[thruster].publish(cmd)

    def get_angular_velocity(self, force):
        """Get propeller angular velocity"""
        # Currently, only a Basic conversion is used. Can add other conversions, like the Bessa curve
        if self._model_config['gazebo']['thrust_conversion']['type'] == "Basic":
            coeff = self._model_config['gazebo']['thrust_conversion']['constant']
            ang_vel = math.sqrt(abs(force / coeff))
            calc_force = coeff * ang_vel * abs(ang_vel)
            if numpy.sign(calc_force) == numpy.sign(force):
                return ang_vel
            else:
                return -ang_vel