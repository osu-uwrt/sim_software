#!/usr/bin/env python

import rospy
import yaml
import math
import numpy
from riptide_msgs.msg import ThrustStamped, Thrust
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrustConverter(object):

    def __init__(self):
        self._thrust_command_topic = rospy.get_param('~thrust_command_topic')
        self._thruster_prefix = rospy.get_param('~thruster_prefix')
        self._thruster_suffix = rospy.get_param('~thruster_suffix')
        self._config_file = rospy.get_param("~config")
        self._propeller_config_file = rospy.get_param("~propeller_config")
        self._config = {}
        self._propeller_config = {}
        self._pubs = {}
        self._thrusters = []
        
        with open(self._config_file, 'r') as stream:
            self._config = yaml.safe_load(stream)
        with open(self._propeller_config_file, 'r') as stream:
            self._propeller_config = yaml.safe_load(stream)

        self.init_thruster_pubs()
        self._sub = rospy.Subscriber(self._thrust_command_topic, ThrustStamped, self.command_cb)

        rospy.loginfo("Thrust Converter initialized")

    def init_thruster_pubs(self):
        """Initialize thruster publishers"""
        self._thrusters = self._config.keys()
        for thruster in self._thrusters:
            input_topic = self._thruster_prefix + str(self._config[thruster]['id']) + self._thruster_suffix
            pub = rospy.Publisher(input_topic, FloatStamped, queue_size=1)
            self._pubs[thruster] = pub

    def command_cb(self, msg):
        """Reads in thruster commands from main vehicle, then converts to angular velocity commands for uuv_thruster_ros_plugin"""
        for thruster in self._thrusters:
            cmd = FloatStamped()
            force = 0
            code = "force = msg.force." + thruster
            exec(code) # Run the command, get thruster force

            ang_vel = self.get_angular_velocity(force)
            cmd.header = msg.header
            code = "cmd.data = " + str(ang_vel)
            exec(code) # Run the command, populate message
            self._pubs[thruster].publish(f)

    def get_angular_velocity(self, force):
        """Get propeller angular velocity"""
        # Currently, only a Basic conversion is used. Can add other conversions, like the Bessa curve
        if self._propeller_config['gazebo']['thrust_conversion']['type'] == "Basic":
            coeff = self._propeller_config['gazebo']['thrust_conversion']['constant']
            ang_vel = math.sqrt(abs(force / coeff))
            calc_force = coeff * ang_vel * abs(ang_vel)
            if numpy.sign(calc_force) == numpy.sign(force):
                return ang_vel
            else:
                return -ang_vel