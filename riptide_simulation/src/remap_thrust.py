#!/usr/bin/env python
# remap_thrust.py
# This node converts real Riptide thruster inputs to UUV sim simulated thruster inputs
# as defined in remap_thrust_cfg.yml

import rospy
import rospkg
import yaml
from riptide_msgs.msg import PwmStamped, Pwm
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

rpack = rospkg.RosPack()
config_path = rpack.get_path('riptide_simulation') + "/cfg/remap_thrust_cfg.yml"
pubs = {}
cfg = {}

def callback(msg):
    for t in cfg['thruster_mapping']:
        f = FloatStamped()
        f.header = msg.header

        # e.g. f.data = msg.pwm.heave_stbd_aft.data
        code = "f.data = msg.pwm." + t[1]
        exec(code)

        pubs[t[0]].publish(f)

def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def main():
    loadConfig()
    for t in cfg['thruster_mapping']:
        sim_topic = cfg['sim_topic_prefix'] + t[0] + cfg['sim_topic_suffix']
        pub = rospy.Publisher(sim_topic, FloatStamped, queue_size=10)
        pubs[t[0]] = pub

    sub = rospy.Subscriber(cfg['real_topic'], PwmStamped, callback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("remap_thrust")
    main()