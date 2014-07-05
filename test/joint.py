#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def hoge(msg):
    print msg.position

rospy.init_node('joint')
rospy.Subscriber('/joint_states', JointState, hoge)
rospy.spin()
