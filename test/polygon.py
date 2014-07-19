#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Pose

def hoge(msg):
    print msg

rospy.init_node('polygon')
rospy.Subscriber('/polygon', Polygon, hoge)
rospy.Subscriber('/pose', Pose, hoge)
rospy.spin()
