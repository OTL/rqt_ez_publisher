#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon

def hoge(msg):
    print msg.points[0].x

rospy.init_node('polygon')
rospy.Subscriber('/polygon', Polygon, hoge)
rospy.spin()
