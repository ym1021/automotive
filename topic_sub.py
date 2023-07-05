#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32
def callback(msg):
	print("%.1f"%msg.data)

rospy.init_node('float_subscriber')
sub = rospy.Subscriber('float_count', Float32, callback)
rospy.spin()