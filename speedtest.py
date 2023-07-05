#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import Float64

def degTorad(deg):
	rad_diff = 0.5304
	rad = deg * (3.14/180)
	return rad + rad_diff

def controller():
	speed = 10000.0
	position_val = 0
	speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
	position_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
	rospy.init_node('key_controller',anonymous=True)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		k = ord(getch.getch())
		if (k == 113): # q
			rospy.loginfo("Exit..")
			exit()
		elif (k == 119): # Up
			rospy.loginfo("speed up")
			position_val = degTorad(0)
			speed += 100
		elif (k == 115): # Down
			rospy.loginfo("speed down")
			position_val = degTorad(0)
			speed -=100
		#elif (k == 67): # Right
		#	position_val = degTorad(15)
		#	speed = 10000
		#elif (k == 68): # Left
		#	position_val = degTorad(-15)
		#	speed = 10000
		rospy.loginfo("current speed : " + str(speed))
		speed_pub.publish(speed)
		position_pub.publish(position_val)
if __name__=='__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass