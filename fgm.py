import rospy
import math
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class FGM:
	def __init__(self):
		self.scan_sub = rospy.Subscriber('/lidar2D', LaserScan, self.scan_callback)
		self.cmd_vel_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
		self.cmd_steering_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

		self.vel = Float64()
		self.steering = Float64()

	def scan_callback(self, data):
		left_area = data.ranges[299:359]
		right_area = data.ranges[0:70]
		center_area = data.ranges[0:30] + data.ranges[329:359]
		# print("left : %d right : %d center : %d" % (len(left_area), len(right_area), len(center_area)))

		threshold = 2.5

		left_gap = self.find_max_gap(left_area, threshold)
		right_gap = self.find_max_gap(right_area, threshold)
		center_gap = self.find_max_gap(center_area, threshold)

		if left_gap > right_gap and left_gap > center_gap:
			self.val = 5000
			self.steering = 0.5304 + 0.25
		elif right_gap > center_gap:
			self.val = 5000
			self.steering = 0.5304 - 0.25
		else:
			self.vel = 7000
			self.steering = 0.5304
		self.cmd_vel_pub.publish(self.vel)
		self.cmd_steering_pub.publish(self.steering)
		

		def find_max_gap(self, distances, threshold):
			max_gap = 0
			current_gap = 0
			for distance in distances:
				if distance > threshold:
					current_gap += 1
				else:
					if current_gap > max_gap:
							max_gap = current_gap
					current_gap = 0
			if current_gap > max_gap:
				max_gap = current_gap
			return max_gap


if __name__ == '__main__':
    try:
        rospy.init_node("FGM")
        fgm = FGM()
        rospy.spin()
    except:
        print("Error occured.")