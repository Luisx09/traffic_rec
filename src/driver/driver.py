#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

# Constants to stand in for MPH limits in signs
SPD_25MPH = 55
SPD_45MPH = 75
NO_SPD_LIM = 85
   
class Driver:
	def __init__(self):
		self.sign_sub = rospy.Subscriber("/sign_data", Header, self.drive_response)
		self.drv_pub = rospy.Publisher('/drv_vel', Float64, queue_size=10)
		self.prev_delta = None
		self.stopping = False
		self.speed_limit = NO_SPD_LIM
		
	def drive_response(self, data):
		traffic_sign = data.frame_id
		dist = data.seq # Distance in mm
		delta_stop = None
		active = rospy.get_param('~active', True)
		if active is False:
			self.drv_pub.publish(0.0)
			return
		Kp = rospy.get_param('~Kd', 1.0)
		Kd = rospy.get_param('~Kd', 1.0)

		if traffic_sign is 'Stopsign':
			# Stop
			print ("It's time to stop!")
			self.stopping = True
			delta_stop = dist - 50
		elif traffic_sign is 'speedLimit25':
			# Go slow
			# Set max speed to a low number
			self.speed_limit = SPD_25MPH
		elif traffic_sign is 'speedLimit45':
			# Go a bit faster
			# Set max speed to a higher number
			self.speed_limit = SPD_45MPH
		
		if self.stopping is True:
			# Assume no change in distance from last time
			if delta_stop is None:
				delta_stop = self.prev_delta

			# Get P component
			P = delta_stop * Kp

			# Get D component
			D = (delta_stop - prev_delta) * Kd

			drv_out = P + D
		else:
			drv_out = self.speed_limit

		if drv_out > self.speed_limit:
			drv_out = self.speed_limit
		self.drv_pub.publish(drv_out)

def main() :

	rospy.init_node('driver', anonymous=True)
	traffic_node = Driver()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
