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
   
class Driver:
	def __init__(self):
		self.model = tf.keras.models.load_model(model_dir)
		self.sign_sub = rospy.Subscriber("/sign_data", Header, drive_response)
		self.drv_pub = rospy.Publisher('/drv_vel', Float64, queue_size=10)\
		self.prev_dist = None
		
	def drive_response(self, data):
		traffic_sign = data.frame_id
		dist = data.seq # Distance in mm

		if traffic_sign is 'Stopsign':
			# Stop
			print ("It's time to stop!")
			delta_stop = dist
		elif traffic_sign is 'speedLimit25':
			# Go slow
			# Set max speed to a low number
		elif traffic_sign is 'speedLimit45':
			# Go a bit faster
			# Set max speed to a higher number
		

def main() :

	rospy.init_node('driver', anonymous=True)
	traffic_node = Driver()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
