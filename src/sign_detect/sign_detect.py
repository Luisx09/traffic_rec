#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
   
class TrafficFind:
	def __init__(self):
		self.model = tf.keras.models.load_model(model_dir)
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, find_sign)
		self.sign_pub = rospy.Publisher('/traffic_sign', Image, queue_size=10)
		
	def find_sign(self, data):
		# Do CV wizardry to find the traffic sign in a brightly colored square and send segment of image
		

def main() :

	rospy.init_node('img_proc', anonymous=True)
	traffic_node = TrafficFind()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

