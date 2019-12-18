#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import imutils
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
   
bridge = CvBridge()

class TrafficFind:
	def __init__(self):
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.find_sign)
		self.sign_pub = rospy.Publisher('/traffic_sign', Image, queue_size=10)
		self.debug_pub = rospy.Publisher('/debug_img', Image, queue_size=10)
		
	def find_sign(self, data):
		# Get current parameter values and store them
		color_num = rospy.get_param('~color', 60) # Color to identify by Hue value in HSV image (Green is 60)
		sensitivity = rospy.get_param('~sensitivity', 30) # Sensitivity to hue when identifying color in thesholded image
		self.detection_box = rospy.get_param('~use_box', True) # Look for detection box or sign itself (Only detection box coded)
    
		cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
		image = cv_image
		bgr_image = cv2.medianBlur(cv_image, 3)
		#bgr_image = cv_image

		# Convert input image to HSV
		hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    
		if self.detection_box is True:
			# Detect a sign within a green bounding box
			# Threshold the HSV image, keep only the green pixels
			green_hue_image = cv2.inRange(hsv_image, (color_num - sensitivity, 100, 100), (color_num + sensitivity, 255, 255))
			thresh = cv2.GaussianBlur(green_hue_image, (9, 9), 2, 2)
			self.debug_pub.publish(bridge.cv2_to_imgmsg(thresh))
			
			# Mask detected pixels in image to white pixels to help with classification later
			image[green_hue_image == 255] = 255
		else:
			# Try to detect the actual sign with contours
			pass
		
		# Extract contours
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		
		# Run detection and cropping on large contours.
		for c in cnts:
			if cv2.contourArea(c) >= 2000:
				self.detect(c, image)
			
	def detect(self, c, img):
		# approximate the contour
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		if self.detection_box is True:
			if len(approx) == 4:
				# Get bounding box coordinates and centroids
				(x, y, w, h) = cv2.boundingRect(approx)
				M = cv2.moments(approx)
				cX = int((M["m10"] / M["m00"]))
				cY = int((M["m01"] / M["m00"]))

				# Crop image using bounding box
				cropped_image = img[y:(y+h), x:(x+w)]

				# Prepare image for ROS msg and add centroid and area of contour to message
				img_out = bridge.cv2_to_imgmsg(cropped_image, "rgb8")
				img_out.header.frame_id = 'centroid(' + str(cX) + ','+ str(cY)+ ')'
				img_out.header.frame_id += ' area(' + str(cv2.contourArea(c))+ ')'
				self.sign_pub.publish(img_out)
				return True
			else:
				return False
		else:
			return False

def main() :

	rospy.init_node('sign_detect', anonymous=True)
	traffic_node = TrafficFind()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

