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

class ShapeDetector:
	def __init__(self):
		pass
 
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		# if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"
 
		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)
 
			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
 
		# if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			shape = "pentagon"
 
		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"
 
		# return the name of the shape
		return shape

class TrafficFind:
	def __init__(self):
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.find_sign)
		self.sign_pub = rospy.Publisher('/traffic_sign', Image, queue_size=10)
		
	def find_sign(self, data):
		# Do CV wizardry to find the traffic sign in a brightly colored square and send segment of image
		# Get current parameter values and store them
		color_num = rospy.get_param('~color', 60)
		sensitivity = rospy.get_param('~sensitivity', 30)
    
		cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
		image = cv_image
		bgr_image = cv2.medianBlur(cv_image, 3)
		#bgr_image = cv_image

		# Convert input image to HSV
		hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    
		# Threshold the HSV image, keep only the green pixels
		green_hue_image = cv2.inRange(hsv_image, (color_num - sensitivity, 100, 100), (color_num + sensitivity, 255, 255))
		thresh = cv2.GaussianBlur(green_hue_image, (9, 9), 2, 2)
		
		# Extract contours
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		sd = ShapeDetector()
		
		shape_no = 0
		largest_c = None
		for c in cnts:
			if cv2.contourArea(c) >= 2000:
				if largest_c is not None:
					if cv2.contourArea(c) > cv2.contourArea(largest_c):
						larg
				else:
					largest_c = c 
			
			if cv2.contourArea(c) >= 2000:
				# compute the center of the contour, then detect the name of the
				# shape using only the contour
				M = cv2.moments(c)
				cX = int((M["m10"] / M["m00"]))
				cY = int((M["m01"] / M["m00"]))
				shape = sd.detect(c)
				shape_no += 1
		
				# multiply the contour (x, y)-coordinates by the resize ratio,
				# then draw the contours and the name of the shape on the image
				c = c.astype("float")
				#c *= ratio
				c = c.astype("int")
				print (shape + ' ' + str(shape_no) + ' is ' + str(cv2.contourArea(c)))
				cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				cv2.putText(image, shape + ' ' + str(shape_no), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			
		self.sign_pub.publish(bridge.cv2_to_imgmsg(image, "rgb8"))

def main() :

	rospy.init_node('img_proc', anonymous=True)
	traffic_node = TrafficFind()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

