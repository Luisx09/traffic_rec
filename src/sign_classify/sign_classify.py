#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# TensorFlow and tf.keras
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import datasets, layers, models

model_dir = '~/tmp/traffic_sign_addedneurons/0.1/'

# Should match image dimensions in CNN initial layer
IMG_WIDTH = 32
IMG_HEIGHT = 32

CLASS_NAMES = ['AddedLane', 'KeepRight', 'leftTurn', 'merge', 'pedestrianCrossing', 'school',
    'signalAhead', 'speedLimit25', 'speedLimit30', 'speedLimit35', 'speedLimit45',
    'stopAhead', 'Stopsign', 'Yield']
   
class TrafficSign:
	def __init__(self):
		self.model = tf.keras.models.load_model(model_dir)
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, which_sign)
		self.sign_pub = rospy.Publisher('/sign', String, queue_size=10)
		
	def which_sign(self, data):
		x = bridge.imgmsg_to_cv2(data, 'rgb8')
		x = cv2.resize(x, (32, 32))
		x = x[...,::-1]
		predictions = loaded.predict(tf.constant(x))
		predicted_label = np.argmax(predictions[0])
		print("Prediction on image:\n", CLASS_NAMES[predicted_label])
		

def main() :

	rospy.init_node('img_proc', anonymous=True)
	traffic_node = TrafficSign()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

