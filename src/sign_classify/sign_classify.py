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
import numpy as np
import regex as re
from PIL import Image as imgPIL

model_dir = '/home/perseus/tmp/traffic_sign_addedneurons/0.1'
bridge = CvBridge()

# Should match image dimensions in CNN initial layer
IMG_WIDTH = 32
IMG_HEIGHT = 32

CLASS_NAMES = ['AddedLane', 'KeepRight', 'leftTurn', 'merge', 'pedestrianCrossing', 'school',
    'signalAhead', 'speedLimit25', 'speedLimit30', 'speedLimit35', 'speedLimit45',
    'stopAhead', 'Stopsign', 'Yield']
   
class TrafficSign:
	def __init__(self):
		self.model = tf.keras.models.load_model(str(model_dir))
		self.img_sub = rospy.Subscriber("/traffic_sign", Image, self.process_sign)
		self.sign_pub = rospy.Publisher('/sign_type', String, queue_size=10)
		
	def process_sign(self, data):
		# Identify the sign
		sign_name = self.which_sign(data)
		print(sign_name)
		# How far away is it?
		#area = re.
	
	def which_sign(self, data):
		img_in = bridge.imgmsg_to_cv2(data, 'rgb8')
		x = cv2.cvtColor(img_in, cv2.COLOR_BGR2RGB)
		x = imgPIL.fromarray(x)
		x = x.resize((32,32))
		x = tf.keras.preprocessing.image.img_to_array(x)
		x = tf.keras.applications.mobilenet.preprocess_input(
			x[tf.newaxis,...])
		predictions = self.model.predict(x)
		predicted_label = np.argmax(predictions[0])
		predicted_label = CLASS_NAMES[predicted_label]
		#print("Prediction on image: " + predicted_label)
		#print(data.header.frame_id)
		#self.sign_pub.publish(predicted_label)
		return predicted_label
		

def main() :

	rospy.init_node('sign_classify', anonymous=True)
	traffic_node = TrafficSign()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

