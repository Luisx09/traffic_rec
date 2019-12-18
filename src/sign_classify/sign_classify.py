#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

# TensorFlow and tf.keras
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import datasets, layers, models
import numpy as np
import regex as re
from PIL import Image as imgPIL

# Address to SavedModel
model_dir = './tmp/robotics_traffic/0.1'
bridge = CvBridge()

# Should match image dimensions in CNN initial layer
IMG_WIDTH = 32
IMG_HEIGHT = 32

CLASS_NAMES = ['speedLimit25', 'speedLimit45', 'stopAhead', 'Stopsign']
   
class TrafficSign:
	def __init__(self):
		# Load message from address in root directory
		self.model = tf.keras.models.load_model(str(model_dir))
		self.img_sub = rospy.Subscriber("/traffic_sign", Image, self.process_sign)
		self.sign_pub = rospy.Publisher('/sign_data', Header, queue_size=10)
		
	def process_sign(self, data):
		# Identify the sign
		sign_name, certain = self.which_sign(data)
		print(sign_name, certain)

		# Get area of detected sign from image frame ID.
		info = data.header.frame_id
		area = re.search('area\((.*)\)', info).group(1)
		
		# Do some math to find distance to sign (in mm).
		calc_dist = 43017.35 * (float(area) ** -0.51668)
		print (area + ' is ' + str(calc_dist) + " mm")

		# Only send message if confident in data.
		if certain == True:
			# Assemble message with sign data and publish
			msg_out = Header()
			msg_out.frame_id = sign_name
			msg_out.seq	= calc_dist
			self.sign_pub.publish(msg_out)
		
	
	def which_sign(self, data):
		# Convert Image message to CV format, then pre-process with Keras
		img_in = bridge.imgmsg_to_cv2(data, 'rgb8')
		x = cv2.cvtColor(img_in, cv2.COLOR_BGR2RGB)
		x = imgPIL.fromarray(x)
		x = x.resize((32,32))
		x = tf.keras.preprocessing.image.img_to_array(x)
		x = tf.keras.applications.mobilenet.preprocess_input(
			x[tf.newaxis,...])
		
		# Run prediction on processed image
		predictions = self.model.predict(x)
		predicted_label = np.argmax(predictions[0])

		#Determine confidence in prediction
		certainty = 100*np.max(predictions[0]) 
		print (certainty)
		certain = (certainty > 90.0)
		predicted_label = CLASS_NAMES[predicted_label]
		return predicted_label, certain
		

def main() :

	rospy.init_node('sign_classify', anonymous=True)
	traffic_node = TrafficSign()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

