
import rospy
import rospkg
import rosparam
import yaml
# import dynamic_reconfigure.client
import os
import cv2
import numpy as np
import time
# import argparse
from std_msgs.msg import Bool, Int8, Int16MultiArray, UInt8
from datetime import datetime
from camera_params import *

class CameraCapture:

	def __init__(self):

		rospy.init_node("camera_caputure_node", anonymous=True)

		self.video1 = cv2.VideoCapture("/dev/video0")

		#########################
		##### ROS Yaml file #####
		#########################
		sv_node = "camera_dyn_sv_params"

		## get parameter from rosparam server that we just loaded above
		self.focus = focus
		self.contrast = contrast
		self.exposure = exposure
		self.saturation = saturation
		self.brightness = brightness
		self.wb = wb

		rospy.loginfo("Using these parameters to server")
		rospy.loginfo("focus : {:}".format(self.focus))
		rospy.loginfo("contrast : {:}".format(self.contrast))
		rospy.loginfo("exposure : {:}".format(self.exposure))
		rospy.loginfo("saturation : {:}".format(self.saturation))
		rospy.loginfo("brightness : {:}".format(self.brightness))
		rospy.loginfo("wb : {:}".format(self.wb))

		###############################
		##### Dynamic reconfigure #####
		###############################
		# print("Wait for server node...")
		# try:
		# 	## This we will get live data from rqt_reconfigure
		# 	self.client = dynamic_reconfigure.client.Client(sv_node, timeout=1, config_callback=self.param_callback)
		# 	print("Open rqt_reconfigure to tune paramters realtime")
		# except rospy.ROSException:
		# 	print("Server node is not alive, load parameters from CameraParams.yaml file")
		# 	pass

		######################
		### Camera setting ###
		######################
		self.frame_height = 2160	#2160	#1080
		self.frame_width = 3840	#3840	#1920

		self.FHD = (1920,1080)
		self.HD = (1280,720)

		self.video1.set(3, self.frame_width)
		self.video1.set(4, self.frame_height)
		self.video1.set(cv2.CAP_PROP_AUTOFOCUS, 0)
		self.video1.set(cv2.CAP_PROP_AUTO_WB, 0)
		self.video1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

		self.video1.set(cv2.CAP_PROP_FOCUS, self.focus)
		self.video1.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
		self.video1.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
		self.video1.set(cv2.CAP_PROP_WB_TEMPERATURE, self.wb)
		self.video1.set(cv2.CAP_PROP_CONTRAST, self.contrast)
		self.video1.set(cv2.CAP_PROP_SATURATION, self.saturation)
		print("============= Camera =============")
		print("Get focus", self.video1.get(cv2.CAP_PROP_FOCUS))
		print("Get contrast", self.video1.get(cv2.CAP_PROP_CONTRAST))
		print("Get exposure ", self.video1.get(cv2.CAP_PROP_EXPOSURE))
		print("Get saturation", self.video1.get(cv2.CAP_PROP_SATURATION))
		print("Get brightness ", self.video1.get(cv2.CAP_PROP_BRIGHTNESS))
		print("Get wb temp ", self.video1.get(cv2.CAP_PROP_WB_TEMPERATURE))


		if not self.video1.isOpened():
			raise SystemExit('ERROR: failed to open camear1!')

		#######################
		### local variables ###
		#######################
		self.cart_mode = 1
		self.ch7 = 1024
		self.prev_ch7 = self.ch7
		self.capture_time = 3.0
		self.image_counter = 1

		self.shelf_number = 1
		self.shelf_number_nav = 1
		self.prev_shelf_number = self.shelf_number
		self.prev_shelf_number_nav = self.shelf_number_nav

		######################
		### File directory ###
		######################
		project_dir = os.getcwd()
		self.images_logger_path = os.path.join(project_dir, "images_logger")
		if not os.path.exists(self.images_logger_path):
			os.mkdir(self.images_logger_path)

		self.init_sub_folder = True

		###############
		### Pub/Sub ###
		###############
		rospy.Subscriber("/nav/camera_shutter", Bool, self.shutter_callback)
		rospy.Subscriber("/nav/shelf_number", Int8, self.shelf_number_callback)
		rospy.Subscriber("/nav/shelf_number_nav", Int8, self.shelf_number_nav_callback)
		rospy.Subscriber("/jmoab/cart_mode", UInt8, self.cart_mode_callback)
		rospy.Subscriber("/jmoab/sbus_rc_ch", Int16MultiArray, self.sbus_rc_callback)

		############
		### Loop ###
		############

		self.loop()

		rospy.spin()

	##########################
	### callback functions ###
	##########################
	def shutter_callback(self, msg):
		# print(msg)
		self.shutter = msg.data

	def shelf_number_callback(self, msg):
		self.prev_shelf_number = self.shelf_number
		self.shelf_number = msg.data

		if self.prev_shelf_number != self.shelf_number:
			self.image_counter = 1

	def shelf_number_nav_callback(self, msg):
		self.prev_shelf_number_nav = self.shelf_number_nav
		self.shelf_number_nav = msg.data

		if self.prev_shelf_number_nav != self.shelf_number_nav:
			self.image_counter = 1

	def cart_mode_callback(self, msg):
		# print(msg)
		self.cart_mode = msg.data

	def sbus_rc_callback(self, msg):
		# print(msg)
		self.prev_ch7 = self.ch7
		self.ch7 = msg.data[6]

		if (self.prev_ch7 != self.ch7) and (self.ch7 < 1500):
			self.init_sub_folder = True

	##########
	## Loop ##
	##########
	def loop(self):

		rate = rospy.Rate(100)
		last_capture_stamp = time.time()

		print("Start camera_capture_node")
		while not rospy.is_shutdown():

			now = datetime.now()

			_, image = self.video1.read()

			## must be in auto-mode, enable navigation, and got shutter flag
			if (self.cart_mode == 2) and (self.ch7 > 1500) and self.shutter:
				
				if self.init_sub_folder:
					
					folder_name = now.strftime("%Y%m%d_%H%M%S")
					sub_store_path = os.path.join(self.images_logger_path, folder_name)
					print("Auto mode recording, data will be stored at {:}".format(sub_store_path))
					if not os.path.exists(sub_store_path):
						os.mkdir(sub_store_path)
					self.init_sub_folder = False

				if ((time.time() - last_capture_stamp) >= self.capture_time):
					image_name = "{:d}_{:d}_{:d}.jpg".format(self.shelf_number, self.shelf_number_nav, self.image_counter)
					image_store_path = os.path.join(sub_store_path, image_name)
					print("saved {:}".format(image_store_path))
					cv2.imwrite(image_store_path, image)
					last_capture_stamp = time.time()
					self.image_counter += 1


			# cv2.imshow("testWebcam", cv2.resize(image, self.FHD, interpolation=cv2.INTER_AREA))

			# cv2.waitKey(1)

			if False:
				print("mode: {:d} ch7: {:d} shutter: {}".format(\
					self.cart_mode, self.ch7, self.shutter))

			rate.sleep()

		self.video1.release()
		cv2.destroyAllWindows()





if __name__ == "__main__":

	CC = CameraCapture()

