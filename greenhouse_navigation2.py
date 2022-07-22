#!/usr/bin/env python

#this bot is for HOUSE 1, 2nd OFF

import rospy
import rospkg
import rosparam
import yaml
import dynamic_reconfigure.client
import os

import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray, UInt8, Bool
import geometry_msgs.msg
from geometry_msgs.msg import Polygon, PolygonStamped, Point32, Twist
import tf2_ros
import time
from simple_pid import PID
import argparse
import socket
import struct
import pickle
import time
from custom_params import wf_decision_list, wf_setpoint_list


class GreenhouseNav:

	def __init__(self, param_file):

		rospy.init_node("greenhouse_nav_node", anonymous=True)

		#########################
		##### ROS Yaml file #####
		#########################
		sv_node = "greenhouse_nav_params_server_node"
		# https://answers.ros.org/question/169866/load-yaml-with-code/
		# load yaml file to rosparam server without running server on python
		f = open(param_file, 'r')
		yamlfile = yaml.load(f, Loader=yaml.SafeLoader)
		rosparam.upload_params("/", yamlfile)

		## get parameter from rosparam server that we just loaded above
		self.vx_max = rosparam.get_param(sv_node+"/vx_max")
		self.wz_max = rosparam.get_param(sv_node+"/wz_max")
		self.vx_wall_follow = rosparam.get_param(sv_node+"/vx_wall_follow")
		self.wz_wall_follow = rosparam.get_param(sv_node+"/wz_wall_follow")
		self.vx_lane_change = rosparam.get_param(sv_node+"/vx_lane_change")
		self.wz_lane_change = rosparam.get_param(sv_node+"/wz_lane_change")
		self.wz_skidding = rosparam.get_param(sv_node+"/wz_skidding")

		self.front_stop_dist = rosparam.get_param(sv_node+"/front_stop_dist")
		self.front_min_scan_ang = rosparam.get_param(sv_node+"/front_min_scan_ang")
		self.front_max_scan_ang = rosparam.get_param(sv_node+"/front_max_scan_ang")

		self.left_wf_min_scan_ang = rosparam.get_param(sv_node+"/left_wf_min_scan_ang")
		self.left_wf_max_scan_ang = rosparam.get_param(sv_node+"/left_wf_max_scan_ang")
		self.left_lct_min_scan_ang = rosparam.get_param(sv_node+"/left_lct_min_scan_ang")
		self.left_lct_max_scan_ang = rosparam.get_param(sv_node+"/left_lct_max_scan_ang")
		self.left_lct_dist = rosparam.get_param(sv_node+"/left_lct_dist")

		self.right_wf_min_scan_ang = rosparam.get_param(sv_node+"/right_wf_min_scan_ang")
		self.right_wf_max_scan_ang = rosparam.get_param(sv_node+"/right_wf_max_scan_ang")
		self.right_lct_min_scan_ang = rosparam.get_param(sv_node+"/right_lct_min_scan_ang")
		self.right_lct_max_scan_ang = rosparam.get_param(sv_node+"/right_lct_max_scan_ang")
		self.right_lct_dist = rosparam.get_param(sv_node+"/right_lct_dist")

		self.right_lc_min_scan_ang = rosparam.get_param(sv_node+"/right_lc_min_scan_ang")
		self.right_lc_max_scan_ang = rosparam.get_param(sv_node+"/right_lc_max_scan_ang")

		self.lc_dist_step1 = rosparam.get_param(sv_node+"/lc_dist_step1")
		self.lc_hdg_step1 = rosparam.get_param(sv_node+"/lc_hdg_step1")
		self.lc_dist_step2 = rosparam.get_param(sv_node+"/lc_dist_step2")
		self.lc_hdg_step2 = rosparam.get_param(sv_node+"/lc_hdg_step2")

		self.wf_p = rosparam.get_param(sv_node+"/wf_p")
		self.wf_i = rosparam.get_param(sv_node+"/wf_i")
		self.wf_d = rosparam.get_param(sv_node+"/wf_d")

		# self.lc_p = rosparam.get_param(sv_node+"/lc_p")
		# self.lc_i = rosparam.get_param(sv_node+"/lc_i")
		# self.lc_d = rosparam.get_param(sv_node+"/lc_d")

		# self.rh_p = rosparam.get_param(sv_node+"/rh_p")
		# self.rh_i = rosparam.get_param(sv_node+"/rh_i")
		# self.rh_d = rosparam.get_param(sv_node+"/rh_d")

		self.ut_p = rosparam.get_param(sv_node+"/ut_p")
		self.ut_i = rosparam.get_param(sv_node+"/ut_i")
		self.ut_d = rosparam.get_param(sv_node+"/ut_d")

		rospy.loginfo("vx_max : {:}".format(self.vx_max))
		rospy.loginfo("wz_max : {:}".format(self.wz_max))
		rospy.loginfo("vx_wall_follow : {:}".format(self.vx_wall_follow))
		rospy.loginfo("wz_wall_follow : {:}".format(self.wz_wall_follow))
		rospy.loginfo("vx_lane_change : {:}".format(self.vx_lane_change))
		rospy.loginfo("wz_lane_change : {:}".format(self.wz_lane_change))
		rospy.loginfo("wz_skidding : {:}".format(self.wz_skidding))

		rospy.loginfo("front_stop_dist : {:}".format(self.front_stop_dist))
		rospy.loginfo("front_min_scan_ang : {:}".format(self.front_min_scan_ang))
		rospy.loginfo("front_max_scan_ang : {:}".format(self.front_max_scan_ang))

		rospy.loginfo("left_wf_min_scan_ang : {:}".format(self.left_wf_min_scan_ang))
		rospy.loginfo("left_wf_max_scan_ang : {:}".format(self.left_wf_max_scan_ang))
		rospy.loginfo("left_lct_min_scan_ang : {:}".format(self.left_lct_min_scan_ang))
		rospy.loginfo("left_lct_max_scan_ang : {:}".format(self.left_lct_max_scan_ang))
		rospy.loginfo("left_lct_dist : {:}".format(self.left_lct_dist))

		rospy.loginfo("right_wf_min_scan_ang : {:}".format(self.right_wf_min_scan_ang))
		rospy.loginfo("right_wf_max_scan_ang : {:}".format(self.right_wf_max_scan_ang))
		rospy.loginfo("right_lct_min_scan_ang : {:}".format(self.right_lct_min_scan_ang))
		rospy.loginfo("right_lct_max_scan_ang : {:}".format(self.right_lct_max_scan_ang))
		rospy.loginfo("right_lct_dist : {:}".format(self.right_lct_dist))
		rospy.loginfo("right_lc_min_scan_ang : {:}".format(self.right_lc_min_scan_ang))
		rospy.loginfo("right_lc_max_scan_ang : {:}".format(self.right_lc_max_scan_ang))

		rospy.loginfo("lc_dist_step1 : {:}".format(self.lc_dist_step1))
		rospy.loginfo("lc_hdg_step1 : {:}".format(self.lc_hdg_step1))
		rospy.loginfo("lc_dist_step2 : {:}".format(self.lc_dist_step2))
		rospy.loginfo("lc_hdg_step2 : {:}".format(self.lc_hdg_step2))

		rospy.loginfo("wf_p : {:}".format(self.wf_p))
		rospy.loginfo("wf_i : {:}".format(self.wf_i))
		rospy.loginfo("wf_d : {:}".format(self.wf_d))

		# rospy.loginfo("lc_p : {:}".format(self.lc_p))
		# rospy.loginfo("lc_i : {:}".format(self.lc_i))
		# rospy.loginfo("lc_d : {:}".format(self.lc_d))
		
		rospy.loginfo("ut_p : {:}".format(self.ut_p))
		rospy.loginfo("ut_i : {:}".format(self.ut_i))
		rospy.loginfo("ut_d : {:}".format(self.ut_d))

		###############################
		##### Dynamic reconfigure #####
		###############################
		print("Wait for server node...")
		try:
			## This we will get live data from rqt_reconfigure
			self.client = dynamic_reconfigure.client.Client(sv_node, timeout=2, config_callback=self.param_callback)
			print("Open rqt_reconfigure to tune paramters realtime")
		except rospy.ROSException:
			print("Server node is not alive, load parameters from GreenhouseNavParams.yaml file")
			pass

		################################
		##### Laserscan parameters #####
		################################
		## left wall-follow
		self.left_wf_first_idx = self.lidarAng_to_lidarIdx(self.left_wf_min_scan_ang)
		self.left_wf_last_idx =  self.lidarAng_to_lidarIdx(self.left_wf_max_scan_ang)

		## right wall-follow
		self.right_wf_first_idx = self.lidarAng_to_lidarIdx(self.right_wf_min_scan_ang) 
		self.right_wf_last_idx = self.lidarAng_to_lidarIdx(self.right_wf_max_scan_ang)

		## left lane-change trigger
		self.left_lct_first_idx = self.lidarAng_to_lidarIdx(self.left_lct_min_scan_ang) 
		self.left_lct_last_idx = self.lidarAng_to_lidarIdx(self.left_lct_max_scan_ang) 

		## right lane-change trigger
		self.right_lct_first_idx = self.lidarAng_to_lidarIdx(self.right_lct_min_scan_ang) 
		self.right_lct_last_idx = self.lidarAng_to_lidarIdx(self.right_lct_max_scan_ang) 

		## right lane-changing
		self.right_lc_first_idx = self.lidarAng_to_lidarIdx(self.right_lc_min_scan_ang) 
		self.right_lc_last_idx = self.lidarAng_to_lidarIdx(self.right_lc_max_scan_ang)

		## front stop
		self.front_stop_first_idx = self.lidarAng_to_lidarIdx(self.front_min_scan_ang) 
		self.front_stop_last_idx = self.lidarAng_to_lidarIdx(self.front_max_scan_ang)	

		self.left_wf_min_dist = 0.5
		self.right_wf_min_dist = 0.5
		self.left_lct_min_dist = 0.0
		self.right_lct_min_dist = 0.0
		self.right_lc_min_dist = 0.5
		self.front_stop_min_dist = 1.0

		self.prev_left_wf_min_dist = self.left_wf_min_dist
		self.prev_right_wf_min_dist = self.right_wf_min_dist 
		self.prev_left_lct_min_dist = self.left_lct_min_dist
		self.prev_right_lct_min_dist = self.right_lct_min_dist
		self.prev_right_lc_min_dist = self.right_lc_min_dist
		self.prev_front_stop_min_dist = self.front_stop_min_dist
		
		###########################
		##### Cart parameters #####
		###########################
		self.vx = 0.0
		self.wz = 0.0
		self.cart_mode = 1

		self.shelf_number = 13
		self.shelf_number_nav = 2
		self.nav_step2_done = True

		self.ch7 = 1024
		self.prev_ch7 = 1024
		self.wz_adj = 0.0
		self.nav_step = 1
		self.right_wf_flag = wf_decision_list[self.shelf_number-1][self.shelf_number_nav-1]
		self.lc_step = 1
		self.lc_lock = False
		self.scan_ready = False
		self.atcart_ready = False
		self.ahrs_ready = False
		self.mission_done = False
		

		#######################
		##### PID setting #####
		#######################
		## PID wall follow ##
		self.pid_wf = PID(self.wf_p, self.wf_i, self.wf_d, setpoint=wf_setpoint_list[self.shelf_number-1][self.shelf_number_nav-1])
		self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)
		self.pid_wf.sample_time = 0.001
		self.pid_wf.output_limits = (-100.0, 100.0)
		self.pid_wf.auto_mode = True

		## PID lane change ##
		# self.pid_lc = PID(self.lc_p, self.lc_i, self.lc_d, setpoint=0.5)
		# self.pid_lc.tunings = (self.lc_p, self.lc_i, self.lc_d)
		# self.pid_lc.sample_time = 0.001
		# self.pid_lc.output_limits = (-100.0, 100.0)
		# self.pid_lc.auto_mode = False

		# ################
		# ## PID Reheading ##
		# ################
		# self.pid_rh = PID(self.rh_p, self.rh_i, self.rh_d, setpoint=self.rh_setpoint)
		# self.pid_rh.tunings = (self.rh_p, self.rh_i, self.rh_d)
		# self.pid_rh.sample_time = 0.001
		# self.rh_out_range = 100.0
		# self.pid_rh.output_limits = (-self.rh_out_range, self.rh_out_range)
		# self.pid_rh.auto_mode = False

		## PID U-Turn ##
		self.pid_ut = PID(self.ut_p, self.ut_i, self.ut_d, setpoint=180.0)
		self.pid_ut.tunings = (self.ut_p, self.ut_i, self.ut_d)
		self.pid_ut.sample_time = 0.001
		self.pid_ut.output_limits = (-100.0, 100.0)
		self.pid_ut.auto_mode = False


		###################
		##### Pub/Sub #####
		###################
		### laserscan repeat topics ###
		self.left_wf_scan_pub = rospy.Publisher("/left_wf_scan", LaserScan, queue_size=1)
		self.right_wf_scan_pub = rospy.Publisher("/right_wf_scan", LaserScan, queue_size=1)
		self.front_stop_scan_pub = rospy.Publisher("/front_stop_scan", LaserScan, queue_size=1)
		self.left_lct_scan_pub = rospy.Publisher("/left_lct_scan", LaserScan, queue_size=1)
		self.right_lct_scan_pub = rospy.Publisher("/right_lct_scan", LaserScan, queue_size=1)
		self.right_lc_scan_pub = rospy.Publisher("/right_lc_scan", LaserScan, queue_size=1)

		### cart control topics ###
		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		self.wheels_cmd_pub = rospy.Publisher("/jmoab/wheels_cmd", Float32MultiArray, queue_size=1)

		### camera trigger topics ###
		self.camera_shutter_pub = rospy.Publisher("/nav/camera_shutter", Bool, queue_size=1)
		self.camera_shutter_msg = Bool()


		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/jmoab/sbus_rc_ch", Int16MultiArray, self.sbus_rc_callback)
		rospy.Subscriber("/jmoab/ahrs", Float32MultiArray, self.ahrs_callback)
		rospy.Subscriber("/jmoab/cart_mode", UInt8, self.atcart_mode_callback)

		################
		##### Loop #####
		################
		self.run()

		rospy.spin()

	##########################
	### callback functions ###
	##########################
	def param_callback(self, config):
		self.vx_max = config["vx_max"]
		self.wz_max = config["wz_max"]
		self.vx_wall_follow = config["vx_wall_follow"]
		self.wz_wall_follow = config["wz_wall_follow"]
		self.vx_lane_change = config["vx_lane_change"]
		self.wz_lane_change = config["wz_lane_change"]
		self.wz_skidding = config["wz_skidding"]

		self.front_stop_dist = config["front_stop_dist"]
		self.front_min_scan_ang = config["front_min_scan_ang"]
		self.front_max_scan_ang = config["front_max_scan_ang"]

		self.left_wf_min_scan_ang = config["left_wf_min_scan_ang"]
		self.left_wf_max_scan_ang = config["left_wf_max_scan_ang"]
		self.left_lct_min_scan_ang = config["left_lct_min_scan_ang"]
		self.left_lct_max_scan_ang = config["left_lct_max_scan_ang"]
		self.left_lct_dist =config["left_lct_dist"] 

		self.right_wf_min_scan_ang = config["right_wf_min_scan_ang"]
		self.right_wf_max_scan_ang = config["right_wf_max_scan_ang"]
		self.right_lct_min_scan_ang = config["right_lct_min_scan_ang"]
		self.right_lct_max_scan_ang = config["right_lct_max_scan_ang"]
		self.right_lct_dist = config["right_lct_dist"]
		self.right_lc_min_scan_ang = config["right_lc_min_scan_ang"]
		self.right_lc_max_scan_ang = config["right_lc_max_scan_ang"]

		self.lc_dist_step1 = config["lc_dist_step1"]
		self.lc_hdg_step1 = config["lc_hdg_step1"]
		self.lc_dist_step2 = config["lc_dist_step2"]
		self.lc_hdg_step2 = config["lc_hdg_step2"]

		self.wf_p = config["wf_p"]
		self.wf_i = config["wf_i"]
		self.wf_d = config["wf_d"]

		# self.lc_p = config["lc_p"]
		# self.lc_i = config["lc_i"]
		# self.lc_d = config["lc_d"]

		self.ut_p = config["ut_p"]
		self.ut_i = config["ut_i"]
		self.ut_d = config["ut_d"]

		self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)
		self.pid_ut.tunings = (self.ut_p, self.ut_i, self.ut_d)

		## left wall-follow
		self.left_wf_first_idx = self.lidarAng_to_lidarIdx(self.left_wf_min_scan_ang)
		self.left_wf_last_idx =  self.lidarAng_to_lidarIdx(self.left_wf_max_scan_ang)

		## right wall-follow
		self.right_wf_first_idx = self.lidarAng_to_lidarIdx(self.right_wf_min_scan_ang) 
		self.right_wf_last_idx = self.lidarAng_to_lidarIdx(self.right_wf_max_scan_ang)

		## left lane-change trigger
		self.left_lct_first_idx = self.lidarAng_to_lidarIdx(self.left_lct_min_scan_ang) 
		self.left_lct_last_idx = self.lidarAng_to_lidarIdx(self.left_lct_max_scan_ang) 

		## right lane-change trigger
		self.right_lct_first_idx = self.lidarAng_to_lidarIdx(self.right_lct_min_scan_ang) 
		self.right_lct_last_idx = self.lidarAng_to_lidarIdx(self.right_lct_max_scan_ang) 

		## right lane-changing
		self.right_lc_first_idx = self.lidarAng_to_lidarIdx(self.right_lc_min_scan_ang) 
		self.right_lc_last_idx = self.lidarAng_to_lidarIdx(self.right_lc_max_scan_ang)

		## front stop
		self.front_stop_first_idx = self.lidarAng_to_lidarIdx(self.front_min_scan_ang) 
		self.front_stop_last_idx = self.lidarAng_to_lidarIdx(self.front_max_scan_ang)	

		print("----------------------- update params ---------------------")

	def scan_callback(self, msg):

		## copy original scan msg to wall_scan msg
		## and publish wall_scan topic immediately
		left_wf_scan = LaserScan()
		left_wf_scan.header.stamp = rospy.Time.now()
		left_wf_scan.header.frame_id = "laser_frame"
		left_wf_scan.time_increment = msg.time_increment
		left_wf_scan.angle_increment = msg.angle_increment
		left_wf_scan.angle_min = np.radians(self.left_wf_min_scan_ang)
		left_wf_scan.angle_max = np.radians(self.left_wf_max_scan_ang)
		left_wf_scan.scan_time = msg.scan_time
		left_wf_scan.range_min = msg.range_min
		left_wf_scan.range_max = msg.range_max
		left_wf_scan.ranges = msg.ranges[self.left_wf_first_idx:self.left_wf_last_idx]
		left_wf_scan.intensities = msg.intensities[self.left_wf_first_idx:self.left_wf_last_idx]

		self.left_wf_min_dist = self.remove_zero_from_array(left_wf_scan.ranges, self.prev_left_wf_min_dist)
		self.prev_left_wf_min_dist = self.left_wf_min_dist

		right_wf_scan = LaserScan()
		right_wf_scan.header.stamp = rospy.Time.now()
		right_wf_scan.header.frame_id = "laser_frame"
		right_wf_scan.time_increment = msg.time_increment
		right_wf_scan.angle_increment = msg.angle_increment
		right_wf_scan.angle_min = np.radians(self.right_wf_min_scan_ang)
		right_wf_scan.angle_max = np.radians(self.right_wf_max_scan_ang)
		right_wf_scan.scan_time = msg.scan_time
		right_wf_scan.range_min = msg.range_min
		right_wf_scan.range_max = msg.range_max
		right_wf_scan.ranges = msg.ranges[self.right_wf_first_idx:self.right_wf_last_idx]
		right_wf_scan.intensities = msg.intensities[self.right_wf_first_idx:self.right_wf_last_idx]

		self.right_wf_min_dist = self.remove_zero_from_array(right_wf_scan.ranges, self.prev_right_wf_min_dist)
		self.prev_right_wf_min_dist = self.right_wf_min_dist

		front_stop_scan = LaserScan()
		front_stop_scan.header.stamp = rospy.Time.now()
		front_stop_scan.header.frame_id = "laser_frame"
		front_stop_scan.time_increment = msg.time_increment
		front_stop_scan.angle_increment = msg.angle_increment
		front_stop_scan.angle_min = np.radians(self.front_min_scan_ang)
		front_stop_scan.angle_max = np.radians(self.front_max_scan_ang)
		front_stop_scan.scan_time = msg.scan_time
		front_stop_scan.range_min = msg.range_min
		front_stop_scan.range_max = msg.range_max
		front_stop_scan.ranges = msg.ranges[self.front_stop_first_idx:self.front_stop_last_idx]
		front_stop_scan.intensities = msg.intensities[self.front_stop_first_idx:self.front_stop_last_idx]

		self.front_stop_min_dist = self.remove_zero_from_array(front_stop_scan.ranges, self.prev_front_stop_min_dist)
		self.prev_front_stop_min_dist = self.front_stop_min_dist

		left_lct_scan = LaserScan()
		left_lct_scan.header.stamp = rospy.Time.now()
		left_lct_scan.header.frame_id = "laser_frame"
		left_lct_scan.time_increment = msg.time_increment
		left_lct_scan.angle_increment = msg.angle_increment
		left_lct_scan.angle_min = np.radians(self.left_lct_min_scan_ang)
		left_lct_scan.angle_max = np.radians(self.left_lct_max_scan_ang)
		left_lct_scan.scan_time = msg.scan_time
		left_lct_scan.range_min = msg.range_min
		left_lct_scan.range_max = msg.range_max
		left_lct_scan.ranges = msg.ranges[self.left_lct_first_idx:self.left_lct_last_idx]
		left_lct_scan.intensities = msg.intensities[self.left_lct_first_idx:self.left_lct_last_idx]

		self.left_lct_min_dist = self.remove_zero_from_array(left_lct_scan.ranges, self.prev_left_lct_min_dist)
		self.prev_left_lct_min_dist = self.left_lct_min_dist

		right_lct_scan = LaserScan()
		right_lct_scan.header.stamp = rospy.Time.now()
		right_lct_scan.header.frame_id = "laser_frame"
		right_lct_scan.time_increment = msg.time_increment
		right_lct_scan.angle_increment = msg.angle_increment
		right_lct_scan.angle_min = np.radians(self.right_lct_min_scan_ang)
		right_lct_scan.angle_max = np.radians(self.right_lct_max_scan_ang)
		right_lct_scan.scan_time = msg.scan_time
		right_lct_scan.range_min = msg.range_min
		right_lct_scan.range_max = msg.range_max
		right_lct_scan.ranges = msg.ranges[self.right_lct_first_idx:self.right_lct_last_idx]
		right_lct_scan.intensities = msg.intensities[self.right_lct_first_idx:self.right_lct_last_idx]

		self.right_lct_min_dist = self.remove_zero_from_array(right_lct_scan.ranges, self.prev_right_lct_min_dist)
		self.prev_right_lct_min_dist = self.right_lct_min_dist

		right_lc_scan = LaserScan()
		right_lc_scan.header.stamp = rospy.Time.now()
		right_lc_scan.header.frame_id = "laser_frame"
		right_lc_scan.time_increment = msg.time_increment
		right_lc_scan.angle_increment = msg.angle_increment
		right_lc_scan.angle_min = np.radians(self.right_lc_min_scan_ang)
		right_lc_scan.angle_max = np.radians(self.right_lc_max_scan_ang)
		right_lc_scan.scan_time = msg.scan_time
		right_lc_scan.range_min = msg.range_min
		right_lc_scan.range_max = msg.range_max
		right_lc_scan.ranges = msg.ranges[self.right_lc_first_idx:self.right_lc_last_idx]
		right_lc_scan.intensities = msg.intensities[self.right_lc_first_idx:self.right_lc_last_idx]

		self.right_lc_min_dist = self.remove_zero_from_array(right_lc_scan.ranges, self.prev_right_lc_min_dist)
		self.prev_right_lc_min_dist = self.right_lc_min_dist


		self.left_wf_scan_pub.publish(left_wf_scan)
		self.right_wf_scan_pub.publish(right_wf_scan)
		self.front_stop_scan_pub.publish(front_stop_scan)
		self.left_lct_scan_pub.publish(left_lct_scan)
		self.right_lct_scan_pub.publish(right_lct_scan)
		self.right_lc_scan_pub.publish(right_lc_scan)

		self.scan_ready = True


	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

		self.atcart_ready = True

	def ahrs_callback(self, msg):

		self.hdg = msg.data[2]

		self.ahrs_ready = True

	def sbus_rc_callback(self, msg):

		self.prev_ch7 = self.ch7

		## reset mission_done to start again once mission_done already
		if (self.ch7 < 1500) and self.mission_done:
			self.mission_done = False
			self.pid_wf.auto_mode = True
			self.nav_step = 1
			self.shelf_number = 1
			self.shelf_number_nav = 1
			self.pid_wf.setpoint = wf_setpoint_list[self.shelf_number-1][self.shelf_number_nav-1]
			self.right_wf_flag = wf_decision_list[self.shelf_number-1][self.shelf_number_nav-1]
			print("Restart mission again")

		self.ch7 = msg.data[6]

	###################
	### Math helper ###
	###################
	def lidarAng_to_lidarIdx(self, ang):
		return int(self.map_with_limit(ang, -180.0, 180.0, 0.0, 2019.0))

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	def remove_zero_from_array(self, range_list, prev_value):

		array = np.asarray(range_list)
		array = array[array !=0]

		if (len(array) == 0):
			min_value = prev_value
		else:
			array = array[array !=0]
			min_value = np.min(array)

		if np.isinf(min_value):
			# print("Got inf!")
			min_value = prev_value

		return min_value

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target

		return diff_ang, sign

	############
	### Loop ###
	############
	def run(self):
		rate = rospy.Rate(20) # 10hz
		
		disable_nav_stamp = time.time()
		output_pid_log = 0.0
		hdg_diff = 0.0
		allow_uturn_stamp = time.time()
		while not (self.scan_ready and self.atcart_ready and self.ahrs_ready):

			print("Wait for all ready | scan_ready: {} | atcart_ready: {} | ahrs_ready: {}".format(\
				self.scan_ready, self.atcart_ready, self.ahrs_ready))

			time.sleep(1)

		while not rospy.is_shutdown():

			## enable navigation ##
			if self.ch7 > 1500 and (not self.mission_done):


				################################
				## nav_step 1, Wall-following ##
				################################
				if self.nav_step == 1:

					self.allow_uturn_period = time.time() - allow_uturn_stamp

					##################################
					## lane-change trigger checking ##
					##################################
					if ((self.left_lct_min_dist >= self.left_lct_dist) or (self.right_lct_min_dist >= self.right_lct_dist)) and (not self.nav_step2_done):
						self.vx = 0.0
						self.wz = 0.0
						self.nav_step = 2
						last_wf_hdg = self.hdg
						output_pid_wf = 0.0
						self.pid_wf.auto_mode = False
						self.pid_ut.auto_mode = False
						self.lc_step = 1
						print("Finished wall-following, lane-change trig")
						time.sleep(2)

					#################################
					## front-stop trigger checking ##
					#################################
					elif (self.front_stop_min_dist < self.front_stop_dist) and (self.allow_uturn_period > 10.0):
						self.vx = 0.0
						self.wz = 0.0
						self.nav_step = 3
						last_wf_hdg = self.hdg
						output_pid_wf = 0.0
						self.pid_wf.auto_mode = False
						self.pid_ut.auto_mode = True
						print("Finished wall-following, front-stop trig")
						time.sleep(2)
					
					##############################
					## doing wall-following nav ##
					##############################
					else:
						### right wall-follow ###
						if self.right_wf_flag:
							output_pid_wf = self.pid_wf(self.right_wf_min_dist)
							if self.right_wf_min_dist > (self.pid_wf.setpoint+0.01):
								# print("go right")
								self.wz = self.map_with_limit(output_pid_wf, -100.0, 0.0, -self.wz_wall_follow, 0.0)
							elif self.right_wf_min_dist < (self.pid_wf.setpoint-0.01):
								# print("go left")
								self.wz = self.map_with_limit(output_pid_wf, 0.0, 100.0, 0.0, self.wz_wall_follow)
							else:
								# print("go straight")
								self.wz = 0.0

						### left wall-follow ###
						else:
							output_pid_wf = self.pid_wf(self.left_wf_min_dist)
							if self.left_wf_min_dist > (self.pid_wf.setpoint+0.01):
								self.wz = self.map_with_limit(output_pid_wf, -100.0, 0.0, self.wz_wall_follow, 0.0)
							elif self.left_wf_min_dist < (self.pid_wf.setpoint-0.01):
								self.wz = self.map_with_limit(output_pid_wf, 0.0, 100.0, 0.0, -self.wz_wall_follow)
							else:
								self.wz = 0.0

						self.vx = self.vx_wall_follow
					
					pid_mode = "W_F"
					output_pid_log = output_pid_wf

				###############################
				## nav_step 2, Lane-changing ##
				###############################
				elif self.nav_step == 2:
					pid_mode = "L_C"

					hdg_diff, diff_sign = self.find_smallest_diff_ang(last_wf_hdg, self.hdg)
					hdg_diff = self.ConvertTo180Range(hdg_diff*(-diff_sign))

					# output_pid_lc = self.pid_lc(self.right_lc_min_dist)
					# if abs(abs(hdg_diff) - 180.0) > 1.0:
					# 	self.wz = self.map_with_limit(output_pid_lc, -100.0, 100.0, -self.wz_lane_change, self.wz_lane_change)	
					# else:
					# 	self.wz = 0.0
					# 	self.nav_step = 1
					# 	self.pid_wf.auto_mode = True
					# 	self.pid_lc.auto_mode = False

					if self.lc_step == 1:
						if (self.right_lc_min_dist < self.lc_dist_step1) and (not self.lc_lock):
							self.wz = 0.0
						else:
							self.lc_lock = True

							if abs(hdg_diff) < self.lc_hdg_step1:
								self.wz = self.wz_lane_change
							else:
								self.wz = 0.0
								self.lc_step = 2
								self.lc_lock = False

					elif self.lc_step == 2:
						if (self.right_lc_min_dist < self.lc_dist_step2) and (not self.lc_lock):
							self.wz = 0.0
						else:

							self.lc_lock = True

							if abs(hdg_diff) < self.lc_hdg_step2:
								self.wz = self.wz_lane_change
							else:
								self.wz = 0.0
								self.nav_step = 1
								self.lc_step = 1
								self.lc_lock = False
								self.pid_wf.auto_mode = True
								self.nav_step2_done = True
								self.shelf_number_nav = 2
								self.pid_wf.setpoint = wf_setpoint_list[self.shelf_number-1][self.shelf_number_nav-1]
								self.right_wf_flag = wf_decision_list[self.shelf_number-1][self.shelf_number_nav-1]
								allow_uturn_stamp = time.time()
								print("Finished lane-changing")
								time.sleep(2)


					self.vx = self.vx_lane_change
					# output_pid_log = output_pid_lc

					print("nav_step: {:d} lc_step: {:d} mode: {:d} vx: {:.2f} wz: {:.2f} hdg: {:.2f} hdg_diff: {:.2f}   l_wf: {:.2f} r_wf: {:.2f} l_lct: {:.2f} r_lct: {:.2f} f_stop: {:.2f} r_lc: {:.2f}".format(\
					self.nav_step, self.lc_step, self.cart_mode, self.vx, self.wz, self.hdg, hdg_diff,\
					self.left_wf_min_dist, self.right_wf_min_dist, self.left_lct_min_dist, self.right_lct_min_dist, self.front_stop_min_dist, self.right_lc_min_dist))

				###########################
				## nav_step 3, U-turning ##
				###########################
				elif self.nav_step == 3:
					
					hdg_diff, diff_sign = self.find_smallest_diff_ang(last_wf_hdg, self.hdg)
					hdg_diff = self.ConvertTo180Range(hdg_diff*(-diff_sign))

					output_pid_ut = self.pid_ut(abs(hdg_diff))

					if abs(hdg_diff) < 175.0:
						self.wz = self.map_with_limit(output_pid_ut, -100.0, 100.0, -self.wz_skidding, self.wz_skidding)
					else:
						self.wz = 0.0
						self.nav_step = 4
						self.pid_wf.auto_mode = True
						self.pid_ut.auto_mode = False
						self.nav_step2_done = False
						
						print("Finished U-Turn")
						time.sleep(2)


					self.vx = 0.0
					output_pid_log = output_pid_ut
					pid_mode = "U_T"

				#######################################
				## nav_step 4, Checking shelf number ##
				#######################################
				elif self.nav_step == 4:

					if (self.shelf_number == 15) and (self.shelf_number_nav == 2):
						self.mission_done = True
						# self.nav_step = 1
						self.pid_wf.auto_mode = False
						self.pid_ut.auto_mode = False
						print("Done mission")
					else:
						self.mission_done = False
						self.nav_step = 1
						allow_uturn_stamp = time.time()
						self.shelf_number += 1
						self.shelf_number_nav = 1
						print("Updated shelf_number")
						self.pid_wf.setpoint = wf_setpoint_list[self.shelf_number-1][self.shelf_number_nav-1]
						self.right_wf_flag = wf_decision_list[self.shelf_number-1][self.shelf_number_nav-1]
					

				else:
					pid_mode = "NO_"
					print("wtf...")


				cmd_vel_msg = Twist()
				cmd_vel_msg.linear.x = self.vx
				cmd_vel_msg.angular.z = self.wz
				self.cmd_vel_pub.publish(cmd_vel_msg)

				#############################
				## Logging for PID control ##
				#############################
				if (self.nav_step == 1) or (self.nav_step == 3):
					print("nav_step: {:d} mode: {:d} PID: {:} right_wf: {} sp: {:.2f} out_pid: {:.2f} vx: {:.2f} wz: {:.2f} hdg: {:.2f} hdg_diff: {:.2f}   l_wf: {:.2f} r_wf: {:.2f} l_lct: {:.2f} r_lct: {:.2f} f_stop: {:.2f} r_lc: {:.2f}    s_NO: {:d} s_NO_nav: {:d}".format(\
						self.nav_step, self.cart_mode, pid_mode, self.right_wf_flag, self.pid_wf.setpoint, output_pid_log, self.vx, self.wz, self.hdg, hdg_diff,\
						self.left_wf_min_dist, self.right_wf_min_dist, self.left_lct_min_dist, self.right_lct_min_dist, self.front_stop_min_dist, self.right_lc_min_dist,\
						self.shelf_number, self.shelf_number_nav))

				disable_nav_stamp = time.time()
			
			## disable navigation ##
			else:

				if (time.time() - disable_nav_stamp) > 2.0:
					print("Wait for navigation enable switch (ch7)")
					disable_nav_stamp = time.time()

				allow_uturn_stamp = time.time()
				output_pid_log = 0.0
				hdg_diff = 0.0
				self.nav_step = 1


			rate.sleep()




if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Greenhouse Navigation node')
	parser.add_argument('--params_file',
						help="A file path of GreenhouseNavParams.yaml, default is the one in greenhouse_dyn_params/cfg/")

	args = parser.parse_args(rospy.myargv()[1:])
	param_file = args.params_file

	if param_file is None:
		print("Use greenhouse_dyn_params/cfg/GreenhouseNavParams.yaml")
		rospack = rospkg.RosPack()
		server_pkg_path = rospack.get_path("greenhouse_dyn_params")
		yaml_name = "GreenhouseNavParams.yaml"
		yaml_path = os.path.join(server_pkg_path, "cfg", yaml_name)
	else:
		print("Use {:}".format(param_file))
		yaml_path = param_file

	ghn = GreenhouseNav(yaml_path)
