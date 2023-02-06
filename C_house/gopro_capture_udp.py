

import cv2
import numpy as np
import time
import socket
import pickle
import os
from datetime import datetime
import threading
import requests
from pathlib import Path
from camera_params import frames_require_list, capture_time_list, gopro_ip, server_user, server_ip, server_dest
import subprocess
from netifaces import interfaces, ifaddresses, AF_INET
import sys
import signal

global cart_mode, shutter, shelf_number, shelf_number_nav, ch7
global last_error_log_stamp
global image_store_path_glob, image_store_path_list
global t2, t3

PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.setblocking(0)

HEALTH_PORT = 7777
IP = "127.0.0.1"
health_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

startTime = time.time()


FHD = (1920,1080)
HD = (1280,720)
HD_FLIP = (720,1280)

cart_mode = 1
ch7 = 1024
prev_ch7 = ch7


shelf_number = 1
shelf_number_nav = 2
prev_shelf_number = shelf_number
prev_shelf_number_nav = shelf_number_nav
last_error_log_stamp = time.time()

# gopro_ip = "172.29.165.51"
#gopro_ip = "172.25.120.51"

#################
### GoPro API ###
#################

def enable_wired_control():
	x = requests.get('http://{}:8080/gopro/camera/control/wired_usb?p=1'.format(gopro_ip), timeout=3)
	return x.status_code

def disable_wired_control():
	x = requests.get('http://{}:8080/gopro/camera/control/wired_usb?p=0'.format(gopro_ip))
	return x.status_code

def photo_mode():
	x = requests.get('http://{}:8080/gp/gpControl/command/mode?p=1'.format(gopro_ip))
	return x.status_code

def take_photo():
	x = requests.get('http://{}:8080/gopro/camera/shutter/start'.format(gopro_ip))
	return x.status_code

def download_last_photo(image_store_path):

	res = requests.get('http://{}:8080/gopro/media/list'.format(gopro_ip))
	# print(res.status_code)
	filename_list = []
	path_list = []
	results = []
	number_of_folders = len(res.json()['media'])
	try:
		if len(res.json()['media']) == 0:
			number_of_files = 0
			print("No files...")
		else:
			number_of_files = len(res.json()['media'][number_of_folders-1]['fs'])
			filename = res.json()['media'][number_of_folders-1]['fs'][number_of_files-1]['n']
			folder_name = res.json()['media'][number_of_folders-1]['d']
			R = requests.get('http://{}:8080/videos/DCIM/{}/{}'.format(gopro_ip, folder_name, filename), allow_redirects=True)
			image_file = Path(image_store_path)
			image_file.write_bytes(R.content)
	except Exception as e:
		print("Error on download_last_photo")
		print(e)
		pass

def print_error_log(text):
	global last_error_log_stamp

	if (time.time() - last_error_log_stamp) > 1.0:
		print(text)
		last_error_log_stamp = time.time()

def check_wire_connection():

	ip_list = []

	for ifaceName in interfaces():
		addresses = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP addr'}] )]
		ip_list.append(' '.join(addresses))

	found_gopro_wire_ip = any(ip.startswith('172.2') for ip in ip_list)

	return found_gopro_wire_ip

######################
### Thread workers ###
######################

def udp_recv_worker(lock):
	global cart_mode, shutter, shelf_number, shelf_number_nav, ch7
	last_recv_stamp_log = time.time()
	time.sleep(2)
	print("Start udp receiver")
	while True:


		try:
			data, addr = sock.recvfrom(1024)
			# print("data len", len(data))
			parse_data = pickle.loads(data)
			if ((time.time() - last_recv_stamp_log) > 1.0):
				print(parse_data)
				last_recv_stamp_log = time.time()
		except socket.error:
			pass
		else:
			# print(parse_data)
			with lock:
				cart_mode = parse_data['cart_mode']
				shutter = parse_data['shutter']
				shelf_number = parse_data['shelf_no']
				shelf_number_nav = parse_data['shelf_no_nav']
				ch7 = parse_data['ch7']

		time.sleep(0.02)



def camera_worker(lock):

	global cart_mode, shutter, shelf_number, shelf_number_nav, ch7
	global image_store_path_glob, image_store_path_list
	global t2, t3

	found_gopro_wire_ip =  check_wire_connection()
	if not found_gopro_wire_ip:
		print("GoPro wire IP not found")
		os.kill(os.getpid(), signal.SIGINT)
	else:
		t2.start()
		t3.start()


	project_dir = os.getcwd()
	images_logger_path = os.path.join(project_dir, "images_logger")
	if not os.path.exists(images_logger_path):
		os.mkdir(images_logger_path)

	enable_stat_code = enable_wired_control()
	if enable_stat_code == 500:
		disable_wired_control()
		enable_stat_code = enable_wired_control()

	## Must be 200 for both
	print("Wired control enable", enable_stat_code)

	mode_stat_code = photo_mode()
	print("Photo mode", mode_stat_code)

	_cart_mode = 1
	_shutter = False
	_shelf_number = 1
	_shelf_number_nav = 2
	_ch7 = 1024

	shutter = False

	print("Start camera_capture_udp loop")
	last_capture_stamp = time.time()
	init_sub_folder = True
	capture_time = 3.0
	image_counter = 1
	prev_shelf_number = _shelf_number
	prev_shelf_number_nav = _shelf_number_nav
	last_log_stamp = time.time()
	capture_criteria_general = False
	capture_criteria_1st_lane = False
	image_store_path_list = []
	_image_store_path_list = []

	while True:
		start_time = time.time()

		# packets = {'healthy' : True}
		# dump_packets = pickle.dumps(packets)
		# health_sock.sendto(dump_packets,(IP, HEALTH_PORT))

		with lock:
			_cart_mode = cart_mode
			_shutter = shutter
			_shelf_number = shelf_number
			_shelf_number_nav = shelf_number_nav
			_ch7 = ch7
			# _image_store_path_list = image_store_path_list


		capture_time = capture_time_list[_shelf_number-1]
		frame_require_1st = frames_require_list[_shelf_number-1]
		frame_require_2nd = frame_require_1st*2

		now = datetime.now()
		# print(image.shape)

		## Reset image_counter once changed lane or U-turn
		if prev_shelf_number != _shelf_number:
			if _shelf_number_nav == 2:
				image_counter = frame_require_1st + 1
			else:
				image_counter = 1



		## must be in auto-mode, enable navigation, and got shutter flag
		if (_cart_mode == 2) and (_ch7 > 1500) and _shutter:
			
			if init_sub_folder:
				
				folder_name = now.strftime("%Y%m%d_%H%M%S")
				sub_store_path = os.path.join(images_logger_path, folder_name)
				print("Auto mode recording, data will be stored at {:}".format(sub_store_path))
				if not os.path.exists(sub_store_path):
					os.mkdir(sub_store_path)
				init_sub_folder = False


			waiting_time_to_capture = (time.time() - last_capture_stamp)
			# print(waiting_time_to_capture)
			if (waiting_time_to_capture >= capture_time):

				capture_criteria_general = (((_shelf_number != 1) and (_shelf_number_nav == 1) and (image_counter <= frame_require_1st)) \
										 or ((_shelf_number != 1) and (_shelf_number_nav == 2) and (image_counter <= frame_require_2nd)))
				capture_criteria_1st_lane = (_shelf_number == 1) and (image_counter <= frame_require_1st)

				if capture_criteria_general or capture_criteria_1st_lane:
					image_name = "{:d}_{:d}.jpg".format(_shelf_number, image_counter)
					image_store_path = os.path.join(sub_store_path, image_name)
					print("capture_time: {:.2f}".format(capture_time))
					print("saved {:}".format(image_store_path))
					last_capture_stamp = time.time()

					capture_stat_code = take_photo()
					print("capture status", capture_stat_code)
					time.sleep(2)
					download_last_photo(image_store_path)

					## append the latest image path to list
					_image_store_path_list.append(image_store_path)
					# print("append", _image_store_path_list)

					image_counter += 1
				else:
					if (_shelf_number != 1):
						if (_shelf_number_nav == 1):
							error_text = "s_no: {:d} s_no_nav: {:d} image_counter is {:d} and it exceeds frame_require_1st as {:d}".format(\
								_shelf_number, _shelf_number_nav, image_counter, frame_require_1st)
						else:
							error_text = "s_no: {:d} s_no_nav: {:d} image_counter is {:d} and it exceeds frame_require_2nd as {:d}".format(\
								_shelf_number, _shelf_number_nav, image_counter, frame_require_2nd)
					else:
						error_text = "s_no: {:d} s_no_nav: {:d} image_counter is {:d} and it exceeds frame_require_1st as {:d}".format(\
								_shelf_number, _shelf_number_nav, image_counter, frame_require_1st)

					print_error_log(error_text)

		else:
			## add sleep to reduce CPU loads when in idle
			# packets = {'healthy' : True}
			# dump_packets = pickle.dumps(packets)
			# health_sock.sendto(dump_packets,(IP, HEALTH_PORT))
			time.sleep(0.02)

		## update image_store_path_list (global) with this new one
		with lock:
			image_store_path_list = _image_store_path_list

		if (time.time() - last_log_stamp) > 1.0:
			times_take = time.time() - start_time
			print("T: {:.2f} prev_s_no: {:d} s_no: {:d} s_no_nav: {:d} img_count: {:d} CCG: {} CC1L: {} FR1: {:d} FR2: {:d}".format(\
				times_take, prev_shelf_number, _shelf_number, _shelf_number_nav, image_counter, capture_criteria_general, capture_criteria_1st_lane, \
				frame_require_1st, frame_require_2nd))
			last_log_stamp = time.time()


		# time.sleep(0.001)
		prev_shelf_number = _shelf_number

		# print("period", time.time() - start_time)

def image_copy_worker(lock):

	global image_store_path_glob, image_store_path_list

	# server_user = "ginzafarm"
	# server_ip = "192.168.1.8"
	# server_dest = "/home/ginzafarm/test_upload/"

	_image_store_path_list = []
	image_store_path_list = []
	prev_image_store_path = None
	image_store_path_glob = None

	last_copy_log_stamp = time.time()
	print("image copy worker started")
	while True:

		## get global list
		with lock:
			_image_store_path_list = image_store_path_list

		if len(_image_store_path_list) > 0:
			print("_image_store_path_list", _image_store_path_list)
			image_path = _image_store_path_list.pop(0)
			print("file to be copied", image_path)

			cmd = "rsync -avP {} {}@{}:{}".format(\
					image_path, server_user, server_ip, server_dest)
			subprocess.call(cmd, shell=True)

			## update local list to global list
			with lock:
				image_store_path_list = _image_store_path_list

		# if (time.time() - last_copy_log_stamp) > 1.0:
		# 	print("in copy thread")
		# 	last_copy_log_stamp = time.time()

		## needs a bit of delay
		time.sleep(0.01)




if __name__ == "__main__":

	global t2, t3

	lock = threading.Lock()

	t1 = threading.Thread(target=camera_worker, args=(lock,))
	t2 = threading.Thread(target=udp_recv_worker, args=(lock,))
	t3 = threading.Thread(target=image_copy_worker, args=(lock,))

	t1.start()
	# t2.start()
	# t3.start()