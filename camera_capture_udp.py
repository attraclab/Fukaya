

import cv2
import numpy as np
import time
import socket
import pickle
import os
from datetime import datetime
import threading

global cart_mode, shutter, shelf_number, shelf_number_nav, ch7

PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.setblocking(0)

startTime = time.time()


frame_height = 2160	#2160	#1080
frame_width = 3840	#3840	#1920

# out = cv2.VideoWriter('out.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 10, (frame_width,frame_height))

FHD = (1920,1080)
HD = (1280,720)
HD_FLIP = (720,1280)

cart_mode = 1
ch7 = 1024
prev_ch7 = ch7


shelf_number = 1
shelf_number_nav = 1
prev_shelf_number = shelf_number
prev_shelf_number_nav = shelf_number_nav

######################
### File directory ###
######################


def udp_recv_worker(lock):
	global cart_mode, shutter, shelf_number, shelf_number_nav, ch7

	print("Start udp receiver")
	while True:


		try:
			data, addr = sock.recvfrom(1024)
			# print("data len", len(data))
			parse_data = pickle.loads(data)
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


def camera_worker(lock):

	global cart_mode, shutter, shelf_number, shelf_number_nav, ch7

	project_dir = os.getcwd()
	images_logger_path = os.path.join(project_dir, "images_logger")
	if not os.path.exists(images_logger_path):
		os.mkdir(images_logger_path)


	gst_str = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=(int)3840,height=(int)2160 !  nvvidconv ! video/x-raw,width=(int)3840,height=(int)2160,format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink'
	video = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

	if not video.isOpened():
		raise SystemExit('ERROR: failed to open camera!')

	print("Start camera_capture_udp loop")
	last_capture_stamp = time.time()
	init_sub_folder = True
	capture_time = 3.0
	image_counter = 1
	while True:
		start_time = time.time()


		_, image = video.read()
		# image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
		# image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
		# print(image.shape)
		# image = cv2.resize(image, (1920,1080), interpolation=cv2.INTER_AREA)
		# cv2.imshow("testWebcam", image)
		# cv2.imshow("testWebcam", cv2.resize(image, HD_FLIP, interpolation=cv2.INTER_AREA))
		# cv2.imshow("testWebcam", cv2.resize(image, HD, interpolation=cv2.INTER_AREA))
		# out.write(image)


		now = datetime.now()
		# print(image.shape)

		## must be in auto-mode, enable navigation, and got shutter flag
		if (cart_mode == 2) and (ch7 > 1500) and shutter:
			
			if init_sub_folder:
				
				folder_name = now.strftime("%Y%m%d_%H%M%S")
				sub_store_path = os.path.join(images_logger_path, folder_name)
				print("Auto mode recording, data will be stored at {:}".format(sub_store_path))
				if not os.path.exists(sub_store_path):
					os.mkdir(sub_store_path)
				init_sub_folder = False

			if ((time.time() - last_capture_stamp) >= capture_time):
				image_name = "{:d}_{:d}_{:d}.jpg".format(shelf_number, shelf_number_nav, image_counter)
				image_store_path = os.path.join(sub_store_path, image_name)
				print("saved {:}".format(image_store_path))
				cv2.imwrite(image_store_path, image)
				last_capture_stamp = time.time()
				image_counter += 1

		if False:
			print("mode: {:d} ch7: {:d} shutter: {}".format(\
				cart_mode, ch7, shutter))


		# print("period", time.time() - start_time)
		cv2.waitKey(1)

	video.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":

	lock = threading.Lock()

	t1 = threading.Thread(target=camera_worker, args=(lock,))
	t2 = threading.Thread(target=udp_recv_worker, args=(lock,))

	t1.start()
	t2.start()