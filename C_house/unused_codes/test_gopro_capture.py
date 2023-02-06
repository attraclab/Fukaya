
import cv2
import requests
from pathlib import Path
import time
import subprocess

FHD = (1920,1080)
HD = (1280,720)
HD_FLIP = (720,1280)
frame_width = 3840
frame_height = 2160

# gopro_ip = "172.29.165.51"
gopro_ip = "172.25.120.51"
# gopro_ip = "172.29.181.51"
# gopro_ip = "172.20.155.51"

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

def download_last_photo(image_file_path):
	res = requests.get('http://{}:8080/gopro/media/list'.format(gopro_ip))
	print(res.status_code)
	filename_list = []
	path_list = []
	results = []
	number_of_folders = len(res.json()['media'])
	if number_of_folders == 0:
		number_of_files = 0
		print("No files...")
	# elif len(res.json()['media']) == 1:
	else:
		number_of_files = len(res.json()['media'][number_of_folders-1]['fs'])
		filename = res.json()['media'][number_of_folders-1]['fs'][number_of_files-1]['n']
		folder_name = res.json()['media'][number_of_folders-1]['d']
		R = requests.get('http://{}:8080/videos/DCIM/{}/{}'.format(gopro_ip, folder_name, filename), allow_redirects=True)
		image_file = Path(image_file_path)
		image_file.write_bytes(R.content)
	print("number_of_folders", number_of_folders)


enable_stat_code = enable_wired_control()
if enable_stat_code == 500:
	disable_wired_control()
	enable_stat_code = enable_wired_control()

## Must be 200 for both
print("enable_status ", enable_stat_code)

mode_stat_code = photo_mode()
print("mode_status ", mode_stat_code)


last_save_stamp = time.time()
while True:

	if (time.time() - last_save_stamp) > 2.0:
		image_name = str(time.time()) + ".jpg"
		capture_stat_code = take_photo()
		print("capture_status", capture_stat_code)
		time.sleep(1)
		download_last_photo(image_name)
		# cmd = "rsync -avP {} ginzafarm@192.168.1.8:/home/ginzafarm/test_upload/".format(image_name)
		# subprocess.call(cmd, shell=True)
		last_save_stamp = time.time()

