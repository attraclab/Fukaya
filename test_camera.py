

import cv2
import numpy as np
import time

startTime = time.time()
video = cv2.VideoCapture("/dev/video4")
# video = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# video = cv2.VideoCapture()
# cameraNumber = 4
# video.open(cameraNumber + cv2.CAP_DSHOW)

frame_height = 2160	#2160	#1080
frame_width = 3840	#3840	#1920

focus = 25
contrast = 30 #128
exposure = 312 #50
saturation = 50 #128
brightness = 1 #70 #90
wb = 5000

print("==================== Before setting ==================")
print("Get frame width ", video.get(3))
print("Get frame height ", video.get(4))
print("Get frame autofocus ", video.get(cv2.CAP_PROP_AUTOFOCUS))
print("Get frame focus length ", video.get(cv2.CAP_PROP_FOCUS))
print("Get FPS ", video.get(cv2.CAP_PROP_FPS))
print("Get brightness ", video.get(cv2.CAP_PROP_BRIGHTNESS))
print("Get exposure ", video.get(cv2.CAP_PROP_EXPOSURE))
print("Get auto wb ", video.get(cv2.CAP_PROP_AUTO_WB))
print("Get wb temp ", video.get(cv2.CAP_PROP_WB_TEMPERATURE))
print("Get auto exposure ", video.get(cv2.CAP_PROP_AUTO_EXPOSURE))
print("Get contrast", video.get(cv2.CAP_PROP_CONTRAST))
print("Get saturation", video.get(cv2.CAP_PROP_SATURATION))
print("Get fourcc", video.get(cv2.CAP_PROP_FOURCC))


video.set(3, frame_width)
video.set(4, frame_height)
fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
video.set(cv2.CAP_PROP_FOURCC, fourcc)
video.set(cv2.CAP_PROP_AUTOFOCUS, 0)
video.set(cv2.CAP_PROP_FOCUS, focus)
video.set(cv2.CAP_PROP_FPS, 30)
video.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
video.set(cv2.CAP_PROP_EXPOSURE, exposure)
video.set(cv2.CAP_PROP_AUTO_WB, 1)
video.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
video.set(cv2.CAP_PROP_CONTRAST, contrast)
video.set(cv2.CAP_PROP_SATURATION, saturation)

## Focus 
# 25 = 50cm
print("********************* After setting *********************")
print("Get frame width ", video.get(3))
print("Get frame height ", video.get(4))
print("Get frame autofocus ", video.get(cv2.CAP_PROP_AUTOFOCUS))
print("Get frame focus length ", video.get(cv2.CAP_PROP_FOCUS))
print("Get FPS ", video.get(cv2.CAP_PROP_FPS))
print("Get brightness ", video.get(cv2.CAP_PROP_BRIGHTNESS))
print("Get exposure ", video.get(cv2.CAP_PROP_EXPOSURE))
print("Get auto wb ", video.get(cv2.CAP_PROP_AUTO_WB))
print("Get wb temp ", video.get(cv2.CAP_PROP_WB_TEMPERATURE))
print("Get auto exposure ", video.get(cv2.CAP_PROP_AUTO_EXPOSURE))
print("Get contrast", video.get(cv2.CAP_PROP_CONTRAST))
print("Get saturation", video.get(cv2.CAP_PROP_SATURATION))


def info():
	print("Get frame focus length ", video.get(cv2.CAP_PROP_FOCUS))
	print("Get contrast", video.get(cv2.CAP_PROP_CONTRAST))
	print("Get exposure ", video.get(cv2.CAP_PROP_EXPOSURE))
	print("Get saturation", video.get(cv2.CAP_PROP_SATURATION))
	print("Get brightness ", video.get(cv2.CAP_PROP_BRIGHTNESS))
	print("====================================================")

## Exposure seems to not be adjustable on Brio during display

while True:



	_, image = video.read()
	# image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
	# image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
	# print(image.shape)
	# image = cv2.resize(image, (1920,1080), interpolation=cv2.INTER_AREA)
	# cv2.imshow("testWebcam", image)
	cv2.imshow("testWebcam", cv2.resize(image, (1920,1080), interpolation=cv2.INTER_AREA))

	k = cv2.waitKey(1)

	if k == ord('a'):
		focus = focus + 1
		print("Increased Focus ", focus)
		video.set(cv2.CAP_PROP_FOCUS, focus)
		info()
	elif k == ord('z'):
		focus = focus - 1
		print("Decreased Focus ", focus)
		video.set(cv2.CAP_PROP_FOCUS, focus)
		info()
	elif k == ord('s'):
		contrast += 1
		print("Increased Contrast ", contrast)
		video.set(cv2.CAP_PROP_CONTRAST, contrast)
		info()
	elif k == ord('x'):
		contrast -= 1
		print("Decreased Contrast ", contrast)
		video.set(cv2.CAP_PROP_CONTRAST, contrast)
		info()
	elif k == ord('d'):
		exposure += 1
		print("Increased exposure ", exposure)
		video.set(cv2.CAP_PROP_EXPOSURE, exposure)
		info()
	elif k == ord('c'):
		exposure -= 1
		print("Decreased exposure ", exposure)
		video.set(cv2.CAP_PROP_EXPOSURE, exposure)
		info()
	elif k == ord('f'):
		saturation += 1
		print("Increased saturation ", saturation)
		video.set(cv2.CAP_PROP_SATURATION, saturation)
		info()
	elif k == ord('v'):
		saturation -= 1
		print("Decreased saturation ", saturation)
		video.set(cv2.CAP_PROP_SATURATION, saturation)
		info()
	elif k == ord('g'):
		brightness += 1
		print("Increased brightness ", brightness)
		video.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
		info()
	elif k == ord('b'):
		brightness -= 1
		print("Decreased brightness ", brightness)
		video.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
		info()
	elif k == ord('k'):
		name = str(time.time()) + ".jpg"
		cv2.imwrite(name, image, [cv2.IMWRITE_JPEG_QUALITY, 100])
	elif k == 27:
		break
	elif k == -1:
		continue
	else:
		print("pressed ", k)
	
video.release()
cv2.destroyAllWindows()
