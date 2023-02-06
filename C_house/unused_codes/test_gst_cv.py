
import cv2
import time

FHD = (1920,1080)
HD = (1280,720)
HD_FLIP = (720,1280)
frame_width = 3840
frame_height = 2160

## This is CSI camera on Jetson only
#gst_str = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=(int)3840,height=(int)2160 !  nvvidconv ! video/x-raw,width=(int)3840,height=(int)2160,format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink'
gst_str = 'v4l2src device=/dev/video0 io-mode=2 ! image/jpeg,width=3840,height=2160 ! jpegdec ! videoconvert ! appsink'
video = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
contrast = 0 #128
# exposure = 312 #50
saturation = 45 #128
brightness = 40 #70 #90
wb = 5000
# video.set(cv2.CAP_PROP_FOCUS, focus)
# video.set(cv2.CAP_PROP_EXPOSURE, exposure)
video.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
video.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
video.set(cv2.CAP_PROP_CONTRAST, contrast)
video.set(cv2.CAP_PROP_SATURATION, saturation)


if not video.isOpened():
	raise SystemExit('ERROR: failed to open camera!')

alpha = 0
beta = 200
last_save_stamp = time.time()
while True:

	_, image = video.read()
	# print(image.shape)
	# image = cv2.resize(image, HD, interpolation=cv2.INTER_AREA)
	## using normalize can help adjusting brightness and contrast by tweaking alpha and bete values
	## https://stackoverflow.com/questions/61016954/controlling-contrast-and-brightness-of-video-stream-in-opencv-and-python
	# image_norm = cv2.normalize(image, None, alpha=alpha,beta=beta, norm_type=cv2.NORM_MINMAX)

	# cv2.imshow("frame", image)

	if (time.time() - last_save_stamp) > 2.0:
		image_name = "{}.jpg".format(time.time())	
		print("Saved {:}".format(image_name))	
		cv2.imwrite(image_name, image)
		last_save_stamp = time.time()


	cv2.waitKey(1)



video.release()
cv2.destroyAllWindows()