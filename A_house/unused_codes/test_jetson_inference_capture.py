import cv2
import jetson.utils
import numpy
import time


camera_device="csi://0"
frame_width=3840
frame_height=2160

#display = jetson.utils.videoOutput("display://0")
#camera = jetson.utils.videoInput("csi://0")

#display = jetson.utils.glDisplay()
camera = jetson.utils.gstCamera(frame_width, frame_height, camera_device)
camera.Open()

last_stamp_save = time.time()

#while not display.IsOpen():
#	continue

while True:
	image = camera.Capture()
	image, width, height = camera.Capture(format="rgb8")
	#display.RenderOnce(image, width, height)
	#print(type(image))
	if (time.time() - last_stamp_save) > 2.0:
		np_image = jetson.utils.cudaToNumpy(image, frame_width, frame_height, 4)
		np_image = cv2.cvtColor(np_image, cv2.COLOR_RGBA2RGB).astype('uint8')
		np_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)
		image_name = "{}.jpg".format(time.time())		
		cv2.imwrite(image_name, np_image)
		last_stamp_save = time.time()
camera.close()
