import os
import shutil
from custom_params import house

images_logger_path = "/home/nvidia/Fukaya/{}_house/images_logger".format(house)

os.chdir(images_logger_path)

dir_list = os.listdir(images_logger_path)
print(dir_list)

if len(dir_list) == 0:
	print("There is no file or folder here, not delete anything")

else:
	for elem in dir_list:
		if elem.startswith("20") or elem.endswith(".jpg"):
			path_to_delete = os.path.join(images_logger_path, elem)
			if os.path.isfile(path_to_delete):
				os.remove(path_to_delete)
			else:
				shutil.rmtree(path_to_delete)
			print("Delete {}".format(path_to_delete))

