
import socket
import pickle
import time
import os, signal

HEALTH_PORT = 7777
health_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
health_sock.bind(("0.0.0.0", HEALTH_PORT))
health_sock.setblocking(0)

healthy_stamp = time.time()
kill_stamp = time.time()
last_log_stamp = time.time()

while True:

	try:
		data, addr = health_sock.recvfrom(1024)
		# print("data len", len(data))
		parse_data = pickle.loads(data)
		healthy_stamp = time.time()
	except socket.error:
		pass
	
	healthy_period = time.time() - healthy_stamp
	last_kill_period = time.time() - kill_stamp


	if (healthy_period > 4.0) and (last_kill_period > 5.0):

		for line in os.popen('ps ax | grep "gopro_capture_udp" | grep -v grep'):
			fields = line.split()
			pid = fields[0]
			## kill the process 
			os.kill(int(pid), signal.SIGKILL)
			print("Kill gopro_capture_udp process {:d}".format(int(pid)))


		kill_stamp = time.time()

	if (time.time() - last_log_stamp) > 0.5:
		print("healthy_T: {:.2f} kill_T: {:.2f}".format(healthy_period, last_kill_period))
		last_log_stamp = time.time()


	time.sleep(0.01)