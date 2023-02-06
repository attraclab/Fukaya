import socket
import pickle


PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.setblocking(0)

while True:

	try:
		data, addr = sock.recvfrom(1024)
		# print("data len", len(data))
		parse_data = pickle.loads(data)
	except socket.error:
		pass
	else:
		print(parse_data)