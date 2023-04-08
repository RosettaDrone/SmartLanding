"""
Developed by opensource developers for opensource developers.
If you are not going to contribute back, don't use this Software.
We are not competing, but collaborating.
The world is big and life is short.

https://github.com/RosettaDrone/rosettadrone

Author: Christopher Pereira (rosetta@imatronix.com)
"""

import socket
import struct
import numpy as np
import cv2
import time
import sys

def printNoLF(str):
	if sys.version_info.major == 3:
		print(str, end='')
	else:
		sys.stdout.write(str)
		sys.stdout.flush()

class RosettaCamera():
	def __init__(self):
		self.debug = False

	def connect(self):
		server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		server_socket.bind(('localhost', 6000))
		server_socket.listen(1)

		printNoLF("Waiting for video connection...")
		self.client_socket, address = server_socket.accept()
		print(" ok")

		self.last_time = time.time()

	def reconnect(self):
		print("Client disconnected. Reconnecting...")
		self.client_socket.close()
		self.client_socket, address = server_socket.accept()

	def getImage(self):
		consecutive_errors = 0
		while True:
			data = self.client_socket.recv(4)
			if not data:
				raw_tcp_reconnect()
				continue

			(w,) = struct.unpack('i', data)

			data = self.client_socket.recv(4)
			if not data:
				raw_tcp_reconnect()
				continue

			(h,) = struct.unpack('i', data)

			data = self.client_socket.recv(8)
			if not data:
				raw_tcp_reconnect()
				continue

			ts = struct.unpack('Q', data)[0]

			data = self.client_socket.recv(4)
			if not data:
				raw_tcp_reconnect()
				continue

			yaw = struct.unpack('f', data)[0]

			data = self.client_socket.recv(4)
			if not data:
				raw_tcp_reconnect()
				continue

			(size,) = struct.unpack('i', data)

			buffer = bytearray(size)
			bytes_received = 0
			while bytes_received < size:
				data = self.client_socket.recv(size - bytes_received)
				if not data:
					raw_tcp_reconnect()
					break
				buffer[bytes_received:bytes_received+len(data)] = data
				bytes_received += len(data)

			if bytes_received != size:
				consecutive_errors += 1
				if consecutive_errors > 100:
					time.sleep(1)
			else:
				consecutive_errors = 0

				if self.debug:
					now = time.time()
					printNoLF("Got image")

					if self.last_time:
						printNoLF(", ts: " + str(ts) + ", ms: " + str(now - self.last_time))
					self.last_time = now

					print(", bytes: ", bytes_received)

				yuv_mat = np.frombuffer(buffer, dtype=np.uint8)
				yuv_mat = yuv_mat.reshape((h + h // 2, w))

				rgb_mat = cv2.cvtColor(yuv_mat, cv2.COLOR_YUV2BGR_I420)

				return rgb_mat, ts, yaw
