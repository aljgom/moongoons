#!/usr/bin/env python

import socket

# Open and connect TCP socket 
TCP_IP = '192.168.1.1'
TCP_PORT = 7777
BUFFER_SIZE = 9216
YUV_SIZE = 460800
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))


# Loop indefinitely
for image_number in range(0, 5):

	# Declare image buffer
	image = bytearray(YUV_SIZE)

	# Loop 50 times to get single entire image
	for packet_number in range(0, 50):
	
		# Receive 9216 bytes of data from server using recv_into
		packet = bytearray(BUFFER_SIZE)
		bytes_recvd = s.recv_into(packet)
		total_bytes = bytes_recvd
		for m in range(0, bytes_recvd):
			index = packet_number * BUFFER_SIZE + m
			image[index] = packet[m]
	
		# Make sure all 9216 bytes have been received
		while (total_bytes < BUFFER_SIZE):
			packet = bytearray(BUFFER_SIZE)
			bytes_recvd = s.recv_into(packet)
			for m in range(0, bytes_recvd):
				index = packet_number * BUFFER_SIZE + total_bytes + m
				image[index] = packet[m]
			total_bytes += bytes_recvd
		
	fd = open('image' + str(image_number) + '.yuv', 'wb')
	fd.write(image)
	
		
	##########################################################
	# Insert Image Processing Algorithm
	##########################################################

	# Output Angle Displacement back to main_motorboard.c
	#if (len(str(ang_dspl))!=3)
	#	length = len(str(ang_dspl))
	#	output = bytearray(3)
	#	for t in xrange(0,length-1)
			

	
#s.send(MESSAGE)

#s.close()
print "received image data"
