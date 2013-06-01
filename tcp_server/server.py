#!/usr/bin/env python

import socket
import time
from colortracker import ColorTracker

# Open and connect TCP socket
TCP_IP = '192.168.1.1'
TCP_PORT = 7777
BUFFER_SIZE = 9216
YUV_SIZE = 460800

# Instantiate socket connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# Declare buffers
image = bytearray(YUV_SIZE)
packet = bytearray(BUFFER_SIZE)

colortracker = ColorTracker()

# Loop indefinitely
for image_number in range(0, 5):

    START = int(round(time.time() * 1000))

    # 50 packets for 1 image
    for packet_number in range(0, 50):

        # Receive 9216 bytes of data from server
        bytes_recvd = s.recv_into(packet)
        total_bytes = bytes_recvd

        # TODO: optimize using list slicing
        for m in range(0, bytes_recvd):
            index = packet_number * BUFFER_SIZE + m
            image[index] = packet[m]

        # Make sure all 9216 bytes have been received
        while (total_bytes < BUFFER_SIZE):
            packet = bytearray(BUFFER_SIZE - total_bytes)
            bytes_recvd = s.recv_into(packet)

            # TODO: optimize this, there's definitely a faster way to do this
            # Copy image data from packet into the image buffer
            for m in range(0, bytes_recvd):
                index = packet_number * BUFFER_SIZE + total_bytes + m
                image[index] = packet[m]

            total_bytes += bytes_recvd

    position = colortracker.get_position(image, threshold=30000)

    if position:
        print "Red found, position is " + str(position) + "!"
    else:
        print "No red in this frame..."
    #print "image %d received" % (image_number)


    print "time is %d" % (int(round(time.time() * 1000))-START)

print "Execution complete."
