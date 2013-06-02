#!/usr/bin/env python

import socket
import subprocess
import os
import cv
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

# Color tracker
colortracker = ColorTracker()

# Camera display
cv.NamedWindow("Camera", 1)
font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, 0, 2, 8)
display_width = 640
display_height = 480

# Loop indefinitely
image_number = 0
while True:

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

    # Write image to file
    image_file = open('tmp.yuv', 'wb+')
    image_file.write(image)
    image_file.close()

    # FFMPEG in silent mode to convert the image
    ffmpeg_command = ['ffmpeg', '-s', '640x480', '-i', 'tmp.yuv', 'tmp.jpg', '-loglevel', 'panic']

    subprocess.call(ffmpeg_command, stdout=open(os.devnull, 'wb'))

    # Now let opencv decompress your image
    cvimage = cv.LoadImage("tmp.jpg")

    # Put on computer display
    cv.ShowImage('Camera', cvimage)

    # Escape kills the program
    # NOTE: waitkey is crucial for refreshing the gui
    if cv.WaitKey(10) == 27:
        break

    position = colortracker.get_position(cvimage, threshold=10000)

    if position:
        print "Red found, position is " + str(position) + "!"
        print "image %d received" % (image_number)
        print "time is %d \n\n" % (int(round(time.time() * 1000))-START)

    image_number += 1


print "Execution complete."
