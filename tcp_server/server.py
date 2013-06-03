#!/usr/bin/env python

import cv
import os
import socket
import subprocess
import sys
import time

from colortracker import ColorTracker


class PositionServer():

    def __init__(self, ip='192.168.1.1', port=7777, buffer_size=9216, yuv_size=460800, display=False, threshold=10000, debug=True):
        # Initialize the environment
        self.tcp_ip = ip
        self.tcp_port = port
        self.buffer_size = buffer_size
        self.yuv_size = yuv_size
	self.debug = debug

        # Allocate memory for images and packets
        self.image = bytearray(self.yuv_size)
        self.packet = bytearray(self.buffer_size)

        # Instantiate color tracker
        self.colortracker = ColorTracker()

        # Display initialization
        self.display = display

        # Colortracker threshold
        self.threshold = threshold

        # Camera display
        if self.display:
            cv.NamedWindow("Camera", 1)
            self.cvfont = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, 0, 2, 8)
            self.display_width = 640
            self.display_height = 480

    def connect(self):
        # Create socket connection
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.tcp_ip, self.tcp_port))

    def main_loop(self):
        image_number = 0
        while True:
            start_time = int(round(time.time() * 1000))

            # 50 packets for 1 image
            for packet_number in range(0, 50):

                # Receive 9216 bytes of data from server
                bytes_recvd = self.socket.recv_into(self.packet)
                total_bytes = bytes_recvd

                # TODO: optimize using list slicing
                for m in range(0, bytes_recvd):
                    index = packet_number * self.buffer_size + m
                    self.image[index] = self.packet[m]

                # Make sure all 9216 bytes have been received
                while (total_bytes < self.buffer_size):
                    packet = bytearray(self.buffer_size - total_bytes)
                    bytes_recvd = self.socket.recv_into(packet)

                    # TODO: optimize this, there's definitely a faster way to do this
                    # Copy image data from packet into the image buffer
                    for m in range(0, bytes_recvd):
                        index = packet_number * self.buffer_size + total_bytes + m
                        self.image[index] = packet[m]

                    total_bytes += bytes_recvd

            # Write image to file
            image_file = open('tmp.yuv', 'wb+')
            image_file.write(self.image)
            image_file.close()

            # FFMPEG in silent mode to convert the image
            ffmpeg_command = ['ffmpeg', '-s', '640x480', '-i', 'tmp.yuv', 'tmp.jpg', '-loglevel', 'panic']

            subprocess.call(ffmpeg_command, stdout=open(os.devnull, 'wb'))

            # Now let opencv decompress your image
            cvimage = cv.LoadImage("tmp.jpg")

            if self.display:
                # Put on computer display
                cv.ShowImage('Camera', cvimage)

                # Esc key kills the program, waitkey refreshes the display
                if cv.WaitKey(10) == 27:
                    break

            position = self.colortracker.get_position(cvimage, threshold=self.threshold)

	    if self.debug:
            	print "image %d received" % (image_number)
            	print "time is %d \n\n" % (int(round(time.time() * 1000)) - start_time)

            if position:
                # Position found
                print "Red found, position is " + str(position) + "!"

                # Assume the "value" is an angular displacement
                ang_dspl = str(int(position))

                # Send angular displacement back to the AR Drone
                if (len(ang_dspl) != 4):
                    ang_dspl = ang_dspl.zfill(4)
                    bytes_sent = self.socket.send(ang_dspl)
                else:
                    bytes_sent = self.socket.send(ang_dspl)
            else:
                # No position found, send "None"
                bytes_sent = self.socket.send(str(None))

            image_number += 1

def cli():
    # Command line interface

    if len(sys.argv) < 2:
        # No args: show usage info
        print "Usage: " + sys.argv[0] + " [threshold] [--showdisplay]"
        sys.exit()

    elif len(sys.argv) == 2:
        # Threshold, no image stream
        threshold = int(sys.argv[1])
        position_server = PositionServer(threshold=threshold)

    elif len(sys.argv) == 3:
        # Threshold, show image stream
        threshold = sys.argv[1]
        display = sys.argv[2] == "--showdisplay"

        position_server = PositionServer(threshold=threshold, display=display)


    position_server.connect()
    print "Connected!"

    print "Starting image streaming..."
    position_server.main_loop()

if __name__ == "__main__":
    cli()
