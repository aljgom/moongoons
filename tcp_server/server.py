#!/usr/bin/env python

import cv
import cProfile
import errno
import os
import socket
import subprocess
import sys
import time

from colortracker import ColorTracker
from datetime import datetime

# Util for file paths
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise


class PositionServer():

    def __init__(self, ip='192.168.1.1', port=7777, buffer_size=9216, yuv_size=92160, display=False, threshold=10000, img_height=96, img_width=640, flip=True, store_images=True, correction_factor=0):
        # Initialize the environment
        self.tcp_ip = ip
        self.tcp_port = port
        self.buffer_size = buffer_size
        self.yuv_size = yuv_size

        # Allocate memory for images and packets
        self.image = bytearray(self.yuv_size)
        self.packet = bytearray(self.yuv_size)

        # Instantiate color tracker
        self.colortracker = ColorTracker()

        # Flipping the value signs
        self.flip = flip

        # Store intermediate images used to get position
        self.store_images = store_images

        # Display initialization
        self.display = display

        # Colortracker threshold
        self.threshold = threshold

        # Alignment correction factor
        self.correction_factor = correction_factor

        # For this specific run, the path to store images in
        self.run_img_path = 'images/' + str(datetime.now()) + '/'

        # Create relevant directories
        make_sure_path_exists(self.run_img_path)

        # Camera display
        if self.display:
            cv.NamedWindow("Camera", 1)
            self.cvfont = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, 0, 2, 8)
            self.width = img_width
            self.height = img_height

    def connect(self):
        # Create socket connection
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.tcp_ip, self.tcp_port))

    def main_loop(self):
        image_number = 0
        while True:
            start_time = int(round(time.time() * 1000))

            # Receive yuv_size bytes of data from server
            bytes_recvd = self.socket.recv_into(self.packet)
            total_bytes = bytes_recvd

            # TODO: optimize using list slicing
            for m in range(0, bytes_recvd):
                index = m
                self.image[index] = self.packet[m]

            # Make sure all yuv_size bytes have been received
            while (total_bytes < self.yuv_size):
                packet = bytearray(self.yuv_size - total_bytes)
                bytes_recvd = self.socket.recv_into(packet)

                # TODO: optimize this, there's definitely a faster way to do this
                # Copy image data from packet into the image buffer
                for m in range(0, bytes_recvd):
                    index = total_bytes + m
                    self.image[index] = packet[m]

                total_bytes += bytes_recvd

            # Write image to file
            image_name = str(datetime.now())
            yuv_filename = self.run_img_path + image_name + '.yuv'
            jpg_filename = self.run_img_path + image_name + '.jpg'

            # If we don't want to store the images, just write into tmp.yuv
            # or tmp.jpg and overwrite each time
            if not self.store_images:
                yuv_filename = 'images/tmp.yuv'
                jpg_filename = 'images/tmp.jpg'

            image_file = open(yuv_filename, 'wb+')
            image_file.write(self.image)
            image_file.close()

            # FFMPEG in silent mode to convert the image
            ffmpeg_command = ['ffmpeg', '-s', str(self.width) + "x" + str(self.height), '-i', yuv_filename, jpg_filename, '-loglevel', 'panic']

            subprocess.call(ffmpeg_command, stdout=open(os.devnull, 'wb'))

            # Now let opencv decompress your image
            cvimage = cv.LoadImage(jpg_filename)

            if self.display:
                # Put on computer display
                cv.ShowImage('Camera', cvimage)

                # Esc key kills the program, waitkey refreshes the display
                if cv.WaitKey(10) == 27:
                    break

            position = self.colortracker.get_position(cvimage, threshold=self.threshold)

            print "time is %d \n\n" % (int(round(time.time() * 1000)) - start_time)
            if position:
                # Some cameras flip the image. Unflip the value for more
                # conventiona signage.
                if self.flip:
                    position *= -1

                position = position + self.correction_factor

                # Position found
                print "Red found, position is " + str(position) + "!"
                print "image %d received" % (image_number)

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

    elif len(sys.argv) >= 3:
        # Threshold, show image stream
        threshold = sys.argv[1]
        display = sys.argv[2] == "--showdisplay"

        # For y displacement correction
        correction_factor = 0
        if len(sys.argv) == 4:
            correction_factor = sys.argv[3]

        position_server = PositionServer(threshold=threshold, display=display, correction_factor=correction_factor)



    position_server.connect()
    print "Connected!"

    print "Starting image streaming..."
    position_server.main_loop()

if __name__ == "__main__":
    cli()
