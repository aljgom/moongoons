# @name               networked_huelock.py
# @desc               server w/ huelock. upon receiving a udp packet with an
#                     image, process the image
# @contributors       Sam Liu <sam@ambushnetworks.com>
# @copyright          All rights reserved

# Notes
# -----
#
# Requires opencv 2.2+
# If you're unable to import cv (but rather, opencv),
# you have an old version of opencv.

import cv
import os
import sys


class Huelock():

    def __init__(self, color_min=(130, 220, 95), color_max=(180, 255, 255)):
        # Default colors work to detect red-cup red!
        self.color_min = cv.Scalar(color_min[0], color_min[1], color_min[2])
        self.color_max = cv.Scalar(color_max[0], color_max[1], color_max[2])

        self.posX = 0
        self.posY = 0

    # Returns x, y of object or None if the threshold was not met
    def detect_hue(self, image_filename=None, image_data=None, threshold=0):
        if image_filename:
            # Get the image from disk
            frame = cv.LoadImage(image_filename)
            frameHSV = cv.CreateImage(cv.GetSize(frame), 8, 3)
            cv.CvtColor(frame, frameHSV, cv.CV_RGB2HSV)

        elif image_data:
            # Raw image from feed
            cv.Smooth(image_data, image_data, cv.CV_BLUR, 3)
            hsv = cv.CreateImage(cv.GetSize(image_data), 8, 3)
            cv.CvtColor(image_data, hsv, cv.CV_BGR2HSV)
            frameHSV = hsv

        else:
            print "ERROR: No image file or data specified!"
            return None

        # Generate new image object with only the detected color in white
        frame_threshed = cv.CreateImage(cv.GetSize(image_data), 8, 1)
        cv.InRangeS(frameHSV, self.color_min, self.color_max, frame_threshed)

        mat = cv.GetMat(frame_threshed)

        #Calculating the moments
        moments = cv.Moments(mat, 0)
        area = cv.GetCentralMoment(moments, 0, 0)
        moment10 = cv.GetSpatialMoment(moments, 1, 0)
        moment01 = cv.GetSpatialMoment(moments, 0,1)

        if (int(area) > int(threshold)):
            #Calculating the coordinate postition of the centroid
            self.posX = int(moment10 / area)
            self.posY = int(moment01 / area)
            print 'x: ' + str(self.posX) + ' y: ' + str(self.posY) + ' area: ' + str(area)
            return (self.posX, self.posY)
        else:
            return None


def cli():
    # Regardless, instantiate an object detector
    detector = Huelock()

    if len(sys.argv) <= 2:
        print "Usage: ./" + str(sys.argv[0]) + "[server port] [threshold]"
    else:
        # Live camera display
        if sys.argv[1]=="test":

            if len(sys.argv) > 2:
                threshold = sys.argv[2]
            else:
                threshold = 0

            # Create a new window and get the camera data
            cv.NamedWindow("Camera", 1)
            font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, 0, 2, 8)

            # For each frame,
            while True:
                # Blocking wait for a frame packet
                frame = 
                width = 640 #TODO FIX THIS

                # Detect the color center
                center = detector.detect_hue(image_data=frame, threshold=threshold)

                if center:
                    quadrocopter_value = center[0] / float(width)
                    quadrocopter_value = quadrocopter_value * 100 - 50

                    # If a center was found, print it and show a circle
                    cv.PutText(frame, str(center[0])+","+str(center[1]), (30,30), font, (55,25,255))
                    cv.PutText(frame,"Value is " + str(quadrocopter_value), (30,90), font, (55,25,255))
                    cv.Circle(frame, center, 10, cv.Scalar(0, 255, 255))
                else:
                    # Otherwise, indicate that the threshold wasn't met
                    cv.PutText(frame,"Not enough red detected...", (30,30), font, (55,25,255))

                cv.ShowImage('Camera', frame)

                # Escape kills the program
                if cv.WaitKey(10) == 27:
                    break

if __name__ == '__main__':
    # Call command line interface
    cli()
