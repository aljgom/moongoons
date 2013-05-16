#!/usr/bin/env python
# -*- coding: utf-8 -*-

# hsv.py
# Returns HSV value of the pixel under the cursor in a video stream

import cv
import time

# Coordinates to examine
x_co = 0
y_co = 0

# When mouseover, change x_co and y_co to the current coordinates
def on_mouse(event,x,y,flag,param):
    global x_co
    global y_co

    if(event==cv.CV_EVENT_MOUSEMOVE):
        x_co=x
        y_co=y

# Create a new named window
cv.NamedWindow("camera", 1)

# Get camera stream
capture = cv.CaptureFromCAM(0)

# Use a font for putting text
font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, 0, 2, 8)

# Infinite loop
while True:
    # For each frame
    src = cv.QueryFrame(capture)

    # Blur it a little
    cv.Smooth(src, src, cv.CV_BLUR, 3)

    # Create hsv version of the image from RGB
    hsv = cv.CreateImage(cv.GetSize(src), 8, 3)
    thr = cv.CreateImage(cv.GetSize(src), 8, 1)
    cv.CvtColor(src, hsv, cv.CV_BGR2HSV)

    # Update mouse position
    cv.SetMouseCallback("camera",on_mouse, 0);

    # Get HSV values
    s = cv.Get2D(hsv,y_co,x_co)

    # Print HSV Values to command line
    print "H:",s[0],"      S:",s[1],"       V:",s[2]

    # Show HSV values on the namedwindow
    cv.PutText(src,str(s[0])+","+str(s[1])+","+str(s[2]), (x_co,y_co),font, (55,25,255))

    # Show the image itself in the named window
    cv.ShowImage("camera", src)

    # Escape kills the program
    if cv.WaitKey(10) == 27:
        break
