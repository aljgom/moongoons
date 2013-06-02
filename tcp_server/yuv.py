# Author : sherin.s@gmail.com (Sherin Sasidhan)
# Date   : 05-Oct-2010

import Image
import sys
from struct import *
import array

def clip(x):
    if x > 255:
        x = 255
    elif(x < 0):
        x = 0
    return x

if len(sys.argv) != 4:
        print "***** Usage syntax Error!!!! *****\n"
        print "Usage:"
        print "python display-image.py [yuv file] [width] [height]"
        sys.exit(1) # exit
else:
        pass

image_name = sys.argv[1]
width = int(sys.argv[2])
height = int(sys.argv[3])
imagedata = array.array('B')

fd = open(image_name, "r+b")
imagedata = fd.read()

image_out = Image.new("RGB", (width, height))

print "width=", width, "height=", height

Image.fromstring("YUV420", (width,height), imagedata)
image_out.save("out.jpg")
image_out.show()
