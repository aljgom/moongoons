Potentially useful snippets
===========================

YUV Imagestring to PIL Image
----------------------------
Source: http://stackoverflow.com/questions/4004710/how-to-decode-this-yuv-colorspace-string-and-save-it-as-an-image
im = Image.fromstring("YCbCr",(imageWidth,imageHeight),imageByteArray)

YUV to RGB
----------
Source:(http://stackoverflow.com/questions/7041172/pils-colour-space-conversion-ycbcr-rgb)

def yuv2rgb(im):
  """convert array-like yuv image to rgb colourspace

  a pure numpy implementation since the YCbCr mode in PIL is b0rked.
  TODO: port this stuff to a C extension, using lookup tables for speed
  """
  ## conflicting definitions exist depending on whether you use the full range
  ## of YCbCr or clamp out to the valid range.  see here
  ## http://www.equasys.de/colorconversion.html
  ## http://www.fourcc.org/fccyvrgb.php
  from numpy import dot, ndarray, array
  if not im.dtype == 'uint8':
    raise ImageUtilsError('yuv2rgb only implemented for uint8 arrays')

  ## better clip input to the valid range just to be on the safe side
  yuv = ndarray(im.shape)  ## float64
  yuv[:,:, 0] = im[:,:, 0].clip(16, 235).astype(yuv.dtype) - 16
  yuv[:,:,1:] = im[:,:,1:].clip(16, 240).astype(yuv.dtype) - 128

  ## ITU-R BT.601 version (SDTV)
  A = array([[1.,                 0.,  0.701            ],
             [1., -0.886*0.114/0.587, -0.701*0.299/0.587],
             [1.,  0.886,                             0.]])
  A[:,0]  *= 255./219.
  A[:,1:] *= 255./112.

  ## ITU-R BT.709 version (HDTV)
#  A = array([[1.164,     0.,  1.793],
#             [1.164, -0.213, -0.533],
#             [1.164,  2.112,     0.]])

  rgb = dot(yuv, A.T)
  return rgb.clip(0, 255).astype('uint8')
