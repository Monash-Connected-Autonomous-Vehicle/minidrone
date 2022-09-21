import numpy as np
import cv2
import math

# read input
img = cv2.imread("lane_image.jpeg")
image = img.astype("float64")
image = cv2.resize(image, (400,700))
pts = np.array([[185,540], [235,540], [185, 560], [235,560]], dtype = np.float32)
# specify input coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,

input_pts = np.array([[185,520], [238,520], [185, 580], [235,580]], dtype = np.float32)

# get top and left dimensions and set to output dimensions of red rectangle

matrix = cv2.getPerspectiveTransform(np.float32(pts), np.float32(input_pts))

# do perspective transformation setting area outside input to black
# Note that output size is the same as the input image size
imgOutput = cv2.warpPerspective(image, matrix, image.shape[:2][::-1])

cv2.imwrite("image_output.jpg", imgOutput)
