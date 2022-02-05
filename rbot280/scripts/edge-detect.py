import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv.Canny(image, lower, upper)
	# return the edged image
	return edged

# Read images
img1 = cv.imread('AerialMIT.tif',0)
img1 = cv.GaussianBlur(img1, (3,3), 0)
img = cv.imread('MIT_SP.tif',0)
img = cv.GaussianBlur(img, (3,3), 0)
bird = cv.imread('birdseye.png')
bird = cv.GaussianBlur(bird, (3,3), 0)

# Find edges
edges = auto_canny(img)
edges1 = auto_canny(img1)
bird_edge = auto_canny(bird)

# Invert colors
bird_invert = cv.bitwise_not(bird_edge)
mit_invert = cv.bitwise_not(edges1)



# Plot
# resize images to fit on screen but keep originals as is
edge1 = cv.resize(bird_edge, (960,540))
invert1 = cv.resize(bird_invert, (960,540))
cv.imshow('gazebo', np.hstack([edge1,invert1]))
edge2 = cv.resize(edges1, (960,540))
invert2 = cv.resize(mit_invert, (960,540))
cv.imshow('mit', np.hstack([edge2,invert2]))
cv.waitKey(0)

# Save image
cv.imwrite('gazebo_map.png', bird_invert)
cv.imwrite('aerialMIT_map.tif', mit_invert)
