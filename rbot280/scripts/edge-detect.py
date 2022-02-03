import numpy
import cv2 as cv
from matplotlib import pyplot as plt

img1 = cv.imread('AerialMIT.tif',0)
img = cv.imread('MIT_SP.tif',0)
edges = cv.Canny(img,50,200)
edges1 = cv.Canny(img1,50,200)

plt.subplot(221),plt.imshow(img,cmap='gray')
plt.title('orig'), plt.xticks([]), plt.yticks([])
plt.subplot(222), plt.imshow(edges,cmap='gray')
plt.title('edges'), plt.xticks([]), plt.yticks([])
plt.subplot(223),plt.imshow(img1,cmap='gray')
plt.title('orig1'), plt.xticks([]), plt.yticks([])
plt.subplot(224), plt.imshow(edges1,cmap='gray')
plt.title('edges1'), plt.xticks([]), plt.yticks([])

plt.show()
