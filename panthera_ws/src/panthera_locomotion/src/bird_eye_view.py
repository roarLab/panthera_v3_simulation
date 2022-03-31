import cv2
import numpy as np
import matplotlib.pyplot as plt
import datetime
# IMAGE_H = 1280
# IMAGE_W = 1280
# a = datetime.datetime.now()
# src = np.float32([[0, IMAGE_H], [1207, IMAGE_H], [0, 0], [IMAGE_W, 0]])
# dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
# M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
# Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
# b = datetime.datetime.now()
# c = b-a

# print(c)
# img = cv2.imread('test1.jpeg') # Read the test img
# img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
# warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
# plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
# plt.show()
img = cv2.imread("test1.jpg",1)
img_org = cv2.imread("test1.jpg",1)

img_height,img_width = img.shape[:2]
print(img_height,img_width)
src = np.float32([[0, img_height], [508, img_height], [0, 0], [img_width, 0]])
dst = np.float32([[100, img_height], [229, img_height], [0, 0], [img_width, 0]])
M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
img = img[200:(200+img_height), 0:img_width] # Apply np slicing for ROI crop
warped_img = cv2.warpPerspective(img, M, (img_width, img_height)) # Image warping
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
plt.show()