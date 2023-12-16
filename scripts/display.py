from PIL import Image
import os, sys
import cv2
import numpy as np
from matplotlib import pyplot as plt



img1 = cv2.imread('/home/morita/ros2_ws/output/rectangle07.jpg')
img2 = cv2.imread('/home/morita/ros2_ws/output/rectangle09.jpg')


img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)
img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)


plt.subplot(121),plt.imshow(img2)
plt.title('Profile of blonde woman'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(img1)
plt.title('Profile of young woman'), plt.xticks([]), plt.yticks([])

plt.show()

