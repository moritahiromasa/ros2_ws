from PIL import Image
import os, sys
import cv2
import numpy as np
from matplotlib import pyplot as plt



img1 = cv2.imread('/home/morita/ros2_ws/Memo/tex/img/img1.png')
img2 = cv2.imread('/home/morita/ros2_ws/Memo/tex/img/img3.png')


img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)
img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)


plt.subplot(121),plt.imshow(img1)
plt.title('link coordinate'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(img2)
plt.title('ADA031'), plt.xticks([]), plt.yticks([])

plt.show()

