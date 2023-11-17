import cv2
import numpy as np
from matplotlib import pyplot as plt


img = cv2.imread('/home/morita/ros2_ws/image/portrait7.jpg',0)

med_val = np.median(img)
sigma = 0.33  # 0.33
min_val = int(max(0, (1.0 - sigma) * med_val))
max_val = int(max(255, (1.0 + sigma) * med_val))

edges = cv2.Canny(img, threshold1 = min_val, threshold2 = max_val)
_, gray = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
 
cv2.imwrite('/home/morita/ros2_ws/output/canny.jpg', edges)


plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])



plt.show()
