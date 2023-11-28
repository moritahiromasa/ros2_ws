import cv2
import numpy as np
from matplotlib import pyplot as plt



# 入力画像の取得
original_img = cv2.imread('/home/morita/ros2_ws/image/portrait9.jpg', 0)
img = cv2.imread('/home/morita/ros2_ws/output/canny.jpg', 0)


# 細線化(スケルトン化) THINNING_ZHANGSUEN
skeleton1 = cv2.ximgproc.thinning(img, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
cv2.imwrite('/home/morita/ros2_ws/output/thinning_zhangsuen.jpg', skeleton1)


# 細線化(スケルトン化) THINNING_GUOHALL 
skeleton2 = cv2.ximgproc.thinning(img, thinningType=cv2.ximgproc.THINNING_GUOHALL)
cv2.imwrite('/home/morita/ros2_ws/output/thinning_ghouhall.jpg', skeleton2)
	

plt.subplot(121),plt.imshow(original_img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(skeleton1,cmap = 'gray')
plt.title('Thinned Image'), plt.xticks([]), plt.yticks([])

plt.show()
