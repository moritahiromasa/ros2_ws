import cv2
import numpy as np
from matplotlib import pyplot as plt


img = cv2.imread('/home/morita/ros2_ws/image/portrait3.jpg',0)
img1 = cv2.imread('/home/morita/ros2_ws/image/portrait3.jpg',3)

img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)


height, width = img.shape[:2] # 画像の縦横サイズを取得

# 縦長画像→幅を拡張する
if height > width:
	diffsize = height - width
	# 元画像を中央ぞろえにしたいので、左右に均等に余白を入れる
	padding_half = int(diffsize / 2)
	padding_img = cv2.copyMakeBorder(img, 0, 0, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))
	cv2.imwrite('/home/morita/output/sample@padding.jpg', padding_img)


# 横長画像→高さを拡張する
elif width > height:
	diffsize = width - height
	padding_half = int(diffsize / 2)
	padding_img = cv2.copyMakeBorder(img, 0, 0, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))
	cv2.imwrite('/home/morita/output/sample@padding.jpg', padding_img)

padding_img = cv2.resize(padding_img, (120, 120))

med_val = np.median(padding_img)
sigma = 0.33  # 0.33
min_val = int(max(0, (1.0 - sigma) * med_val))
max_val = int(max(255, (1.0 + sigma) * med_val))

edges = cv2.Canny(padding_img, threshold1 = min_val, threshold2 = max_val)
_, gray = cv2.threshold(padding_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
 
cv2.imwrite('/home/morita/ros2_ws/output/canny.jpg', edges)


plt.subplot(131),plt.imshow(img1,)
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(132),plt.imshow(img,cmap = 'gray')
plt.title('Grayscale Image'), plt.xticks([]), plt.yticks([])
plt.subplot(133),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])



plt.show()
