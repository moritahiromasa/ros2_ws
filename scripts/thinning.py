import cv2
import numpy as np
from matplotlib import pyplot as plt



# 入力画像の取得
img = cv2.imread('/home/morita/ros2_ws/image/portrait7.jpg', cv2.IMREAD_GRAYSCALE)


### Canny  ###
med_val = np.median(img)
sigma = 0.33  # 0.33
min_val = int(max(0, (1.0 - sigma) * med_val))
max_val = int(max(255, (1.0 + sigma) * med_val))
 
canny_edges = cv2.Canny(img, threshold1 = min_val, threshold2 = max_val)



### Gaussian Laplacian   ###
kernel_size = 5
sigma = 0
blurred = cv2.GaussianBlur(img, (kernel_size, kernel_size), sigma) # ガウシアンフィルターの適用

# ラプラシアン フィルターを適用
laplacian_8U_k3 = cv2.Laplacian(blurred, cv2.CV_8U, ksize=3)
laplacian_8U_k5 = cv2.Laplacian(blurred, cv2.CV_8U, ksize=5)
laplacian_32F_k3 = cv2.Laplacian(blurred, cv2.CV_32F, ksize=3)
laplacian_32F_k5 = cv2.Laplacian(blurred, cv2.CV_32F, ksize=5)



# データ型をcv2.CV_32F(float32)からcv2.CV_8U(uint8)に変換
laplacian_32F_k3 = np.clip(laplacian_32F_k3 * 255, a_min = 0, a_max = 255).astype(np.uint8)
laplacian_32F_k5 = np.clip(laplacian_32F_k5 * 255, a_min = 0, a_max = 255).astype(np.uint8)


# 絶対値を取得
laplacian_8U_k3 = cv2.convertScaleAbs(laplacian_8U_k3)
laplacian_8U_k5 = cv2.convertScaleAbs(laplacian_8U_k5)
laplacian_32F_k3 = cv2.convertScaleAbs(laplacian_32F_k3)
laplacian_32F_k5 = cv2.convertScaleAbs(laplacian_32F_k5)


# 細線化(スケルトン化) THINNING_ZHANGSUEN
skeleton1 = cv2.ximgproc.thinning(laplacian_8U_k5, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
cv2.imwrite('/home/morita/ros2_ws/output/thinning_zhangsuen.jpg', skeleton1)
cv2.imwrite('/home/morita/ros2_ws/output/canny.jpg', canny_edges)


# 細線化(スケルトン化) THINNING_GUOHALL 
#skeleton2 = cv2.ximgproc.thinning(img, thinningType=cv2.ximgproc.THINNING_GUOHALL)
#cv2.imwrite('/home/morita/ros2_ws/output/thinning_ghouhall.jpg', skeleton2)
	

plt.subplot(131),plt.imshow(img, cmap='gray')
plt.title('Original'), plt.xticks([]), plt.yticks([])
plt.subplot(132),plt.imshow(canny_edges, cmap='gray')
plt.title('Canny Edge'), plt.xticks([]), plt.yticks([])
plt.subplot(133),plt.imshow(skeleton1, cmap='gray')
plt.title('Laplacian thinning'), plt.xticks([]), plt.yticks([])

plt.show()
