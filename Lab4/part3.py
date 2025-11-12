import numpy as np
import cv2
import matplotlib.pyplot as plt

# 讀取 .raw 檔案
file = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\Lab4\LiDAR_data\LiDAR_desk1\lidar_desk_Depth.raw"
width, height = 640, 480  # 根據你的相機設定修改
depth_raw = np.fromfile(file, dtype=np.uint16)
depth_image = depth_raw.reshape((height, width))

# 將深度值正規化以便顯示
depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
depth_normalized = np.uint8(depth_normalized)

plt.imshow(depth_normalized, cmap='gray')
plt.title('Depth Image')
plt.show()

edges_sobel = cv2.Sobel(depth_normalized, cv2.CV_64F, 1, 1, ksize=5)
edges_sobel = cv2.convertScaleAbs(edges_sobel)
cv2.imshow('Sobel', edges_sobel)
cv2.waitKey(0)


edges_canny = cv2.Canny(depth_normalized, 50, 150)
cv2.imshow('Canny Edges', edges_canny)
cv2.waitKey(0)

edges_laplace = cv2.Laplacian(depth_normalized, cv2.CV_64F)
edges_laplace = cv2.convertScaleAbs(edges_laplace)
cv2.imshow('Laplace', edges_laplace)
cv2.waitKey(0)
