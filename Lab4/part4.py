import numpy as np
import cv2
import open3d as o3d
import glob
import matplotlib.pyplot as plt

# =============================
# 🔧 相機內參 (Intrinsic)
# =============================
fx, fy = 456.25, 456.175781
cx, cy = 306.34375, 251.482422
width, height = 640, 480

# =============================
# 🧮 讀取深度圖函式 (.raw)
# =============================
def load_depth_raw(filename, width=640, height=480):
    depth = np.fromfile(filename, dtype=np.uint16)
    depth = depth.reshape((height, width))
    return depth

# =============================
# 🎨 深度轉點雲函式
# =============================
def depth_to_point_cloud(depth, color, fx, fy, cx, cy):
    h, w = depth.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))
    z = depth.astype(np.float32) / 1000.0  # mm -> m
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy

    # 建立有效深度遮罩
    valid = (z > 0) & (z < 5)

    # flatten 成一維
    x = x[valid]
    y = y[valid]
    z = z[valid]
    points = np.stack((x, y, z), axis=-1)

    # 顏色 flatten 再套用同樣的 valid
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    colors = color.reshape(-1, 3)
    valid_flat = valid.flatten()
    colors = colors[valid_flat] / 255.0

    return points, colors


# =============================
# 🧱 建立 Open3D 點雲物件
# =============================
def create_point_cloud(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd
# 匯入多張
file_list = [
    ("LiDAR_data_cp/lidar_desk_Depth.raw", "LiDAR_data_cp/lidar_desk_Depth.png"),
    ("LiDAR_data_cp/lidar_desk_2_Depth.raw", "LiDAR_data_cp/lidar_desk_2_Depth.png"),
    ("LiDAR_data_cp/lidar_desk3_Depth.raw", "LiDAR_data_cp/lidar_desk3_Depth.png")
]

point_clouds = []
for raw_file, color_file in file_list:
    depth = load_depth_raw(raw_file)
    color = cv2.imread(color_file)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    points, colors = depth_to_point_cloud(depth, color, fx, fy, cx, cy)
    pcd = create_point_cloud(points, colors)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
    point_clouds.append(pcd)

    print(f"載入完成: {raw_file}")

# 配準 (ICP)
base_pcd = point_clouds[0]
for i in range(1, len(point_clouds)):
    print(f"正在對齊第 {i+1} 張點雲...")
    reg = o3d.pipelines.registration.registration_icp(
        point_clouds[i], base_pcd, 0.02,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    # 套用變換
    point_clouds[i].transform(reg.transformation)
    base_pcd += point_clouds[i]

# 顯示結果
o3d.visualization.draw_geometries(point_clouds)

# 可選：儲存成 PLY 檔案
o3d.io.write_point_cloud("merged_result.ply", base_pcd)
print("✅ 已輸出 merged_result.ply")
