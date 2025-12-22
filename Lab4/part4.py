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

# =============================
# ⚙️ 全域配準輔助函式 (Global Registration Helpers)
# =============================

# 1. 準備點雲資料 (降取樣、計算法向量、計算 FPFH 特徵)
def prepare_dataset(pcd, voxel_size):
    # 下採樣 (Downsample)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # 法向量估計 (Normal Estimation)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    # FPFH 特徵計算
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

# 2. 執行全域配準 (RANSAC + FPFH)
# 2. 執行全域配準 (RANSAC + FPFH)
def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    # 設置參數
    distance_threshold = voxel_size * 1.5
    
    # 執行 RANSAC
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, 
        target_down, 
        source_fpfh, 
        target_fpfh,
        True, # 🌟 修正：加入 mutual_filter 參數 (通常設為 True)
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(3000,  confidence=0.999)
    )
    return result

# =============================
# 🖼️ 深度圖降噪函式 (Denoising)
# =============================
def denoise_depth_map(depth, kernel_size=5):
    """使用中值濾波器對深度圖進行降噪。"""
    # 由於深度圖是 np.uint16 格式，cv2.medianBlur 可以直接處理
    # kernel_size 必須是奇數 (例如 3, 5, 7)
    denoised_depth = cv2.medianBlur(depth, kernel_size)
    return denoised_depth


# =============================
# 🚀 主處理流程
# =============================

# 匯入多張
file_list = [
    ("LiDAR_data_cp/lidar_desk_Depth.raw", "LiDAR_data_cp/lidar_desk_Depth.png"),
    ("LiDAR_data_cp/lidar_desk_2_Depth.raw", "LiDAR_data_cp/lidar_desk_2_Depth.png"),
    ("LiDAR_data_cp/lidar_desk3_Depth.raw", "LiDAR_data_cp/lidar_desk3_Depth.png")
]

point_clouds = []
# ❗️ 設定下採樣的體素大小。根據您的場景大小，這個值可能需要調整。
# 0.01m = 1cm，是常見的初始值。
voxel_size = 0.01 

# 載入所有點雲，並計算用於粗略配準的特徵
pcd_data = [] # 儲存 {pcd_full, pcd_down, pcd_fpfh}
for raw_file, color_file in file_list:
    depth = load_depth_raw(raw_file)
    color = cv2.imread(color_file)
    depth_denoised = denoise_depth_map(depth, kernel_size=5)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    points, colors = depth_to_point_cloud(depth, color, fx, fy, cx, cy)
    pcd_full = create_point_cloud(points, colors)
    
    # 計算粗略配準所需的資料
    pcd_down, pcd_fpfh = prepare_dataset(pcd_full, voxel_size)
    pcd_data.append({
        "full": pcd_full, 
        "down": pcd_down, 
        "fpfh": pcd_fpfh
    })
    point_clouds.append(pcd_full) # 原始點雲列表，用於最終顯示
    o3d.visualization.draw_geometries(pcd_full)
    print(f"載入並預處理完成: {raw_file}")

# 初始化合併結果，以第一張點雲為基準
base_pcd = pcd_data[0]["full"]
current_transformation = np.eye(4) # 從第一個到當前點雲的總變換

# 迭代配準 (從第二張開始)
for i in range(1, len(pcd_data)):
    source = pcd_data[i] # 待對齊的點雲 (Source)
    target = pcd_data[i-1] # 前一張點雲 (Target)
    
    print(f"\n--- 正在對齊第 {i+1} 張 (Source) 到第 {i} 張 (Target) ---")

    # 1. 粗略配準 (Global Registration)
    print("1. 執行 RANSAC 粗略配準...")
    result_global = execute_global_registration(
        source["down"], target["down"], source["fpfh"], target["fpfh"], voxel_size
    )
    initial_transformation = result_global.transformation
    print(f"   粗略配準 Inlier: {result_global.fitness:.4f}")

    # 2. 精確配準 (ICP Registration)
    print("2. 執行 ICP 精確配準...")
    # ICP 距離門檻可以設得小一些，因為粗略配準已經給了好的初始值
    distance_threshold_icp = voxel_size / 2 
    
    # 以粗略配準的結果作為 ICP 的初始變換
    result_icp = o3d.pipelines.registration.registration_icp(
        source["full"], target["full"], distance_threshold_icp,
        initial_transformation, # 關鍵：使用粗略配準的結果
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    print(f"   ICP 配準 Fitness: {result_icp.fitness:.4f}")

    # 3. 變換累積與合併
    # 將當前點雲 (source) 轉換到世界座標系 (即 base_pcd 的座標系)
    pcd_data[i]["full"].transform(result_icp.transformation)
    base_pcd += pcd_data[i]["full"]
    print("--- 配準與合併完成 ---")


# 顯示最終結果
o3d.visualization.draw_geometries([base_pcd])

# 可選：儲存成 PLY 檔案
o3d.io.write_point_cloud("merged_result_improved.ply", base_pcd)
print("✅ 已輸出 merged_result_improved.ply")