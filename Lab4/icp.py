# icp_from_depth_raw.py
# 目的：讀三筆 (raw+png) 深度資料 → 反投影成點雲 → 依照投影片 (1)~(5) 實作 ICP 對齊
# 依賴：open3d, numpy, opencv-python

import numpy as np
import open3d as o3d
import cv2
import os

# ===========================
# 路徑（改成你的三筆資料）
# ===========================
DATA = [
    dict(raw=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_Depth.raw",
         png=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_Depth.png"),
    dict(raw=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_2_Depth.raw",
         png=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_2_Depth.png"),
    dict(raw=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk3_Depth.raw",
         png=r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk3_Depth.png"),
]

# ===========================
# 相機內參（請依你的 CSV）
# 例：Fx/Fy=456.25/456.18, PPx/PPy=306.34/251.48
# ===========================
W, H = 640, 480
fx, fy = 456.25, 456.18
cx, cy = 306.34, 251.48          # = PPx, PPy
DEPTH_SCALE = 1000.0             # Z(m) = depth_uint16 / DEPTH_SCALE ；你也可試 1000.0
DEPTH_TRUNC = 5.0                # 超過此距離視為無效，視場景調 3.0~5.0 m

# ===========================
# ICP 參數（常用可調）
# ===========================
VOXEL = 0.004                    # 降採樣粒度（公尺），小→更細但噪點多
MAX_CORR_DIST = 0.015           # 對應點搜尋半徑（約 3cm）   #可以調小一點試試看
ICP_MAX_ITERS = 50               # 每層迭代次數
USE_POINT_TO_PLANE = True        # Plane 比 Point-to-Point 收斂快（需估法線）


# ---------- 工具：讀 RAW (uint16) ----------
def read_raw_u16(path, width, height):
    buf = np.fromfile(path, dtype=np.uint16)
    if buf.size != width * height:
        raise ValueError(f"{path}: RAW size {buf.size} != {width*height}")
    return buf.reshape(height, width)

# ---------- 工具：RAW + PNG → Open3D 點雲 ----------
def rawpng_to_pcd(raw_path, png_path, W, H, fx, fy, cx, cy,
                  depth_scale=5000.0, depth_trunc=4.0):
    depth_u16 = read_raw_u16(raw_path, W, H)

    # (可選) 先做小型去噪，讓邊緣穩定一點
    depth_u16 = depth_u16.copy()
    m = depth_u16[depth_u16 > 0]
    if m.size:
        depth_u16[depth_u16 == 0] = int(np.median(m))
    # 3x3 中值（保邊）
    depth_u16 = cv2.medianBlur(depth_u16, 3)

    depth_o3d = o3d.geometry.Image(depth_u16)

    if png_path and os.path.exists(png_path):
        color_bgr = cv2.imread(png_path, cv2.IMREAD_COLOR)
        if color_bgr is None:
            raise RuntimeError(f"read png failed: {png_path}")
        if (color_bgr.shape[1], color_bgr.shape[0]) != (W, H):
            color_bgr = cv2.resize(color_bgr, (W, H), interpolation=cv2.INTER_NEAREST)
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    else:
        color_rgb = np.full((H, W, 3), 255, np.uint8)
    color_o3d = o3d.geometry.Image(color_rgb)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d,
        depth_scale=depth_scale,      # Z_meter = depth / depth_scale
        depth_trunc=depth_trunc,      # 過遠截斷
        convert_rgb_to_intensity=False
    )

    intr = o3d.camera.PinholeCameraIntrinsic(int(W), int(H), float(fx), float(fy), float(cx), float(cy))
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr)  #pcd就是Point Cloud（點雲）。

    # 座標系調整（讓視覺上更直觀）
    pcd.transform([[1,0,0,0],
                   [0,-1,0,0],
                   [0,0,-1,0],
                   [0,0,0,1]])

    # 清噪＋降採樣＋估法線（為 point-to-plane 做準備）
    pcd = pcd.voxel_down_sample(VOXEL)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL*5, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(50)
    return pcd

# ---------- 工具：跑一次 ICP ----------
def run_icp(source, target, init=np.eye(4)):
    # (1) 初始化：初始變換（單位矩陣或大致方位）
    init_T = init.copy()

    # (2) 最近點搜尋 & (3) 估算變換：Open3D 的 registration_icp 內部完成
    if USE_POINT_TO_PLANE:
        method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        method = o3d.pipelines.registration.TransformationEstimationPointToPoint()

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITERS)

    result = o3d.pipelines.registration.registration_icp(
        source, target,
        max_correspondence_distance=MAX_CORR_DIST,
        init=init_T,
        estimation_method=method,
        criteria=criteria
    )

    # (4) 套用轉換：result.transformation 就是 T；(5) 迭代由 Open3D 內部做收斂判定
    return result


if __name__ == "__main__":
    # 讀 3 帧點雲
    pcds = [rawpng_to_pcd(d["raw"], d["png"], W, H, fx, fy, cx, cy,
                          DEPTH_SCALE, DEPTH_TRUNC) for d in DATA]

    # 以第 1 帧為基準，依序把 2、3 對齊上來
    base = pcds[0]
    acc_T = np.eye(4)

    for idx in [1, 2]:
        print(f"\n=== Align frame {idx+1} → frame 1 ===")
        src = pcds[idx]
        # 可加個粗略初始化（例如只給小位移/旋轉），這裡用 I
        result = run_icp(src, base, init=np.eye(4))
        print("fitness:", result.fitness, "inlier_rmse:", result.inlier_rmse)
        print("T =\n", result.transformation)

        # 套用轉換，把 source 變換到 base 座標系
        src_t = src.transform(result.transformation)
        # 累積（可視化或輸出）
        base += src  # 只是簡單疊加；正式要做融合/重採樣可用 o3d.geometry.VoxelGrid 等

    # 存檔預覽：最後疊合的點雲
    o3d.io.write_point_cloud("/home/ajubi/shimizuken/dsp_lab4/icp_merged.ply", base, write_ascii=False)
    print("Saved merged point cloud: icp_merged.ply")

    # 嘗試開視窗（若有 GUI）
    try:
        o3d.visualization.draw_geometries([base])
    except Exception as e:
        print("[INFO] GUI not available:", e)
