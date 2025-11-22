import numpy as np
import cv2
from depth_to_point_cloud import read_raw_u16, depth_to_point_cloud
from icp import icp
from point_cloud_to_depth import point_cloud_to_depth

# ===== 相機參數（跟你原本 depth_to_3Dcloud 那支一樣）=====
W, H = 640, 480
fx = 456.25
fy = 456.18
cx = 306.34
cy = 251.48

DEPTH_SCALE = 1000.0
DEPTH_TRUNC = 5.0

# ===== 檔案路徑 =====
DEPTH1_RAW = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_Depth.raw"
DEPTH2_RAW = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_2_Depth.raw"
DEPTH3_RAW = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk3_Depth.raw"

OUT_MERGED_RAW   = r"/home/ajubi/shimizuken/dsp_lab4/icp_merged_depth.raw"
OUT_MERGED_PNG   = r"/home/ajubi/shimizuken/dsp_lab4/icp_merged_depth.png"
OUT_ROTATED_PNG  = r"/home/ajubi/shimizuken/dsp_lab4/icp_merged_depth_rotated.png"
OUT_EDGES_PNG    = r"/home/ajubi/shimizuken/dsp_lab4/icp_merged_depth_edges.png"

def main():
    # 1) 讀 RAW 深度圖（uint16）
    depth1 = read_raw_u16(DEPTH1_RAW, W, H)
    depth2 = read_raw_u16(DEPTH2_RAW, W, H)
    depth3 = read_raw_u16(DEPTH3_RAW, W, H)

    # 2) 深度圖 → 3D 點雲
    pc1 = depth_to_point_cloud(depth1, fx, fy, cx, cy, DEPTH_SCALE, DEPTH_TRUNC)
    pc2 = depth_to_point_cloud(depth2, fx, fy, cx, cy, DEPTH_SCALE, DEPTH_TRUNC)
    pc3 = depth_to_point_cloud(depth3, fx, fy, cx, cy, DEPTH_SCALE, DEPTH_TRUNC)

    # 3) ICP 把多張點雲合併
    # 以 pc1 當世界座標
    merged = pc1.copy()

    # 把 pc2 貼到 pc1 / merged
    R_21, t_21, pc2_aligned = icp(pc2, merged)
    merged = np.vstack([merged, pc2_aligned])

    # 把 pc3 也貼到 merged
    R_31, t_31, pc3_aligned = icp(pc3, merged)
    merged = np.vstack([merged, pc3_aligned])

    print("Merged point cloud size:", merged.shape)

    # 4) 合併後的點雲 → 深度圖（原始視角）
    merged_depth = point_cloud_to_depth(
        merged, fx, fy, cx, cy, W, H, DEPTH_SCALE
    )

    # 存成 RAW
    merged_depth.tofile(OUT_MERGED_RAW)

    # ---- (A) 原視角灰階深度圖 ----
    depth_for_show = (merged_depth.astype(np.float32) / DEPTH_SCALE)
    depth_for_show = np.clip(depth_for_show / DEPTH_TRUNC, 0, 1)
    depth_gray = (depth_for_show * 255).astype(np.uint8)
    cv2.imwrite(OUT_MERGED_PNG, depth_gray)
    print("Saved merged depth PNG (gray):", OUT_MERGED_PNG)

    # ---- (B) 轉視角：旋轉點雲再投影 ----
    # 例：繞 Y 軸旋轉 20 度
    theta = np.deg2rad(20.0)
    R_view = np.array([
        [ np.cos(theta), 0.0, np.sin(theta)],
        [ 0.0,           1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ], dtype=np.float32)

    merged_rot = (R_view @ merged.T).T  # (N,3)

    rotated_depth = point_cloud_to_depth(
        merged_rot, fx, fy, cx, cy, W, H, DEPTH_SCALE
    )

    depth_rot_show = (rotated_depth.astype(np.float32) / DEPTH_SCALE)
    depth_rot_show = np.clip(depth_rot_show / DEPTH_TRUNC, 0, 1)
    depth_rot_gray = (depth_rot_show * 255).astype(np.uint8)
    cv2.imwrite(OUT_ROTATED_PNG, depth_rot_gray)
    print("Saved rotated-view depth PNG (gray):", OUT_ROTATED_PNG)

    # ---- (C) 輪廓圖：對原視角灰階圖做 Laplacian 邊緣偵測 ----
    edges_laplace = cv2.Laplacian(depth_gray, cv2.CV_64F)
    edges_laplace = cv2.convertScaleAbs(edges_laplace)
    cv2.imwrite(OUT_EDGES_PNG, edges_laplace)
    print("Saved Laplacian edge PNG:", OUT_EDGES_PNG)

if __name__ == "__main__":
    main()
