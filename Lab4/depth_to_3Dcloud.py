# depth_to_ply_and_stl_fixed.py
import numpy as np
import cv2
import os

# ===== 路徑（改成你的） =====
RAW_PATH = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_Depth.raw"
PNG_PATH = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_Depth.png"  # 可無
OUT_PLY  = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_fixed.ply"
OUT_STL  = r"/home/ajubi/shimizuken/dsp_lab4/lidar_desk_fixed.stl"

# ===== 相機參數（固定值，依你要求） =====
W, H = 640, 480
fx = 456.25
fy = 456.18
cx = 306.34
cy = 251.48

# ===== 其他參數 =====
DEPTH_SCALE = 1000.0  # Z(m) = depth_uint16 / DEPTH_SCALE；不確定就試 1000.0 或 5000.0
DEPTH_TRUNC = 5.0     # 超過此距離(公尺)的點剔除，避免遠距離雜點
CONT_THRESH = 0.10    # STL 用的連續性門檻(公尺)：同一格內深度跳躍>10cm就不連

def read_raw_u16(path, width, height):
    buf = np.fromfile(path, dtype=np.uint16)
    if buf.size != width * height:
        raise ValueError(f"RAW size {buf.size} != {width*height}")
    return buf.reshape(height, width)

def backproject(depth_u16, fx, fy, cx, cy, depth_scale):
    Z = depth_u16.astype(np.float32) / depth_scale
    # 截斷太遠的深度
    if DEPTH_TRUNC is not None:
        Z = np.where((Z > 0) & (Z <= DEPTH_TRUNC), Z, 0.0)
    h, w = Z.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y, Z

def write_ply_color(path, points, colors):
    points = points.astype(np.float32)
    colors = colors.astype(np.uint8)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x,y,z),(r,g,b) in zip(points, colors):
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")

def write_ascii_stl(path, triangles):
    with open(path, "w") as f:
        f.write("solid depth_surface\n")
        for tri in triangles:
            v1, v2, v3 = tri
            n = np.cross(v2 - v1, v3 - v1)
            n /= (np.linalg.norm(n) + 1e-12)
            f.write(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n")
            f.write("    outer loop\n")
            for (x,y,z) in (v1, v2, v3):
                f.write(f"      vertex {x:.6e} {y:.6e} {z:.6e}\n")
            f.write("    endloop\n  endfacet\n")
        f.write("endsolid depth_surface\n")

if __name__ == "__main__":
    # 1) 讀取深度
    depth = read_raw_u16(RAW_PATH, W, H)

    # 2) 反投影
    X, Y, Z = backproject(depth, fx, fy, cx, cy, DEPTH_SCALE)
    valid = Z > 0
    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    mask = valid.reshape(-1)

    # 3) PLY（若有 PNG 就用顏色，否則用灰）
    colors = None
    if os.path.exists(PNG_PATH):
        img = cv2.imread(PNG_PATH, cv2.IMREAD_COLOR)
        if img is not None:
            if (img.shape[1], img.shape[0]) != (W, H):
                img = cv2.resize(img, (W, H), interpolation=cv2.INTER_NEAREST)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            colors = img.reshape(-1, 3)[mask]
    if colors is None:
        # 用深度做簡單色帶（藍-紅）
        z = pts[mask, 2]
        z = (z - z.min()) / (z.max() - z.min() + 1e-9)
        colors = np.stack([z * 255, (1 - z) * 255, np.full_like(z, 128)], axis=1)

    write_ply_color(OUT_PLY, pts[mask], colors)
    print("Wrote PLY:", OUT_PLY, "points:", mask.sum())

    # 4) STL（把像素網格2×2切兩個三角形，含連續性檢查）
    tris = []
    for i in range(H - 1):
        for j in range(W - 1):
            a=(i,j); b=(i,j+1); c=(i+1,j); d=(i+1,j+1)
            if not (valid[a] and valid[b] and valid[c] and valid[d]):
                continue
            # 連續性：避免跨大深度跳躍
            if max(abs(Z[a]-Z[b]), abs(Z[a]-Z[c]), abs(Z[b]-Z[d]),
                   abs(Z[c]-Z[d]), abs(Z[a]-Z[d]), abs(Z[b]-Z[c])) > CONT_THRESH:
                continue
            Pa = np.array([X[a], Y[a], Z[a]], dtype=np.float64)
            Pb = np.array([X[b], Y[b], Z[b]], dtype=np.float64)
            Pc = np.array([X[c], Y[c], Z[c]], dtype=np.float64)
            Pd = np.array([X[d], Y[d], Z[d]], dtype=np.float64)
            tris.append(np.stack([Pa, Pb, Pc], axis=0))
            tris.append(np.stack([Pb, Pd, Pc], axis=0))

    tris = np.array(tris, dtype=np.float64)
    write_ascii_stl(OUT_STL, tris)
    print("Wrote STL:", OUT_STL, "faces:", tris.shape[0])
