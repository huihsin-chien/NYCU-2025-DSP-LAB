import numpy as np
def point_cloud_to_depth(points, fx, fy, cx, cy, width, height, depth_scale):
    """
    points: (N,3) 合併後點雲
    回傳: depth_u16 (H,W)
    """
    X, Y, Z = points[:,0], points[:,1], points[:,2]
    # 投影到像素座標
    u = (X * fx / Z + cx).round().astype(int)
    v = (Y * fy / Z + cy).round().astype(int)

    depth = np.zeros((height, width), dtype=np.uint16)

    # 只保留在畫面內的點
    mask = (u >= 0) & (u < width) & (v >= 0) & (v < height) & (Z > 0)
    u = u[mask]; v = v[mask]; Z = Z[mask]

    # 如果多個點落在同一個 (v,u)，通常取最近的那個
    depth_float = np.full((height, width), np.inf, dtype=np.float32)
    for x, y, z in zip(u, v, Z):
        if z < depth_float[y, x]:
            depth_float[y, x] = z

    valid = np.isfinite(depth_float)
    depth[valid] = (depth_float[valid] * depth_scale).astype(np.uint16)
    return depth
