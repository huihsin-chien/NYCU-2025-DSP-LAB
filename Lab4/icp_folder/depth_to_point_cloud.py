import numpy as np
def depth_to_point_cloud(depth_u16, fx, fy, cx, cy, depth_scale, depth_trunc=None):
    Z = depth_u16.astype(np.float32) / depth_scale
    if depth_trunc is not None:
        Z = np.where((Z > 0) & (Z <= depth_trunc), Z, 0.0)

    h, w = Z.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    valid = Z > 0
    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=-1)  # (N,3)
    return pts
def read_raw_u16(path, width, height):
    buf = np.fromfile(path, dtype=np.uint16)
    if buf.size != width * height:
        raise ValueError(f"RAW size {buf.size} != {width*height}")
    return buf.reshape(height, width)