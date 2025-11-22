import numpy as np

def nearest_neighbor(src, dst):
    """
    src: (N, 3) 來源點雲
    dst: (M, 3) 目標點雲
    回傳:
      distances: (N,) 每個來源點到最近目標點的距離
      indices:   (N,) 每個來源點對應到的目標點 index
    """
    # src[:, None, :] -> (N,1,3)
    # dst[None, :, :] -> (1,M,3)
    diff = src[:, None, :] - dst[None, :, :]   # (N,M,3)
    dist2 = np.sum(diff**2, axis=2)            # (N,M)
    indices = np.argmin(dist2, axis=1)         # 對每個來源點找最近目標點的 index
    distances = np.sqrt(dist2[np.arange(src.shape[0]), indices])
    return distances, indices
