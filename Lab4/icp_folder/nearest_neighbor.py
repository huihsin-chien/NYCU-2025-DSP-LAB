# nearest_neighbor.py
import numpy as np
from scipy.spatial import cKDTree

def nearest_neighbor(src, dst):
    """
    src: (N, 3) 來源點雲
    dst: (M, 3) 目標點雲
    回傳:
      distances: (N,) 每個來源點到最近目標點的距離
      indices:   (N,) 每個來源點對應到的目標點 index
    """
    # 在目標點雲上建立 kd-tree
    tree = cKDTree(dst)

    # 對每個 src 點查詢最近的 dst 點
    distances, indices = tree.query(src, k=1)

    return distances.astype(np.float32), indices
