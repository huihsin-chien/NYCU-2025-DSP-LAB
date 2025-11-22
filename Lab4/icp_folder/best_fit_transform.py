import numpy as np
def best_fit_transform(A, B):
    """
    A: (N,3) 來源點
    B: (N,3) 目標點 (已配對)
    回傳:
      R: (3,3) 旋轉矩陣
      t: (3,)   平移向量
    """
    assert A.shape == B.shape

    # 1) 各自中心
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    # 2) 搬到各自中心
    AA = A - centroid_A
    BB = B - centroid_B

    # 3) 算 H
    H = AA.T @ BB   # (3,3)

    # 4) SVD
    U, S, Vt = np.linalg.svd(H)

    # 5) 旋轉
    R = Vt.T @ U.T

    # 避免發生反射（det(R) = -1）
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 6) 平移
    t = centroid_B - R @ centroid_A

    return R, t
