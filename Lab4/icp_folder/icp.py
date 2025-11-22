import numpy as np
from nearest_neighbor import nearest_neighbor
from best_fit_transform import best_fit_transform
from depth_to_point_cloud import depth_to_point_cloud



def icp(source, target, max_iterations=50, tolerance=1e-5, init_R=None, init_t=None):
    """
    source: (N,3) 要被搬動的點雲 (紅)
    target: (M,3) 固定不動的點雲 (藍)
    回傳:
      R_total, t_total, aligned_source
    """
    src = source.copy()

    # 初始化
    if init_R is None:
        R_total = np.eye(3)
    else:
        R_total = init_R
        src = (init_R @ src.T).T

    if init_t is None:
        t_total = np.zeros(3)
    else:
        t_total = init_t
        src = src + init_t

    prev_error = np.inf

    for i in range(max_iterations):
        # (1) 找最近點
        distances, indices = nearest_neighbor(src, target)

        # (2) 算最佳 R, t
        R, t = best_fit_transform(src, target[indices])

        # (3) 套用變換到目前的 src
        src = (R @ src.T).T + t

        # (4) 更新總變換（把每次的小 R,t 累積起來）
        R_total = R @ R_total
        t_total = R @ t_total + t

        # (5) 檢查誤差是否收斂
        mean_error = distances.mean()
        # print(f"iter {i}, mean error = {mean_error:.6f}")
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return R_total, t_total, src
