import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
from scipy.signal import find_peaks
def find_closest_receive_after_emit(emit_times, receive_times, max_diff=1250):
    res = []
    j = 0
    n_receive = len(receive_times)
    
    for emit_t in emit_times:
        while j < n_receive and receive_times[j] <= emit_t:
            j += 1
        if j == n_receive:
            res.append(np.nan)
        else:
            diff = receive_times[j] - emit_t
            if diff < max_diff:
                res.append(diff)
            else:
                res.append(np.nan)
    return np.array(res)

# --- 主程式 ---
emit_file = r"/home/ajubi/shimizuken/dsp_lab4/part1_emit_times_ps.csv"       # 發射檔名
receive_file = r"/home/ajubi/shimizuken/dsp_lab4/part1_receive_times_ps.csv"

# 讀取csv，第0欄當作timestamp
df_emit = pd.read_csv(emit_file, header=None)
df_receive = pd.read_csv(receive_file, header=None)

# 取第0欄數值並排序
emit_times = np.sort(df_emit.iloc[:, 0].values)
receive_times = np.sort(df_receive.iloc[:, 0].values)

# 找最近的符合條件時間差
diffs = find_closest_receive_after_emit(emit_times, receive_times, max_diff=1250)

valid_diffs = diffs[~np.isnan(diffs)]

print(f"有效時間差數量: {len(valid_diffs)}")
print(f"時間差最大值: {valid_diffs.max()}")
print(f"時間差最小值: {valid_diffs.min()}")

counts, bin_edges = np.histogram(valid_diffs, bins=500)
peaks = find_peaks(counts, height=6000)[0]
print(f"找到的峰值位置: {bin_edges[peaks]}")
print("count數量",counts[peaks]);
print("peak:",peaks);
distance = bin_edges[peaks] / 2 * 3e8 / 1e10 #100ps  # 單位: 公尺
print(f"距離(峰值): {distance} 公尺")


# 畫 histogram
plt.figure(figsize=(8,5))
plt.hist(valid_diffs, bins=500, color='skyblue', edgecolor='black')
plt.title('Histogram of time differences (receive - emit)')
plt.xlabel('Time difference')
plt.ylabel('Count')
plt.grid(True)
plt.show()

# 存檔
out_dir = os.path.dirname(receive_file)
os.makedirs(out_dir, exist_ok=True)
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(out_dir, f"lidar_time_diff_hist_mode_{ts}.png")
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close()
print(f"直方圖（含標註）已存檔：{out_path}")

