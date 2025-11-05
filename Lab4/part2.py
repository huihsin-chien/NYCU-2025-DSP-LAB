import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
emit_file = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\Lab4\DSPLAB04\part2\part2_emit_times_ps.csv"
receive_file = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\Lab4\DSPLAB04\part2\part2_receive_times_ps.csv"

df_emit = pd.read_csv(emit_file, header=None)
df_receive = pd.read_csv(receive_file, header=None)

emit_times = np.sort(df_emit.iloc[:, 0].values)
receive_times = np.sort(df_receive.iloc[:, 0].values)

diffs = find_closest_receive_after_emit(emit_times, receive_times, max_diff=1250)
valid_diffs = diffs[~np.isnan(diffs)]

# 建立 histogram
counts, bin_edges = np.histogram(valid_diffs, bins=500)
bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2

# --- 滑動視窗掃描 ---
window_size = 10  # 你可以調整視窗大小
best_sum = 0
best_centroid = None

for i in range(len(counts) - window_size + 1):
    window_counts = counts[i:i + window_size]
    window_bins = bin_centers[i:i + window_size]
    total = np.sum(window_counts)
    if total > best_sum:
        # 計算該 window 的質心 (weighted average)
        centroid = np.sum(window_bins * window_counts) / total
        best_sum = total
        best_centroid = centroid

print(f"最佳視窗總count = {best_sum}")
print(f"對應質心時間差 = {best_centroid:.3f} ps")

# --- 計算距離 ---
# 光速 c = 3e8 m/s = 0.3 m/ns = 0.0003 m/ps
distance_m = 0.015 * best_centroid  # 除以2, 光走去回
print(f"估計物體距離 = {distance_m:.3f} 公尺")

# --- 畫圖 ---
plt.figure(figsize=(10,6))
plt.bar(bin_centers, counts, width=bin_edges[1]-bin_edges[0], color='skyblue', edgecolor='black')
plt.axvline(best_centroid, color='red', linestyle='--', label=f'Estimated peak: {best_centroid:.1f} ps\n({distance_m:.2f} m)')
plt.title('Histogram with estimated object distance')
plt.xlabel('Time difference (ps)')
plt.ylabel('Count')
plt.legend()
plt.grid(True)
plt.show()
