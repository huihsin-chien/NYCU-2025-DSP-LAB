import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

csv_path = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\Lab4\DSPLAB04\part2\part2_emit_times_ps.csv"    # 你的 CSV 檔案路徑

# 嘗試自動偵測分隔符號
df = pd.read_csv(csv_path, sep=None, engine="python", header=None, low_memory=False)

print(f"讀到的資料維度: {df.shape}")
print("前5列資料：")
print(df.head())

# 假設你要第一欄
column_index = 9

if df.shape[1] <= column_index:
    raise ValueError(f"資料只有 {df.shape[1]} 欄，沒有第 {column_index} 欄！")

numbers = df.iloc[:, column_index].dropna().astype(float).to_numpy()
print(f"成功讀取 {len(numbers)} 筆數據")
