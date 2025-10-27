import re

input_file = "C:\\Users\\jianh\\OneDrive\\Documents\\大學\\三上\\DSP lab\\LAB03-20251015\\car_90cm\\Log 2025-10-15 16_57_53.txt"
output_file = "C:\\Users\\jianh\\OneDrive\\Documents\\大學\\三上\\DSP lab\\LAB03-20251015\\car3_extracted.txt"

# 模式允許 "(0x)" 前面沒引號、後面有可能沒有空格
pattern = re.compile(r'\(0x\)\s*([0-9A-Fa-f][0-9A-Fa-f\-\s]*)')

unique_results = []
seen = set()

with open(input_file, "r", encoding="utf-8") as f:
    for line in f:
        # 嘗試擷取所有符合 "(0x)" 格式的資料
        match = pattern.search(line)
        if not match:
            continue

        hex_str = match.group(1).strip().replace(' ', '')  # 移除多餘空白
        bytes_list = hex_str.split('-')

        # 資料長度檢查，至少要有 18 bytes 才能取中間 6 個
        if len(bytes_list) >= 18:
            middle_bytes = '-'.join(bytes_list[4:16])
            if middle_bytes not in seen:
                seen.add(middle_bytes)
                unique_results.append(middle_bytes)

# 輸出
with open(output_file, "w", encoding="utf-8") as f:
    for r in unique_results:
        f.write(r + "\n")

print(f"擷取完成，共 {len(unique_results)} 筆資料，結果存於 {output_file}")
