import re

input_file = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\LAB03-20251015\strait\square_3.txt"
output_file = r"C:\Users\jianh\OneDrive\Documents\大學\三上\DSP lab\LAB03-20251015\strait\ble_processed_square_3.txt"

# 改成允許前面有任意文字（.*?），抓雙引號中的 hex 字串
pattern = re.compile(r'.*?"([0-9A-Fa-f ]+)"\s*value received')

def split_to_2byte_list(hex_str: str):
    """把像 'F2BF 6EFA' 之類的字串拆成每 2 hex 一組的列表: ['F2','BF','6E','FA', ...]
    - 會保留原本 token 的順序，並移除非 hex 字元。
    """
    # 先抓出原本以空白或其他非 hex 分隔的 token
    orig_tokens = re.findall(r'[0-9A-Fa-f]+', hex_str)
    bytes2 = []
    for t in orig_tokens:
        clean = re.sub(r'[^0-9A-Fa-f]', '', t)
        # 拆成每 2 個 hex 為一組
        for i in range(0, len(clean), 2):
            bytes2.append(clean[i:i+2].upper())
    return bytes2


def process_hex_str(hex_str: str):
    """Process the captured hex string and return the middle bytes
    split into 2-hex groups separated by a space. Returns None if the
    original token count is too small (keeps compatibility with prior
    filtering logic).
    """
    orig_tokens = re.findall(r'[0-9A-Fa-f]+', hex_str)
    # 保持原本的最小 token 數檢查（原先是以每 token 為 4 hex 判斷）
    if len(orig_tokens) < 10:
        return None

    bytes2 = split_to_2byte_list(hex_str)
    # 原先 middle 使用 bytes_list[2:8]（取原 token 的第 3~8 個 token）
    # 對應到每個原 token 會拆成兩個 2-hex bytes，所以計算 slice
    start = 2 * 2   # 原 token index 2 對應到 bytes2 index 4
    end = 8 * 2     # 原 token index 8 (exclusive) 對應到 bytes2 index 16
    # 防禦式：若長度不足，則只取到目前可用的長度
    end = min(end, len(bytes2))
    middle_bytes = ' '.join(bytes2[start:end])
    return middle_bytes


unique_results = []
seen = set()

with open(input_file, "r", encoding="utf-8") as f:
    for line in f:
        match = pattern.search(line)
        if not match:
            continue

        hex_str = match.group(1).strip()
        middle = process_hex_str(hex_str)
        if not middle:
            continue
        print(f"middle_bytes: {middle}")
        if middle not in seen:
            seen.add(middle)
            unique_results.append(middle)

# 輸出到檔案
with open(output_file, "w", encoding="utf-8") as f:
    for r in unique_results:
        f.write(r + "\n")

print(f"擷取完成，共 {len(unique_results)} 筆資料，結果存於 {output_file}")
