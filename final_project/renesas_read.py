import serial
import time
import re
import socketio

# --- 設定區 ---
SERIAL_PORT = 'COM11'  # 樹莓派通常是 /dev/ttyACM0 或 /dev/ttyUSB0
BAUD_RATE = 115200            # 根據你的設備調整，通常是 115200
SERVER_URL = "http://192.168.50.105:5000"

# 動作與指令代號的對應表 (根據你的 index.html 設定)
ACTION_MAP = {
    "punch": "0",  # 烈焰衝擊
    "shake": "3",  # 雷鳴亂流
    "down": "2",   # 大地崩裂
}

# --- 初始化 ---
sio = socketio.Client()
last_action = None
last_time = 0

def connect_server():
    try:
        if not sio.connected:
            sio.connect(SERVER_URL)
            print(f"成功連線至 Server: {SERVER_URL}")
    except Exception as e:
        print(f"連線 Server 失敗: {e}")

def process_action(current_action):
    global last_action, last_time
    current_time = time.time()
    
    # 邏輯判斷：連續兩次動作一樣 且 間隔小於 7 秒
    if current_action == last_action:
        interval = current_time - last_time
        if interval <= 7.0:
            label = ACTION_MAP.get(current_action)
            if label:
                print(f"【觸發攻擊】偵測到連續 {current_action}！發送指令 {label} (間隔 {interval:.2f}s)")
                sio.emit('renesas_cast_spell', {'label': label})
                # 觸發後重置，避免第三次動作又觸發一次 (看你需求)
                last_action = None 
                return
        else:
            print(f"動作匹配但間隔太久 ({interval:.2f}s)，不觸發")
    
    # 更新最後一次的狀態
    last_action = current_action
    last_time = current_time

def main():
    connect_server()
    while (1):
        # 開啟 Serial Port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"正在讀取 Serial Port: {SERIAL_PORT}...")
        
        while True:
            if ser.in_waiting > 0:
                # 讀取一行並解碼
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # 使用正規表達式提取動作名稱
                # 匹配格式: Predicted Class: punch [0.40]
                match = re.search(r"Predicted Class:\s+(\w+)", line)
                
                if match:
                    detected_class = match.group(1)
                    print(f"收到動作: {detected_class}")
                    process_action(detected_class)
            
            # 保持 Socket 活著
            sio.sleep(0.01)

       

if __name__ == '__main__':
    main()