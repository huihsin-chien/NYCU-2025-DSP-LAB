import sys
import time
import board
import neopixel_spi as neopixel 
import signal

# --- LED 燈條配置 ---
# 注意：使用 board.D18 通常對 NeoPixel 在 RPi 上最可靠。
# 如果您堅持使用 GPIO 12，請將 PIN 設定為 board.D12。
SPI_BUS = board.SPI() 
LED_COUNT      = 10        # 燈珠數量
LED_BRIGHTNESS = 0.2       # 亮度 (0.0 到 1.0)
LED_ORDER      = neopixel.GRB # 燈珠顏色順序 (通常是 GRB 或 RGB)

# 初始化 NeoPixel 物件
# 自動使用 rpi-ws281x 驅動
pixels = neopixel.NeoPixel_SPI(
    SPI_BUS, 
    LED_COUNT,
    brightness=LED_BRIGHTNESS,
    auto_write=False
)

def turn_off():
    """關閉所有燈珠"""
    pixels.fill((0, 0, 0))
    pixels.show()
    # 清理函式 (確保在腳本結束時關閉燈條)
    pixels.deinit()
    
# 設置訊號處理，確保程式被中斷時能關燈
def signal_handler(sig, frame):
    turn_off()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# --- 燈效函數 ---

def set_color(rgb_tuple):
    """設置所有燈珠為單一顏色."""
    pixels.fill(rgb_tuple)
    pixels.show()

def blink_red_fast():
    """閃爍模式 1：紅色快速閃爍 (100ms)"""
    for _ in range(2): # 執行兩次閃爍
        set_color((255, 0, 0))  # 紅色
        time.sleep(0.1)
        set_color((0, 0, 0))    # 關燈
        time.sleep(0.1)

def blink_blue_slow():
    """閃爍模式 2：藍色慢速閃爍 (500ms)"""
    for _ in range(2): # 執行兩次閃爍
        set_color((0, 0, 255))
        time.sleep(0.5)
        set_color((0, 0, 0))
        time.sleep(0.5)

def breathe_green():
    """閃爍模式 3：綠色呼吸燈（簡易版）"""
    steps = 5
    
    # 增亮
    for b in range(0, 256, steps):
        pixels.fill((0, b, 0))
        pixels.show()
        time.sleep(0.01)

    # 減弱
    for b in range(255, -1, -steps):
        pixels.fill((0, b, 0))
        pixels.show()
        time.sleep(0.01)

# --- 主執行部分 ---
if __name__ == '__main__':
    label = sys.argv[1] if len(sys.argv) > 1 else "off"
    print("--- 腳本開始執行 ---") # <-- 新增
    # label = "class1"
    
    # 執行對應的燈效
    if label == "class1":
        blink_red_fast()
    elif label == "class2":
        blink_blue_slow()
    elif label == "class3":
        breathe_green()
    else:
        # 這裡不執行 turn_off()，因為我們希望在腳本執行結束時自動關閉
        blink_red_fast()
        pass
        
    # 程式結束時會調用 deinit() 關閉燈光
    turn_off()
