import time
import board
# 引入基於 SPI 的函式庫
import neopixel_spi as neopixel 

# --- 配置參數 ---
# 樹莓派 5 預設的 SPI 匯流排
SPI_BUS = board.SPI() 
# 燈條上的 LED 數量
NUM_PIXELS = 8 
# 亮度 (0.0 到 1.0)
BRIGHTNESS = 0.5 

# 初始化 NeoPixel 燈條
# 注意：使用 SPI 需要指定 SPI 匯流排
pixels = neopixel.NeoPixel_SPI(
    SPI_BUS, 
    NUM_PIXELS, 
    brightness=BRIGHTNESS, 
    auto_write=False
)

print(f"啟動 {NUM_PIXELS} 顆 NeoPixel 測試...")

def color_chase(color, wait):
    """依序點亮每顆燈珠"""
    for i in range(NUM_PIXELS):
        pixels[i] = color
        pixels.show()
        time.sleep(wait)

def color_all(color):
    """將所有燈珠設為同一顏色"""
    pixels.fill(color)
    pixels.show()
    time.sleep(2)

try:
    print("測試：紅色")
    color_all((255, 0, 0))
    time.sleep(1)

    print("測試：綠色")
    color_all((0, 255, 0))
    time.sleep(1)

    print("測試：藍色")
    color_all((0, 0, 255))
    time.sleep(1)
    
    print("測試：彩虹追逐")
    # RGB 顏色 (255, 0, 0) 是紅色
    color_chase((255, 0, 0), 0.1) 
    color_chase((0, 255, 0), 0.1) 
    color_chase((0, 0, 255), 0.1)

except Exception as e:
    print(f"程式運行錯誤: {e}")

finally:
    # 結束時關閉所有燈光
    pixels.fill((0, 0, 0))
    pixels.show()
    print("測試結束，燈光已關閉。")