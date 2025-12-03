import time
import threading
from flask import Flask, render_template, request, jsonify
import smbus2 # MPU6050
try:
    import board
    import neopixel  # adafruit-circuitpython-neopixel for WS2812B
    NEOPIXEL_AVAILABLE = True
except Exception as e:
    # neopixel (adafruit) not available on this platform/environment
    print("neopixel import failed, falling back to rpi_ws281x:", e)
    NEOPIXEL_AVAILABLE = False
    from rpi_ws281x import PixelStrip, Color

import pygame.mixer as mixer # MP3 Playback
import math

# --- 1. 全域變數與設定 ---

# MPU6050 設定  
BUS = smbus2.SMBus(1)
MPU6050_ADDR = 0x68
ACCEL_THRESHOLD = 2.0 # 加速度閾值 (單位: g)

# WS2812B 設定
LED_COUNT = 30        # 燈條上的 LED 數量
LED_PIN = 18          # 樹莓派 GPIO pin (18 is PWM0/PCM)
LED_FREQ_HZ = 800000  # LED 訊號頻率 (Hz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # 亮度 (0 to 255)
LED_CHANNEL = 0

# Map brightness from 0-255 to 0.0-1.0 for neopixel (if available)
if NEOPIXEL_AVAILABLE:
    LED_BRIGHTNESS_F = max(0.0, min(1.0, LED_BRIGHTNESS / 255.0))
    # Use board.D18 for GPIO18 PWM pin
    strip = neopixel.NeoPixel(board.D18, LED_COUNT, brightness=LED_BRIGHTNESS_F, auto_write=False, pixel_order=neopixel.GRB)
else:
    # Provide a minimal shim with the API your code expects: fill(), show(), __setitem__, len()
    class NeoPixelShim:
        def __init__(self, count, pin, freq_hz, dma, invert, brightness_int, channel):
            self._count = count
            self._hw_ok = False
            self._buffer = [(0, 0, 0)] * count
            try:
                # Try to create underlying PixelStrip (rpi_ws281x)
                self._strip = PixelStrip(count, pin, freq_hz, dma, invert, brightness_int, channel)
                try:
                    self._strip.begin()
                    self._hw_ok = True
                except Exception as e:
                    # Hardware init failed (e.g. unsupported revision) — fall back to buffer
                    print("PixelStrip.begin() failed, using dummy buffer:", e)
            except Exception as e:
                # PixelStrip creation failed — fall back to buffer
                print("PixelStrip creation failed, using dummy buffer:", e)

        def __len__(self):
            return self._count

        def fill(self, color_tuple):
            r, g, b = [int(x) for x in color_tuple]
            if self._hw_ok:
                for i in range(self._count):
                    self._strip.setPixelColor(i, Color(r, g, b))
            else:
                self._buffer = [(r, g, b)] * self._count

        def show(self):
            if self._hw_ok:
                self._strip.show()
            else:
                # no-op for dummy buffer (optionally log)
                pass

        def __setitem__(self, idx, color_tuple):
            r, g, b = [int(x) for x in color_tuple]
            if self._hw_ok:
                self._strip.setPixelColor(idx, Color(r, g, b))
            else:
                self._buffer[idx] = (r, g, b)

    # instantiate shim using same LED_* constants
    strip = NeoPixelShim(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, False, LED_BRIGHTNESS, LED_CHANNEL)

# MP3 設定
MP3_FILE_DEFAULT = 'sweet-life-luxury-chill-438146.mp3' # 請自行準備音檔並放在同目錄
mixer.init()
current_mp3 = MP3_FILE_DEFAULT

# Flask/狀態 設定
app = Flask(__name__)
# 儲存超過閾值的時間戳記
log_timestamps = []
# 控制燈光/音樂的狀態
current_pattern = 'Solid Red' # 初始燈光模式
is_music_enabled = True # 是否播放音樂

# --- 2. 燈光/音樂 控制函式 ---

def play_alert_music():
    """播放當前設定的MP3音樂一次。"""
    if is_music_enabled:
        try:
            mixer.music.load(current_mp3)
            mixer.music.play()
        except Exception as e:
            print(f"Error playing MP3: {e}")

def set_led_pattern(pattern_name):
    """根據名稱設定 LED 燈光模式。"""
    global current_pattern
    current_pattern = pattern_name
    
    # 先關閉所有燈光
    strip.fill((0, 0, 0))
    strip.show()

    if pattern_name == 'Solid Red':
        strip.fill((255, 0, 0))
        strip.show()
    elif pattern_name == 'Flash Blue':
        # 閃爍會在 MPU 偵測迴圈中處理
        pass 
    elif pattern_name == 'Off':
        # 已在前面關閉
        pass
    
    # 實際的動態閃爍控制，將在 MPU 偵測迴圈中實作

# --- 3. MPU6050 核心偵測與控制迴圈 ---

def read_raw_data(addr):
    """讀取 MPU6050 16-bit 資料。"""
    high = BUS.read_byte_data(MPU6050_ADDR, addr)
    low = BUS.read_byte_data(MPU6050_ADDR, addr+1)
    
    value = ((high << 8) | low)
    
    if (value > 32768):
        value = value - 65536
    return value

def init_mpu6050():
    """初始化 MPU6050 (設定電源管理暫存器)。"""
    # 寫入電源管理暫存器 (喚醒 MPU6050)
    BUS.write_byte_data(MPU6050_ADDR, 0x6b, 0)
    print("MPU6050 initialized.")

def mpu_detector_loop():
    """獨立執行緒：持續讀取 MPU6050 並檢查閾值。"""
    init_mpu6050()
    
    while True:
        # 讀取加速度原始數據
        acc_x = read_raw_data(0x3b)
        acc_y = read_raw_data(0x3d)
        acc_z = read_raw_data(0x3f)
        
        # MPU6050 加速度敏感度 (2g 範圍, sensitivity = 16384 LSB/g)
        # 轉換為 g (重力加速度)
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        
        # 計算總加速度向量的量值 (排除重力影響，這裡只計算總量值)
        # 這裡檢查的是 **向量長度的變化**，但最簡單是檢查總加速度量值
        total_accel = math.sqrt(Ax**2 + Ay**2 + Az**2)

        # 檢查是否超過閾值
        if total_accel > ACCEL_THRESHOLD:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            print(f"!!! ALERT: Acceleration {total_accel:.2f}g > {ACCEL_THRESHOLD}g at {timestamp}")
            
            # 記錄時間戳記
            log_timestamps.append((timestamp, f"{total_accel:.2f}g"))
            
            # 播放音樂
            play_alert_music()
            
            # 觸發燈光閃爍 (Flash Blue)
            if current_pattern == 'Flash Blue':
                strip.fill((0, 0, 255))  # Blue
                strip.show()
                time.sleep(0.1)
                strip.fill((0, 0, 0))    # Off
                strip.show()
                time.sleep(0.1)
            
            # 避免連續觸發，延遲一下
            time.sleep(0.5) 

        else:
            # 正常狀態下的燈光處理 (如果不是閃爍模式，維持設定的靜態模式)
            if current_pattern == 'Solid Red':
                set_led_pattern('Solid Red')
            elif current_pattern == 'Off':
                set_led_pattern('Off')
            
            # 正常讀取延遲
            time.sleep(0.05)


# --- 4. Flask 網頁伺服器 ---

@app.route('/')
def index():
    """主頁面：顯示日誌和控制選項。"""
    # 反轉日誌清單，讓最新的紀錄顯示在最上面
    recent_logs = log_timestamps[-20:][::-1]
    
    return render_template('index.html', 
                           logs=recent_logs,
                           current_pattern=current_pattern,
                           is_music_enabled=is_music_enabled,
                           current_mp3=current_mp3)

@app.route('/update_settings', methods=['POST'])
def update_settings():
    """處理來自網頁的設定更新請求。"""
    global current_mp3, is_music_enabled
    
    # 接收燈光模式
    new_pattern = request.form.get('pattern')
    if new_pattern in ['Solid Red', 'Flash Blue', 'Off']:
        set_led_pattern(new_pattern)
        print(f"Setting LED pattern to: {current_pattern}")

    # 接收音樂檔案
    new_mp3 = request.form.get('mp3_file')
    if new_mp3:
        # 這裡假設使用者輸入的音檔路徑是有效的
        current_mp3 = new_mp3
        print(f"Setting MP3 file to: {current_mp3}")
        
    # 接收音樂開關
    music_switch = request.form.get('music_switch')
    is_music_enabled = (music_switch == 'on')
    print(f"Music enabled: {is_music_enabled}")
    
    return jsonify({'status': 'success', 'message': 'Settings updated successfully'})

@app.route('/logs')
def get_logs():
    """提供 API 讓網頁可以定期更新日誌。"""
    # 只回傳最新的 20 筆記錄
    return jsonify(log_timestamps[-20:][::-1])

# --- 5. 程式啟動與執行緒管理 ---

if __name__ == '__main__':
    
    # 啟動 MPU6050 偵測執行緒
    mpu_thread = threading.Thread(target=mpu_detector_loop, daemon=True)
    mpu_thread.start()
    
    # 啟動 Flask 網頁伺服器 (在主執行緒中)
    # host='0.0.0.0' 讓區域網路中的其他設備可以存取
    print("Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=False) # 樹莓派上不要用 debug=True