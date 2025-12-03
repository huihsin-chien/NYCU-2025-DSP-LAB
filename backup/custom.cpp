#include <cstdio>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

static int i2c_fd = -1;
static const int MPU6050_ADDR = 0x68;
// === 校正偏移（照 Arduino 那組） ===
static const float AX_BIAS = 1.00f;
static const float AY_BIAS = 0.05f;
static const float AZ_BIAS = -0.38f;

static const float GX_BIAS = 1.11f;
static const float GY_BIAS = -4.02f;
static const float GZ_BIAS = 0.27f;

// 新增：調用 Python 腳本來控制燈條
void call_python_led_control(const char* label) {
    // 構建命令字串： python3 /path/to/led_control.py class_label
    // 假設 led_control.py 放在與 C++ 執行檔相同的目錄
    // 如果不在，請修改路徑
    char command[256];
    // 注意：這裡我用 system() 執行 Python 腳本。
    // system() 會阻塞 C++ 主程式直到 Python 腳本執行完畢。
    // 由於 Python 腳本內有 sleep，這會暫時停止 C++ 的感測器讀取。
    // 如果不希望阻塞，需要使用多執行緒或非同步調用 (更複雜)。
    snprintf(command, sizeof(command), "python3 ./source/led_control_neopixel.py %s", label);

    // 執行命令
    int ret = system(command);
    if (ret != 0) {
        std::cerr << "Python script execution failed for label: " << label << std::endl;
    }
}

// 根據分類結果決定燈效 (現在只是調用 Python)
void handleLedByLabel(const char* label) {
    if (strcmp(label, "class1") == 0) {
        call_python_led_control("class1");
    } else if (strcmp(label, "class2") == 0) {
        call_python_led_control("class2");
    } else if (strcmp(label, "class3") == 0) {
        call_python_led_control("class3");
    } else {
        // 關閉燈條
        call_python_led_control("off"); // Python 腳本中未分類時預設關閉
    }
}
// 簡單的 I2C 寫入一個暫存器
static bool i2c_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(i2c_fd, buf, 2) != 2) {
        perror("i2c write");
        return false;
    }
    return true;
}

// 從某個暫存器開始讀 size 個 bytes
static bool i2c_read_bytes(uint8_t start_reg, uint8_t *buf, size_t size) {
    if (write(i2c_fd, &start_reg, 1) != 1) {
        perror("i2c set reg");
        return false;
    }
    if (read(i2c_fd, buf, size) != (int)size) {
        perror("i2c read");
        return false;
    }
    return true;
}

// ---- IMU 初始化 (MPU6050) ----
bool imu_init() {
    const char *dev = "/dev/i2c-1";
    i2c_fd = open(dev, O_RDWR);
    if (i2c_fd < 0) {
        perror("open /dev/i2c-1");
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("ioctl I2C_SLAVE");
        return false;
    }

    // 喚醒 MPU6050：PWR_MGMT_1 (0x6B) 寫 0x00
    if (!i2c_write_register(0x6B, 0x00)) {
        printf("Failed to wake up MPU6050\n");
        return false;
    }

    // 設定加速度 ±2g（ACCEL_CONFIG 0x1C = 0x00）、陀螺儀 ±250 dps（GYRO_CONFIG 0x1B = 0x00）
    if (!i2c_write_register(0x1C, 0x00)) {
        printf("Failed to set accel config\n");
        return false;
    }
    if (!i2c_write_register(0x1B, 0x00)) {
        printf("Failed to set gyro config\n");
        return false;
    }

    printf("MPU6050 init OK\n");
    return true;
}

// ---- 讀一筆 IMU 資料：ax,ay,az (m/s^2)，gx,gy,gz (deg/s) ----
// ---- 讀一筆 IMU 資料：ax,ay,az (m/s^2)，gx,gy,gz (deg/s) ----
bool imu_read(float &ax, float &ay, float &az,
              float &gx, float &gy, float &gz) {
    uint8_t buf[14];

    if (!i2c_read_bytes(0x3B, buf, sizeof(buf))) {
        printf("Failed to read MPU6050 data\n");
        return false;
    }

    auto to_int16 = [](uint8_t high, uint8_t low) -> int16_t {
        return (int16_t)((high << 8) | low);
    };

    int16_t ax_raw = to_int16(buf[0],  buf[1]);
    int16_t ay_raw = to_int16(buf[2],  buf[3]);
    int16_t az_raw = to_int16(buf[4],  buf[5]);
    // buf[6], buf[7] 是溫度
    int16_t gx_raw = to_int16(buf[8],  buf[9]);
    int16_t gy_raw = to_int16(buf[10], buf[11]);
    int16_t gz_raw = to_int16(buf[12], buf[13]);

    const float ACCEL_SCALE = 16384.0f; // ±2g
    const float G           = 9.80665f;
    const float GYRO_SCALE  = 131.0f;   // ±250 dps

    // 1) 先把 raw 轉成實際單位
    float ax_si = (ax_raw / ACCEL_SCALE) * G;   // m/s^2
    float ay_si = (ay_raw / ACCEL_SCALE) * G;
    float az_si = (az_raw / ACCEL_SCALE) * G;

    float gx_dps = gx_raw / GYRO_SCALE;         // deg/s
    float gy_dps = gy_raw / GYRO_SCALE;
    float gz_dps = gz_raw / GYRO_SCALE;

    // 2) 套用跟 Arduino 一樣的 bias 校正
    ax = ax_si - AX_BIAS;
    ay = ay_si - AY_BIAS;
    az = az_si - AZ_BIAS;

    gx = gx_dps - GX_BIAS;
    gy = gy_dps - GY_BIAS;
    gz = gz_dps - GZ_BIAS;

    return true;
}


int main(int argc, char **argv) {
    printf("Edge Impulse IMU (MPU6050) live inferencing\n");
    static const char *stable_label = "(none)";
    static int same_count = 0;

    if (!imu_init()) {
        printf("IMU init failed\n");
        return 1;
    }
   



    // 確認模型設定
    const float sample_interval_ms = 1000.0f / EI_CLASSIFIER_FREQUENCY;
    const size_t frame_size = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // 240
    const size_t axes = 6; // ax, ay, az, gx, gy, gz

    if (frame_size % axes != 0) {
        printf("Error: frame_size (%zu) not divisible by axes (%zu)\n",
               frame_size, axes);
        return 1;
    }

    const size_t samples_per_window = frame_size / axes; // 應該是 40

    printf("Frame size = %zu, axes = %zu, samples_per_window = %zu, freq = %.2f Hz\n",
           frame_size, axes, samples_per_window, (float)EI_CLASSIFIER_FREQUENCY);

    // 可選：初始化 classifier（有些平台建議）
    run_classifier_init();

    while (true) {
        float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

        // 收集一個 window 的 IMU 資料
        for (size_t i = 0; i < samples_per_window; i++) {
            float ax, ay, az, gx, gy, gz;
            if (!imu_read(ax, ay, az, gx, gy, gz)) {
                printf("IMU read failed\n");
                return 1;
            }

            size_t base = i * axes;
            features[base + 0] = ax;
            features[base + 1] = ay;
            features[base + 2] = az;
            features[base + 3] = gx;
            features[base + 4] = gy;
            features[base + 5] = gz;

            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)sample_interval_ms));
        }

        // 丟進 Edge Impulse classifier
        signal_t signal;
        numpy::signal_from_buffer(features, frame_size, &signal);

        ei_impulse_result_t result = {};
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        if (res != EI_IMPULSE_OK) {
            printf("run_classifier failed (%d)\n", res);
            continue;
        }

       // === 1. 找出這一窗的 Top-1（原本的做法） ===
float best_val = 0.0f;
const char *best_label = "(none)";
for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > best_val) {
        best_val = result.classification[ix].value;
        best_label = result.classification[ix].label;
    }
}

// === 2. 穩定化邏輯（去抖動） ===
// stable_label & same_count 已經在 while 外宣告（很重要）
if (best_label == stable_label) {
    same_count++;
}
else {
    same_count = 1;
}

// 若連續出現 >=3 次 & 機率 > 0.8，才更新穩定結果
if (same_count >= 3 && best_val > 0.8f) {
    stable_label = best_label;
    // 根據分類結果控制 WS2812 燈效
    handleLedByLabel(stable_label);
    printf("Stable label updated to: %s\n", stable_label);
}
else{
    printf("No stable output. Light up for best_label: %s\n", best_label);
    handleLedByLabel(best_label);

}

// === 3. 印出結果 ===
printf("=== New inference ===\n");

// raw 是原始模型輸出，只供 debug
printf("Raw Top-1: %s (%.3f)\n", best_label, best_val);

// stable 是我們決定的「真正動作」
printf("Stable label: %s\n", stable_label);

printf("All classes:\n");
for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    printf("  %s: %.3f\n",
        result.classification[ix].label,
        result.classification[ix].value);
}
printf("\n");

    }
    return 0;
}
