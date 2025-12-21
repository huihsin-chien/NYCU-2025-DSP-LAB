#!/usr/bin/env python3
import time
import csv
from smbus2 import SMBus

# === 你可以改這裡 ===
I2C_BUS_NUM = 1           # 樹莓派大多是 bus 1
MPU_ADDR    = 0x68
OUT_FILE    = "imu__train_data_53.csv"
SAMPLE_HZ   = 40.0        # 取樣頻率 (Hz) —— 跟以前 Arduino 差不多
LABEL       = 0           # 這一段錄的動作類別

# === 校正偏移（照 Arduino 那組） ===
AX_BIAS = 2.6253
AY_BIAS = -0.1314
AZ_BIAS = 0.0441
GX_BIAS = -0.4750
GY_BIAS = 2.9039
GZ_BIAS = -0.8965

# === MPU6050 相關常數 ===
ACCEL_SCALE = 16384.0   # LSB/g for ±2g
GYRO_SCALE  = 131.0     # LSB/(deg/s) for ±250 dps
G = 9.80665             # m/s^2


def mpu_write_reg(bus: SMBus, reg: int, val: int):
    bus.write_byte_data(MPU_ADDR, reg, val)


def mpu_read_bytes(bus: SMBus, start_reg: int, length: int) -> bytes:
    return bus.read_i2c_block_data(MPU_ADDR, start_reg, length)


def mpu_init(bus: SMBus):
    # 喚醒：PWR_MGMT_1 = 0x00
    mpu_write_reg(bus, 0x6B, 0x00)
    time.sleep(0.1)

    # ACCEL_CONFIG = 0x00 → ±2g
    mpu_write_reg(bus, 0x1C, 0x00)

    # GYRO_CONFIG = 0x00 → ±250 dps
    mpu_write_reg(bus, 0x1B, 0x00)


def read_imu(bus: SMBus):
    """讀一筆 ax,ay,az,gx,gy,gz（已轉 SI 單位 + 校正 bias）"""

    # 從 ACCEL_XOUT_H (0x3B) 開始連續讀 14 bytes
    data = mpu_read_bytes(bus, 0x3B, 14)

    def to_int16(h, l):
        val = (h << 8) | l
        if val & 0x8000:
            val -= 0x10000
        return val

    ax_raw = to_int16(data[0],  data[1])
    ay_raw = to_int16(data[2],  data[3])
    az_raw = to_int16(data[4],  data[5])
    # data[6], data[7] = 溫度，略過
    gx_raw = to_int16(data[8],  data[9])
    gy_raw = to_int16(data[10], data[11])
    gz_raw = to_int16(data[12], data[13])

    # raw → 實際單位
    ax_si = (ax_raw / ACCEL_SCALE) * G   # m/s^2
    ay_si = (ay_raw / ACCEL_SCALE) * G
    az_si = (az_raw / ACCEL_SCALE) * G

    gx_dps = gx_raw / GYRO_SCALE        # deg/s
    gy_dps = gy_raw / GYRO_SCALE
    gz_dps = gz_raw / GYRO_SCALE

    # 套用和 Arduino 一樣的 bias 校正
    ax = ax_si - AX_BIAS
    ay = ay_si - AY_BIAS
    az = az_si - AZ_BIAS

    gx = gx_dps - GX_BIAS
    gy = gy_dps - GY_BIAS
    gz = gz_dps - GZ_BIAS

    return ax, ay, az, gx, gy, gz


def main():
    interval = 1.0 / SAMPLE_HZ
    print("準備中... 3 秒後開始記錄")
    time.sleep(3)

    with SMBus(I2C_BUS_NUM) as bus, open(OUT_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        # CSV 表頭
        writer.writerow(["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "label"])

        print(f"Using I2C bus {I2C_BUS_NUM}, addr 0x{MPU_ADDR:02X}")
        print(f"Logging to {OUT_FILE}")
        print(f"Sample rate: {SAMPLE_HZ} Hz, label = {LABEL}")

        mpu_init(bus)
        print("MPU6050 init OK, start logging... (Ctrl+C to stop)")

        t0 = time.time()

        try:
            while True:
                t = time.time()
                ax, ay, az, gx, gy, gz = read_imu(bus)

                t_ms = int((t - t0) * 1000.0)

                # 印在螢幕上
                print(f"{t_ms},{ax:.4f},{ay:.4f},{az:.4f},{gx:.4f},{gy:.4f},{gz:.4f},{LABEL}")

                # 寫到 CSV
                writer.writerow([t_ms, ax, ay, az, gx, gy, gz, LABEL])

                # 控制取樣頻率
                elapsed = time.time() - t
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopped by user (Ctrl+C).")
        except Exception as e:
            print("Error:", e)


if __name__ == "__main__":
    main()
