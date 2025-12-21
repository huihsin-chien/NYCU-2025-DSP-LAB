#!/usr/bin/env python3
import time
from smbus2 import SMBus

I2C_BUS_NUM = 1
MPU_ADDR    = 0x68

ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0
G           = 9.80665

N_SAMPLES   = 500   # 收 500 筆平均

def mpu_write_reg(bus, reg, val):
    bus.write_byte_data(MPU_ADDR, reg, val)

def mpu_read_bytes(bus, start_reg, length):
    return bus.read_i2c_block_data(MPU_ADDR, start_reg, length)

def to_int16(h, l):
    v = (h << 8) | l
    if v & 0x8000:
        v -= 0x10000
    return v

def mpu_init(bus):
    # 喚醒 + 設定 ±2g / ±250 dps
    mpu_write_reg(bus, 0x6B, 0x00)   # PWR_MGMT_1 = 0, 喚醒
    time.sleep(0.1)
    mpu_write_reg(bus, 0x1C, 0x00)   # ACCEL ±2g
    mpu_write_reg(bus, 0x1B, 0x00)   # GYRO ±250 dps

def read_once(bus):
    data = mpu_read_bytes(bus, 0x3B, 14)

    ax_raw = to_int16(data[0],  data[1])
    ay_raw = to_int16(data[2],  data[3])
    az_raw = to_int16(data[4],  data[5])
    gx_raw = to_int16(data[8],  data[9])
    gy_raw = to_int16(data[10], data[11])
    gz_raw = to_int16(data[12], data[13])

    ax_si = (ax_raw / ACCEL_SCALE) * G
    ay_si = (ay_raw / ACCEL_SCALE) * G
    az_si = (az_raw / ACCEL_SCALE) * G

    gx_dps = gx_raw / GYRO_SCALE
    gy_dps = gy_raw / GYRO_SCALE
    gz_dps = gz_raw / GYRO_SCALE

    return ax_si, ay_si, az_si, gx_dps, gy_dps, gz_dps

def main():
    print("請把 IMU 放好，**Z 軸朝下**，保持完全靜止...")
    time.sleep(3)

    with SMBus(I2C_BUS_NUM) as bus:
        mpu_init(bus)

        sum_ax = sum_ay = sum_az = 0.0
        sum_gx = sum_gy = sum_gz = 0.0

        for i in range(N_SAMPLES):
            ax, ay, az, gx, gy, gz = read_once(bus)
            sum_ax += ax; sum_ay += ay; sum_az += az
            sum_gx += gx; sum_gy += gy; sum_gz += gz
            time.sleep(0.01)

        mean_ax = sum_ax / N_SAMPLES
        mean_ay = sum_ay / N_SAMPLES
        mean_az = sum_az / N_SAMPLES

        mean_gx = sum_gx / N_SAMPLES
        mean_gy = sum_gy / N_SAMPLES
        mean_gz = sum_gz / N_SAMPLES

        print("\n=== Mean (measured, Z-down) ===")
        print(f"ax = {mean_ax:.4f}")
        print(f"ay = {mean_ay:.4f}")
        print(f"az = {mean_az:.4f}")
        print(f"gx = {mean_gx:.4f}")
        print(f"gy = {mean_gy:.4f}")
        print(f"gz = {mean_gz:.4f}")

        # ✅ 理想值：Z 軸朝下、靜止 → az = -9.80665
        ax_true, ay_true, az_true = 0.0, 0.0, -9.80665
        gx_true, gy_true, gz_true = 0.0, 0.0, 0.0

        AX_BIAS = mean_ax - ax_true
        AY_BIAS = mean_ay - ay_true
        AZ_BIAS = mean_az - az_true

        GX_BIAS = mean_gx - gx_true
        GY_BIAS = mean_gy - gy_true
        GZ_BIAS = mean_gz - gz_true

        print("\n=== 建議的 bias 常數（貼回程式用） ===")
        print(f"AX_BIAS = {AX_BIAS:.4f}")
        print(f"AY_BIAS = {AY_BIAS:.4f}")
        print(f"AZ_BIAS = {AZ_BIAS:.4f}")
        print(f"GX_BIAS = {GX_BIAS:.4f}")
        print(f"GY_BIAS = {GY_BIAS:.4f}")
        print(f"GZ_BIAS = {GZ_BIAS:.4f}")

if __name__ == "__main__":
    main()
