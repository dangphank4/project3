from MMC5983 import MMC5983
from ICM20602 import ICM20602
import time

# Khởi tạo cảm biến
imu = ICM20602(spi_bus=0, spi_dev=3)
imu.initialize()
mmc = MMC5983(spi_bus=0, spi_dev=0)
mmc.reset()

# Đọc dữ liệu liên tục
try:
    while True:
        accel, gyro = imu.read_accel_gyro()
        ax, ay, az = [round(v, 3) for v in accel]
        gx, gy, gz = [round(v, 2) for v in gyro]
        pitch, roll = imu.compute_pitch_roll(ax, ay, az)

        print(f"Accel [g]: x={ax}, y={ay}, z={az} | Pitch: {round(pitch,2)}°, Roll: {round(roll,2)}°")
        print(f"Gyro  [dps]: x={gx}, y={gy}, z={gz}")

        mag = mmc.measure()
        if mag:
            print("Magnetometer [Gauss]:", mag)
            heading = mmc.calculate_heading(mag, pitch=pitch, roll=roll)  # Thêm pitch/roll nếu có IMU
            print("Heading [deg]:", round(heading, 2))
        else:
            print("⚠️ Không có dữ liệu")

        time.sleep(0.1)  # đọc 10 Hz
except KeyboardInterrupt:
    print("⛔ Kết thúc.")
finally:
    mmc.close()
