from ICM20602 import ICM20602
import time
import math

def main():
    imu = ICM20602(spi_bus=0, spi_dev=3)
    imu.initialize()
    print("WHO_AM_I:", hex(imu.whoami()))

    try:
        while True:
            accel, gyro = imu.read_accel_gyro()
            ax, ay, az = [v * 9.81 for v in accel]  # g → m/s²
            gx, gy, gz = [(gyro[0]) * math.pi / 180,
                          (gyro[1]) * math.pi / 180,
                          (gyro[2]) * math.pi / 180]  # °/s → rad/s
            pitch, roll = imu.compute_pitch_roll(ax, ay, az)

            print(f"Pitch: {round(pitch,2)}°, Roll: {round(roll,2)}°")
            print(f"Accel [m/s²]: x={ax:7.3f}, y={ay:7.3f}, z={az:7.3f}")
            print(f"Gyro  [rad/s]: x={gx:7.3f}, y={gy:7.3f}, z={gz:7.3f}")
            time.sleep(0.1)  # ~20 Hz

    except KeyboardInterrupt:
        print("\n Dừng đọc dữ liệu IMU.")
    finally:
        imu.close()


if __name__ == "__main__":
    main()
