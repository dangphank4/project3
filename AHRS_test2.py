import time
import numpy as np
import math
from MMC5983 import MMC5983
from ahrs.filters import Madgwick
from AHRS import AHRS

def main():
    icm = AHRS(spi_bus=0, spi_dev=3)
    icm.initialize()
    icm.calibrate_gyro()
    print("✅ IMU initialized")
    print("WHO_AM_I:", hex(icm.whoami()))
    
    mag_sensor = MMC5983(spi_bus=0, spi_dev=0)
    print("WHO_AM_I (MMC5983):", hex(mag_sensor.read_id()))

    icm.madgwick_filter.beta = 0.05  # giảm noise drift
    q = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion khởi tạo
    last_time = time.time()  # để tính dt

    print("=== Bắt đầu đọc dữ liệu IMU ===")
    try:
        while True:
            accel, gyro = icm.read_accel_gyro()
            mag = icm.read_mag()
            ax, ay, az = accel
            gx, gy, gz = gyro
            mx,my,mz = mag

            acc_data = np.array([-ax, ay, az], dtype=np.float64)
            gyr_data = np.array([gx, gy, gz], dtype=np.float64)
            mag_data = np.array([mx, my, mz], dtype=np.float64)

            now = time.time()
            dt = now - last_time
            last_time = now

            icm.madgwick_filter.Dt = dt
            q = icm.madgwick_filter.updateMARG(
                q=q,
                acc=acc_data,
                gyr=gyr_data,
                mag=mag_data
            )

            if q is not None:
                roll, pitch, yaw = icm.quaternion_to_euler(q)
                print(f"Pitch: {math.degrees(pitch):6.2f}° | "
                      f"Roll: {math.degrees(roll):6.2f}° | "
                      f"Yaw: {math.degrees(yaw):6.2f}°")
            else:
                print("⚠️ Madgwick update failed")

            print(f"Accel [m/s²]: x={ax:6.3f} y={ay:6.3f} z={az:6.3f}")
            print(f"Gyro  [rad/s]: x={gx:6.3f} y={gy:6.3f} z={gz:6.3f}")
            print(f"Mag   [nT]:    x={mx:6.1f} y={my:6.1f} z={mz:6.1f}")
            print("-" * 60)
            
            time.sleep(0.05)  # ~20Hz

    except KeyboardInterrupt:
        print("\n🛑 Dừng đọc dữ liệu IMU.")
    finally:
        icm.close()
        mag_sensor.close()

if __name__ == "__main__":
    main()
