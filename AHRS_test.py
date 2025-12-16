import numpy as np
import math
from AHRS import AHRS
from MMC5983 import MMC5983
from ahrs.filters import Madgwick
import time

TARGET_FREQ = 200.0
LPF_ALPHA = 0.6 #Làm mượt tín hiệu
ACC_TRUST_THRESHOLD = 0.15 #Ngưỡng tin cậy của gia tốc kế
BETA_MIN = 0.04 #Giá trị beta tối thiểu cho bộ lọc Madgwick
BETA_MAX = 0.15 #Giá trị beta tối đa cho bộ lọc Madgwick
GYRO_CALIB_SAMPLES = 150 #Số mẫu để hiệu chuẩn con quay hồi chuyển
YAW_COMPL_ALPHA = 0.3 # Hệ số lọc thông thấp cho góc yaw (None để tắt)

MAG_UNIT_CONVERSION = 100000.0 
GRAVITY = 9.80665


MAG_JUMP_THRESHOLD = 150.0   # ngưỡng nhảy đột ngột của từ kế   
MAG_NORM_MIN = 0.24 * MAG_UNIT_CONVERSION   # ngưỡng nhỏ nhất của từ kế
MAG_NORM_MAX = 0.66 * MAG_UNIT_CONVERSION  # ngưỡng lớn nhất của từ kế
MAG_ALPHA = 0.65  # hệ số lọc thông thấp cho từ kế
RAD2DEG = 180.0 / math.pi

acc_lpf_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)
gyro_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
mag_prev = np.zeros(3)
beta_theory = 0.05

def safe_clamp(x, low = -1.0, high = 1.0):
    return max(low, min(high, x))


#giúp chuẩn hóa quaternion, tránh lỗi số học
def quaternion_normalize(q):
    q = np.array(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n

# Chuyển quaternion sang góc Euler (roll, pitch, yaw)
def quaternion_to_euler(q):

    qn = quaternion_normalize(q)
    w, x, y, z = qn

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)   

    sinp = 2.0 * (w * y - z * x)
    sinp = safe_clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    t2 = 2.0 * (w * z + x * y)
    t3 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t2, t3)

    return roll, pitch, yaw

def init_sensors():
    icm = AHRS(spi_bus=0, spi_dev=3)
    icm.initialize()
    print("IMU initialized")
    try:
        print("WHO_AM_I (ICM):", hex(icm.whoami()))
    except Exception:
        pass

    mag_sensor = MMC5983(spi_bus=0, spi_dev=0)
    try:
        print("WHO_AM_I (MMC5983):", hex(mag_sensor.read_id()))
    except Exception:
        pass

    madwick = Madgwick(frequency=TARGET_FREQ, beta=BETA_MIN)
    
    return icm, mag_sensor, madwick


# Hiệu chuẩn con quay hồi chuyển, đo BIAS và độ ồn
def calibrate_gyro(icm, samples = GYRO_CALIB_SAMPLES, delay=0.005):
    print("Calibrating gyro... Keep IMU stationary.")
    
    gyro_samples = []

    for i in range(samples):
        try:
            _, gyro = icm.read_accel_gyro()
            g = np.array(gyro, dtype=np.float64)

            gyro_samples.append(g)
        except Exception:
            pass
      
        time.sleep(delay)

    if len(gyro_samples) == 0:
        print("No valid gyro samples collected.")
        return (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            0.05
        )
    
    gyro_samples = np.array(gyro_samples)
    
    gyro_bias = np.mean(gyro_samples, axis=0)
    
    gyro_noise = gyro_samples - gyro_bias

    sigma_xyz = np.std(gyro_noise, axis=0)
    sigma_avg = np.mean(sigma_xyz)

    beta_theory = math.sqrt(3.0 / 4.0) * sigma_avg

    return gyro_bias, beta_theory 

# Đọc dữ liệu từ kế với lọc thông thấp và phát hiện nhảy đột ngột
def read_mag_filtered(mag_sensor):
    global mag_prev

    try:
        mx, my, mz = mag_sensor.measure()
    except Exception:
        return mag_prev, False   # giữ giá trị cũ

    # scale đơn vị
    mag_raw = np.array([
        mx * MAG_UNIT_CONVERSION,
        my * MAG_UNIT_CONVERSION,
        mz * MAG_UNIT_CONVERSION
    ], dtype=np.float64)

    # FIX TRỤC – PHẢI đồng bộ với accel + gyro
    mag_raw[1] *= -1.0
    mag_raw[2] *= -1.0

    # Norm raw 
    mag_raw_norm = np.linalg.norm(mag_raw)
    if mag_raw_norm < MAG_NORM_MIN or mag_raw_norm > MAG_NORM_MAX:
        return mag_prev, False

    # Jump detection 
    if np.linalg.norm(mag_raw - mag_prev) > MAG_JUMP_THRESHOLD:
        return mag_prev, False

    # Low-pass filter
    mag_f = MAG_ALPHA * mag_prev + (1.0 - MAG_ALPHA) * mag_raw

    # Update trạng thái
    mag_prev = mag_f.copy()

    return mag_f, True

# Đọc dữ liệu cảm biến với lọc và xử lý
def read_sensors(icm, mag_sensor, lpf_alpha=LPF_ALPHA):
    global acc_lpf_prev

    try:
        accel, gyro = icm.read_accel_gyro()
        ax, ay, az = accel
        gx, gy, gz = gyro
    except Exception:
        ax, ay, az = 0.0, 0.0, 0.0
        gx, gy, gz = 0.0, 0.0, 0.0

    acc_raw = np.array([ax, -ay, -az], dtype=np.float64)

    acc_lpf = lpf_alpha * acc_lpf_prev + (1.0 - lpf_alpha) * acc_raw
    acc_lpf_prev = acc_lpf.copy()


    mag_data, mag_valid = read_mag_filtered(mag_sensor)

    acc_data = acc_lpf
    gyr_data = np.array([gx, -gy, -gz], dtype=np.float64)


    return acc_data, gyr_data, mag_data, mag_valid
# Cập nhật phương hướng sử dụng bộ lọc Madgwick với điều chỉnh beta động
def update_orientation(icm, madgwick_filter, q, acc_data, gyr_data, mag_data, dt, mag_valid,
                       acc_trust_threshold=ACC_TRUST_THRESHOLD, beta_min=BETA_MIN, beta_max=BETA_MAX):

    acc_norm = np.linalg.norm(acc_data)

    acc_g = acc_norm / GRAVITY

    acc_deviation = abs(acc_g - 1.0)

    if acc_deviation <= acc_trust_threshold:
        beta = np.clip(
            2.0 * beta_theory,
            beta_min,
            beta_max
        )
        acc_trust = 1.0
    else:
        beta = beta_min 
        acc_trust = 0.0

    try:
        madgwick_filter.beta = beta
    except Exception:
        try:
            madgwick_filter.gain = beta
        except Exception:
            pass

    try:
        madgwick_filter.Dt = dt
    except Exception:
        try:
            madgwick_filter.frequency = 1.0 / dt if dt > 1e-6 else madgwick_filter.frequency
        except Exception:
            pass

    q_new = None
    try:
        if mag_valid and np.linalg.norm(mag_data) > 1e-9 and acc_trust > 0.0:
            q_new = madgwick_filter.updateMARG(q=q, acc=acc_data, gyr=gyr_data, mag=mag_data)
        else:
            if hasattr(madgwick_filter, "updateIMU"):
                q_new = madgwick_filter.updateIMU(q=q, acc=acc_data, gyr=gyr_data)
            elif hasattr(madgwick_filter, "update"):
                try:
                    # attempt 6-DOF signature
                    q_new = madgwick_filter.update(q=q, gyr=gyr_data, acc=acc_data)
                except TypeError:
                    # fallback: call with q,gyr,acc order
                    try:
                        q_new = madgwick_filter.update(q, gyr_data, acc_data)
                    except Exception:
                        q_new = None
            else:
                q_new = None
    except Exception as e:
        print("Madgwick update exception:", e)
        q_new = q

    if q_new is None:
        q_new = q
    q_new = quaternion_normalize(q_new)

    return q_new, beta, acc_trust

# Gửi dữ liệu AHRS trong vòng lặp chính
def sendData():
    global beta_theory

    icm, mag_sensor, madgwick_filter = init_sensors()

    gyro_bias, beta_theory = calibrate_gyro(icm, samples=GYRO_CALIB_SAMPLES)

    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    last_time = time.time()
    yaw_prev = 0.0

    target_dt = 1.0 / TARGET_FREQ if TARGET_FREQ > 0 else 0.01

    print("\nStarting AHRS loop (press Ctrl+C to stop)...\n")
    try:
        while True:
            loop_start = time.time()

            acc_data, gyr_data, mag_data, mag_valid = read_sensors(icm, mag_sensor)

            # bù bias gyro 
            gyr_data = gyr_data - gyro_bias

            now = time.time()
            dt = now - last_time if (now - last_time) > 1e-6 else target_dt
            last_time = now

            q, beta_used, acc_trust = update_orientation(
                icm, madgwick_filter, q, acc_data, gyr_data, mag_data, dt, mag_valid
            )

            roll, pitch, yaw = quaternion_to_euler(q)

            if YAW_COMPL_ALPHA is not None and mag_valid:
                yaw = YAW_COMPL_ALPHA * yaw_prev + (1.0 - YAW_COMPL_ALPHA) * yaw
                yaw_prev = yaw

            print("=" * 80)
            print(f"DT: {dt*1000:6.1f} ms | Beta: {beta_used:.4f} | AccTrust: {acc_trust}")
            print(f"Accel [g] : ax={acc_data[0]:.3f}, ay={acc_data[1]:.3f}, az={acc_data[2]:.3f} | Norm: {np.linalg.norm(acc_data):.3f}")
            gx_dps = gyr_data[0] * RAD2DEG
            gy_dps = gyr_data[1] * RAD2DEG
            gz_dps = gyr_data[2] * RAD2DEG
            print(f"Gyro  [°/s]: gx={gx_dps:.2f}, gy={gy_dps:.2f}, gz={gz_dps:.2f}")
            print(f"Mag   [µT] : mx={mag_data[0]:.3f}, my={mag_data[1]:.3f}, mz={mag_data[2]:.3f} | valid={mag_valid}")
            print(f"→ Roll:  {math.degrees(roll):7.2f}° | Pitch: {math.degrees(pitch):7.2f}° | Yaw:   {math.degrees(yaw):7.2f}°")

            loop_end = time.time()
            elapsed = loop_end - loop_start
            sleep_time = target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping AHRS loop.")
    finally:
        try:
            icm.close()
        except Exception:
            pass
        try:
            mag_sensor.close()
        except Exception:
            pass


if __name__ == "__main__":
    sendData()