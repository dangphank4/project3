import numpy as np
import math
from AHRS import AHRS
from MMC5983 import MMC5983
from ahrs.filters import Madgwick
import time

TARGET_FREQ = 200.0
LPF_ALPHA = 0.6
ACC_TRUST_THRESHOLD = 0.15
BETA_MIN = 0.2
BETA_MAX = 0.6
GYRO_CALIB_SAMPLES = 150
YAW_COMPL_ALPHA = None

MAG_UNIT_CONVERSION = 100000.0 
GRAVITY = 9.80665

acc_lpf_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)
gyro_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)


def safe_clamp(x, low = -1.0, high = 1.0):
    return max(low, min(high, x))

def quaternion_normalize(q):
    q = np.array(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)
    return q / n

def quaternion_to_euler(q):
    w, x, y, z = q

    qn = quaternion_normalize([w, x, y, z])
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

def calibrate_gyro(icm, samples=GYRO_CALIB_SAMPLES, delay=0.005):
    print("Calibrating gyro... Keep IMU stationary.")
    gx_sum = 0.0
    gy_sum = 0.0
    gz_sum = 0.0
    valid = 0
    for i in range(samples):
        try:
            _, gyro = icm.read_accel_gyro()
        except Exception:
            gyro = (0.0, 0.0, 0.0)
        gx_sum += gyro[0]
        gy_sum += gyro[1]
        gz_sum += gyro[2]
        valid += 1
        time.sleep(delay)
    if valid == 0:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)
    bias = np.array([gx_sum / valid, gy_sum / valid, gz_sum / valid], dtype=np.float64)
    print(f"Gyro bias: {bias}")
    return bias

def read_sensors(icm, mag_sensor, lpf_alpha=LPF_ALPHA):
    global acc_lpf_prev

    try:
        accel, gyro = icm.read_accel_gyro()
        ax, ay, az = accel
        gx, gy, gz = gyro
    except Exception:
        ax, ay, az = 0.0, 0.0, 0.0
        gx, gy, gz = 0.0, 0.0, 0.0

    acc_raw = np.array([ax, ay, az], dtype=np.float64)

    acc_lpf = lpf_alpha * acc_lpf_prev + (1.0 - lpf_alpha) * acc_raw
    acc_lpf_prev = acc_lpf.copy()

    mag = None
    try:
        mag = mag_sensor.measure()
    except Exception:
        mag = None

    if mag:
        try:
            mx, my, mz = [v * MAG_UNIT_CONVERSION for v in mag]
            mag_valid = True
        except Exception:
            mx, my, mz = 0.0, 0.0, 0.0
            mag_valid = False
    else:
        mx, my, mz = 0.0, 0.0, 0.0
        mag_valid = False

    acc_data = acc_lpf
    gyr_data = np.array([gx, gy, gz], dtype=np.float64)
    mag_data = np.array([mx, my, mz], dtype=np.float64)


    raw = (ax, ay, az, gx, gy, gz, mx, my, mz)

    return acc_data, gyr_data, mag_data, raw, mag_valid

def update_orientation(icm, madgwick_filter, q, acc_data, gyr_data, mag_data, dt, mag_valid,
                       acc_trust_threshold=ACC_TRUST_THRESHOLD, beta_min=BETA_MIN, beta_max=BETA_MAX):
    """
    Adaptive Madgwick update:
      - nếu acc gần 1g -> tăng beta (acc correction mạnh)
      - nếu mag có sẵn và đáng tin -> dùng updateMARG (9-DOF)
      - nếu mag không đáng tin -> dùng updateIMU (6-DOF)
    """

    acc_norm = np.linalg.norm(acc_data)

    acc_g = acc_norm / GRAVITY

    acc_deviation = abs(acc_g - 1.0)

    if acc_deviation <= acc_trust_threshold:
        beta = beta_max
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

def compute_orientation(icm, mag_sensor, madgwick_filter,
             q, gyro_bias, last_time, YAW_COMPL_ALPHA, yaw_prev, target_dt):
    loop_start = time.time()

    # Đọc cảm biến
    acc_data, gyr_data, mag_data, raw, mag_valid = read_sensors(icm, mag_sensor)

    # Bù bias gyro
    gyr_data = gyr_data - gyro_bias

    # Tính dt
    now = time.time()
    dt = now - last_time if (now - last_time) > 1e-6 else target_dt
    last_time = now

    # Cập nhật orientation
    q, beta_used, acc_trust = update_orientation(
        icm, madgwick_filter, q, acc_data, gyr_data, mag_data, dt, mag_valid
    )

    # Euler angles
    roll, pitch, yaw = quaternion_to_euler(q)

    # Yaw complementary filter
    if YAW_COMPL_ALPHA is not None and mag_valid:
        yaw = YAW_COMPL_ALPHA * yaw_prev + (1.0 - YAW_COMPL_ALPHA) * yaw
        yaw_prev = yaw

    # Debug print
    ax, ay, az, gx, gy, gz, mx, my, mz = raw
    print("=" * 80)
    print(f"DT: {dt*1000:6.1f} ms | Beta: {beta_used:.4f} | AccTrust: {acc_trust}")
    print(f"Accel [g] : ax={ax:.3f}, ay={ay:.3f}, az={az:.3f} | Norm: {np.linalg.norm(acc_data):.3f}")
    print(f"Gyro  [°/s]: gx={gyr_data[0]:.2f}, gy={gyr_data[1]:.2f}, gz={gyr_data[2]:.2f}")
    print(f"Mag   [µT] : mx={mx:.3f}, my={my:.3f}, mz={mz:.3f} | valid={mag_valid}")
    #print(f"→ Roll:  {math.degrees(roll):7.2f}° | Pitch: {math.degrees(pitch):7.2f}° | Yaw:   {math.degrees(yaw):7.2f}°")

    # Tính thời gian chạy 1 vòng
    loop_end = time.time()
    elapsed = loop_end - loop_start
    sleep_time = target_dt - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

    return q, last_time, yaw_prev, roll, pitch, yaw

def run_loop(icm, mag_sensor, madgwick_filter,
             q, gyro_bias, YAW_COMPL_ALPHA, target_dt):

    last_time = time.time()
    yaw_prev = 0.0

    try:
        while True:
            q, last_time, yaw_prev, roll, pitch, yaw = compute_orientation(
                icm, mag_sensor, madgwick_filter,
                q, gyro_bias, last_time,
                YAW_COMPL_ALPHA, yaw_prev, target_dt
            )

            print("Output: "+"=" * 80)
            print(f"Roll={math.degrees(roll):.2f}°, "
                  f"Pitch={math.degrees(pitch):.2f}°, "
                  f"Yaw={math.degrees(yaw):.2f}°")

            time.sleep(target_dt)

    except KeyboardInterrupt:
        print("\nStopping AHRS loop.")


    finally:
        try:
            icm.close()
        except:
            pass
        try:
            mag_sensor.close()
        except:
            pass


if __name__ == "__main__":

    icm, mag_sensor, madgwick_filter = init_sensors()
    gyro_bias = calibrate_gyro(icm, samples=GYRO_CALIB_SAMPLES, delay=0.005)
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    target_dt = 1.0 / TARGET_FREQ if TARGET_FREQ > 0 else 0.01

    run_loop(
        icm=icm,
        mag_sensor=mag_sensor,
        madgwick_filter=madgwick_filter,
        q=q,
        gyro_bias=gyro_bias,
        YAW_COMPL_ALPHA=YAW_COMPL_ALPHA,
        target_dt=target_dt
    )
