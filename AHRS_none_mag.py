import numpy as np
import math
from AHRS import AHRS
from MMC5983 import MMC5983
from ahrs.filters import Madgwick
import time
from collections import deque

# ===================== CONFIG =====================
TARGET_FREQ = 200.0

LPF_ALPHA = 0.6
ACC_TRUST_THRESHOLD = 0.15

BETA_MIN = 0.04
BETA_MAX = 0.15
GYRO_CALIB_SAMPLES = 150

YAW_COMPL_ALPHA = 0.98   # yaw gyro 98%, mag 2%

MAG_UNIT_CONVERSION = 100000.0
GRAVITY = 9.80665

MAG_JUMP_THRESHOLD = 150.0
MAG_NORM_MIN = 0.24 * MAG_UNIT_CONVERSION
MAG_NORM_MAX = 0.66 * MAG_UNIT_CONVERSION
MAG_ALPHA = 0.65

RAD2DEG = 180.0 / math.pi

acc_lpf_prev = np.zeros(3)
gyro_bias = np.zeros(3)
mag_prev = np.zeros(3)
beta_theory = 0.05


# ====== THAM SỐ SIẾT CHẶT ======
MAG_VAR_THRESHOLD = 60.0              # độ ổn định mag
ACC_TRUST_G = 0.08                    # |acc|-1g
GYRO_Z_MAX = math.radians(30.0)       # 30 deg/s
YAW_ERR_MAX = math.radians(20.0)      # mag chỉ chỉnh nhỏ

MAG_HIST_LEN = 10
mag_hist = deque(maxlen=MAG_HIST_LEN)


def safe_clamp(x, low=-1.0, high=1.0):
    return max(low, min(high, x))

def quaternion_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1, 0, 0, 0], dtype=np.float64)

def quaternion_to_euler(q):
    w, x, y, z = quaternion_normalize(q)

    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = safe_clamp(2*(w*y - z*x))
    pitch = math.asin(sinp)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    return roll, pitch, yaw

def yaw_from_mag(acc, mag):
    acc_n = np.linalg.norm(acc)
    mag_n = np.linalg.norm(mag)
    if acc_n < 1e-6 or mag_n < 1e-6:
        return None

    ax, ay, az = acc / acc_n
    mx, my, mz = mag / mag_n

    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )

    return math.atan2(-my2, mx2)

def init_sensors():
    icm = AHRS(spi_bus=0, spi_dev=3)
    icm.initialize()

    mag = MMC5983(spi_bus=0, spi_dev=0)

    madgwick = Madgwick(frequency=TARGET_FREQ, beta=BETA_MIN)
    return icm, mag, madgwick

def calibrate_gyro(icm, samples=GYRO_CALIB_SAMPLES):
    print("Calibrating gyro...")
    data = []

    for _ in range(samples):
        _, g = icm.read_accel_gyro()
        data.append(g)
        time.sleep(0.005)

    data = np.array(data)
    bias = np.mean(data, axis=0)
    noise = data - bias
    sigma = np.mean(np.std(noise, axis=0))
    beta = math.sqrt(3.0 / 4.0) * sigma

    return bias, beta


def angle_diff(a, b):
    """Hiệu góc trong [-pi, pi]"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def read_mag_filtered(mag_sensor, acc, gyr, yaw_gyro):
    """
    return: mag_filtered, mag_valid
    """

    global mag_prev

    try:
        mx, my, mz = mag_sensor.measure()
    except:
        return mag_prev, False

    mag = np.array([
        mx * MAG_UNIT_CONVERSION,
        -my * MAG_UNIT_CONVERSION,
        -mz * MAG_UNIT_CONVERSION
    ])

    norm = np.linalg.norm(mag)
    if norm < MAG_NORM_MIN or norm > MAG_NORM_MAX:
        return mag_prev, False

    if np.linalg.norm(mag - mag_prev) > MAG_JUMP_THRESHOLD:
        return mag_prev, False

    mag_f = MAG_ALPHA * mag_prev + (1 - MAG_ALPHA) * mag
    mag_prev = mag_f.copy()

    mag_hist.append(mag_f)
    if len(mag_hist) >= 5:
        mag_var = np.mean(np.var(mag_hist, axis=0))
        if mag_var > MAG_VAR_THRESHOLD:
            return mag_prev, False

    acc_norm = np.linalg.norm(acc) / GRAVITY
    if abs(acc_norm - 1.0) > ACC_TRUST_G:
        return mag_prev, False

    if abs(gyr[2]) > GYRO_Z_MAX:
        return mag_prev, False

    yaw_mag = yaw_from_mag(acc, mag_f)
    if yaw_mag is None:
        return mag_prev, False

    if abs(angle_diff(yaw_mag, yaw_gyro)) > YAW_ERR_MAX:
        return mag_prev, False

    return mag_f, True

def read_sensors(icm, mag_sensor):
    global acc_lpf_prev

    accel, gyro = icm.read_accel_gyro()
    ax, ay, az = accel
    gx, gy, gz = gyro

    acc = np.array([ax, -ay, -az])
    acc_lpf = LPF_ALPHA * acc_lpf_prev + (1 - LPF_ALPHA) * acc
    acc_lpf_prev = acc_lpf.copy()

    gyr = np.array([gx, -gy, -gz])

    mag, mag_valid = read_mag_filtered(mag_sensor)

    return acc_lpf, gyr, mag, mag_valid

def update_orientation(madgwick, q, acc, gyr, dt):
    madgwick.Dt = dt
    q = madgwick.updateIMU(q=q, acc=acc, gyr=gyr)
    return quaternion_normalize(q)

def sendData():
    global beta_theory

    icm, mag_sensor, madgwick = init_sensors()
    gyro_bias, beta_theory = calibrate_gyro(icm)

    q = np.array([1.0, 0.0, 0.0, 0.0])
    yaw_gyro = 0.0

    last_time = time.time()
    target_dt = 1.0 / TARGET_FREQ

    print("AHRS running...\n")

    try:
        while True:
            acc, gyr, mag, mag_valid = read_sensors(icm, mag_sensor)
            gyr -= gyro_bias

            now = time.time()
            dt = now - last_time
            last_time = now

            q = update_orientation(madgwick, q, acc, gyr, dt)

            roll, pitch, _ = quaternion_to_euler(q)

            # ===== YAW =====
            yaw_gyro += gyr[2] * dt
            yaw = yaw_gyro

            if mag_valid:
                yaw_mag = yaw_from_mag(acc, mag)
                if yaw_mag is not None:
                    yaw = YAW_COMPL_ALPHA * yaw_gyro + (1 - YAW_COMPL_ALPHA) * yaw_mag
                    yaw_gyro = yaw

            print(f"Roll={math.degrees(roll):7.2f}° | "
                  f"Pitch={math.degrees(pitch):7.2f}° | "
                  f"Yaw={math.degrees(yaw):7.2f}°")

            sleep = target_dt - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("Stopped")

    finally:
        icm.close()
        mag_sensor.close()

if __name__ == "__main__":
    sendData()
