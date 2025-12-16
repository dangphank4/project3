import time
from pca9685 import PCA9685
import numpy as np

manual_vector = np.array([0.0, 0.0, 0.0, 0.0])

# Mixing matrix 6x4
SQRT2_2 = 0.7071
mixing_matrix = np.array([
    [ 1, -1,  0, -SQRT2_2],   # T1: forward-right (CW)
    [ 1,  1,  0, +SQRT2_2],   # T2: forward-left  (CW)
    [ 1,  1,  0, -SQRT2_2],   # T3: rear-right    (CCW)
    [ 1, -1,  0, +SQRT2_2],   # T4: rear-left     (CCW)
    [ 0,  0,  1,        0],   # T5: vertical-right (CW)
    [ 0,  0,  1,        0],   # T6: vertical-left  (CCW)
])
    
# ==== KHỞI TẠO PCA9685 TRÊN I2C1 ====
pwm = PCA9685(bus=1, use_extclk=True)  
pwm.set_pwm_frequency(50)
pwm.output_enable()

def control_thruster(forward, lateral, ascend, yaw):
    manual_vector[0] = forward
    manual_vector[1] = lateral
    manual_vector[2] = ascend
    manual_vector[3] = yaw

    control_vector = manual_vector.copy()

    thrusts = mixing_matrix @ control_vector
    if np.max(np.abs(thrusts)) > 1:
        thrusts = thrusts / np.max(np.abs(thrusts))
    send_thrust_pwm(thrusts)

gain = 0.2
ESC_MAX_POWER = 400
ESC_NEUTRAL = 1500
#THRUSTER_MAP = [2, 3, 4, 1, 0, 5]
THRUSTER_MAP = [2, 5, 6, 1, 0, 7]

def send_thrust_pwm(thrusts):
    for i, t in enumerate(thrusts):
        power = int(np.clip(t, -1, 1) * gain * ESC_MAX_POWER) + ESC_NEUTRAL
        pwm.channel_set_pwm(THRUSTER_MAP[i], power)
def set_camera_tilt(Camera_Tilt):
    tiltpwm = int(1500 + Camera_Tilt * 400 / 45.0)
    pwm.channel_set_pwm(15, tiltpwm)

#light on
pwm.channel_set_pwm(9, 1300)
#time.sleep(2)
#light off
pwm.channel_set_pwm(9, 1100)

# set_camera_tilt(20)
# time.sleep(10)
# set_camera_tilt(0)
# time.sleep(2)

print("ARM...")
send_thrust_pwm([0,0,0,0,0,0])
time.sleep(1)

#pwm.channel_set_pwm(0, 1600)
#time.sleep(1)
print("RUN...")
send_thrust_pwm([0.0,0.0,0.0,0.0,-0.5,0.5])
time.sleep(30)
#send_thrust_pwm([-0.5,-0.5,0,0,0.8,0,8])
#time.sleep(10)
send_thrust_pwm([0,0,0,0,0,0])
#
time.sleep(2)

# ==== TẮT TOÀN BỘ PWM ====
pwm.output_disable()
