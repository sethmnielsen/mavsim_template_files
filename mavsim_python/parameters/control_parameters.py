import sys
sys.path.append('..')
import numpy as np
import pickle as pkl
# import chap5.transfer_function_coef as TF

gravity = 9.8
sigma = 2.0
Va0 = 17

data = []
with open("trim.pkl", 'rb') as f:
    data = pkl.load(f)

deltas_trim = data[1]

#----------roll loop-------------
roll_kp = 1.0
roll_kd = 0.1

# ----------course loop-------------
course_kp = 1.0
course_ki = 0

# ----------sideslip loop-------------
sideslip_ki = 0
sideslip_kp = 1.0

# ----------yaw damper-------------
yaw_damper_tau_r = 1.0
yaw_damper_kp = 1.0

# ----------pitch loop-------------
pitch_kp = 1.0
pitch_kd = 0.1
K_theta_DC = 1.0

# ----------altitude loop-------------
altitude_kp = 1.0
altitude_ki = 0
altitude_zone = 1

# ---------airspeed hold using throttle---------------
airspeed_throttle_kp = 1.0
airspeed_throttle_ki = 0
