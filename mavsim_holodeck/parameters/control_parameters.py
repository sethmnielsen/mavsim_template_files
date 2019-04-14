import sys
sys.path.append('..')
import numpy as np
import pickle as pkl
import parameters.simulation_parameters as SP

ts_control = SP.ts_control

gravity = 9.8
sigma = 0.05
Va0 = 25

# data = []
# with open("trim.pkl", 'rb') as f:
    # data = pkl.load(f)

# deltas_trim = data[1]

#----------roll loop-------------
roll_kp = 0.4743
roll_kd = 0.1584

# ----------course loop-------------
course_kp = 1.25
course_ki = 0.2

# ----------sideslip loop-------------
sideslip_ki = 0
sideslip_kp = 0.1

# ----------yaw damper-------------
yaw_damper_tau_r = 0.05
yaw_damper_kp = 0.5

# ----------pitch loop-------------
pitch_kp = -4.5
pitch_kd = -0.7
K_theta_DC = 1.0

# ----------altitude loop-------------
altitude_kp = 0.05
altitude_ki = 0.011
altitude_zone = 2.0

# ---------airspeed hold using throttle---------------
airspeed_throttle_kp = 1.25
airspeed_throttle_ki = 0.35