"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
from tools.rotations import Euler2Rotation

from message_types.msg_state import msg_state
from chap8.ekf_attitude import ekf_attitude
from chap8.ekf_position import ekf_position
from IPython.core.debugger import Pdb

class observer:
    def __init__(self, ts_control):
        # initialized estimated state message
        self.estimated_state = msg_state()
        # use alpha filters to low pass filter gyros and accels
        self.lpf_gyro_x = alpha_filter(alpha=0.5)
        self.lpf_gyro_y = alpha_filter(alpha=0.5)
        self.lpf_gyro_z = alpha_filter(alpha=0.5)
        self.lpf_accel_x = alpha_filter(alpha=0.5)
        self.lpf_accel_y = alpha_filter(alpha=0.5)
        self.lpf_accel_z = alpha_filter(alpha=0.5)
        # use alpha filters to low pass filter static and differential pressure
        self.lpf_static = alpha_filter(alpha=0.9)
        self.lpf_diff = alpha_filter(alpha=0.5)
        # ekf for phi and theta
        self.attitude_ekf = ekf_attitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = ekf_position()

    def update(self, measurements):

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        gyro_x = self.lpf_gyro_x.update(measurements.gyro_x)
        gyro_y = self.lpf_gyro_y.update(measurements.gyro_y)
        gyro_z = self.lpf_gyro_z.update(measurements.gyro_z)

        self.estimated_state.p = gyro_x - self.estimated_state.bx
        self.estimated_state.q = gyro_y - self.estimated_state.by
        self.estimated_state.r = gyro_z - self.estimated_state.bz

        measurements.accel_x = self.lpf_accel_x.update(measurements.accel_x)
        measurements.accel_y = self.lpf_accel_y.update(measurements.accel_y)
        measurements.accel_z = self.lpf_accel_z.update(measurements.accel_z)

        # invert sensor model to get altitude and airspeed
        static_p = self.lpf_static.update(measurements.static_pressure)
        diff_p = self.lpf_diff.update(measurements.diff_pressure)
        if diff_p < 0:
            diff_p = 0
        self.estimated_state.h = static_p / (MAV.rho*MAV.gravity)
        self.estimated_state.Va = np.sqrt( 2 * diff_p / MAV.rho )


        ##### EKFs #####

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(self.estimated_state, measurements)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(self.estimated_state, measurements)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state

class alpha_filter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.y*self.alpha + u*(1-self.alpha)
        return self.y
