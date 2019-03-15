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
from tools.tools import Euler2Rotation

from message_types.msg_state import msg_state
from IPython.core.debugger import Pdb

def jacobian(fun, x, state):
    # compute jacobian of fun with respect to x
    f = fun(x, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i] += eps
        f_eps = fun(x_eps, state)
        df = (f_eps - f) / eps
        J[:, i] = df
    return J

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

class ekf_attitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.diag([1e-5, 1e-5])
        self.Q_gyro = np.eye(3) * SENSOR.gyro_sigma**2
        self.R_accel = np.eye(3) * SENSOR.accel_sigma**2
        self.N = 10  # number of prediction step per sample
        self.xhat = np.zeros(2) # initial state: phi, theta
        self.P = np.eye(2) * 0.1
        self.Ts = SIM.ts_control/self.N

    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.phi = self.xhat[0]
        state.theta = self.xhat[1]

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        p = state.p
        q = state.q
        r = state.r
        sphi = np.sin(x[0])
        cphi = np.cos(x[0])
        ttheta = np.tan(x[1])
        _f = np.array([p + q*sphi*ttheta + r*cphi*ttheta,
                       q*cphi - r*sphi])
        return _f

    def h(self, x, state):
        # measurement model y
        p = state.p
        q = state.q
        r = state.r
        Va = state.Va
        g = MAV.gravity
        sphi = np.sin(x[0])
        cphi = np.cos(x[0])
        stheta = np.sin(x[1])
        ctheta = np.cos(x[1])
        _h = np.array([q*Va*stheta + g*stheta, 
                       r*Va*ctheta - p*Va*stheta - g*ctheta*sphi,
                       -q*Va*ctheta - g*ctheta*cphi])
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
             # propagate model
            self.xhat += self.Ts * self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # compute G matrix for gyro noise
            phi = self.xhat[0]
            theta = self.xhat[1]
            sphi = np.sin(state.phi) 
            cphi = np.cos(state.phi)
            ttheta = np.tan(state.theta)
            G = np.array([[1, sphi*ttheta, cphi*ttheta],
                          [0,        cphi,       -sphi]])

            # update P with continuous time model
            # self.P += self.Ts * (A@self.P + self.P@A.T + self.Q + G@self.Q@G.T)
            # convert to discrete time models
            A_d = np.eye(2) + A*self.Ts + A@A * (self.Ts**2)/2
            G_d = self.Ts * G
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + G_d @ self.Q_gyro @ G_d.T + self.Q*self.Ts**2
            
    def measurement_update(self, state, measurement):
        # measurement updates
        threshold = 2.0
        h = self.h(self.xhat, state)
        C = jacobian(self.h, self.xhat, state)
        y = np.array([measurement.accel_x, measurement.accel_y, measurement.accel_z])

        L = self.P@C.T@ np.linalg.inv( self.R_accel + C@self.P@C.T ) 
        I_LC = np.eye(2) - L @ C
        self.P = I_LC @ self.P @ I_LC.T + L @ self.R_accel @ L.T
        self.xhat += L @ (y - h)

class ekf_position:
    # implement continous-discrete EKF to estimate pn, pe, chi, Vg
    def __init__(self):
        self.Q = np.eye(7) * 0.01
        self.R_gps = np.diag([SENSOR.gps_n_sigma, SENSOR.gps_e_sigma, \
                            SENSOR.gps_Vg_sigma, SENSOR.gps_course_sigma])**2
        self.R_pseudo = np.eye(2) * 0.01
        self.N = 25  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.zeros(7) # n, pe, Vg, chi, wn, we, psi
        self.xhat[2] = 25  # Vg
        self.P = np.eye(7) * 0.5
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999


    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.pn = self.xhat[0]
        state.pe = self.xhat[1]
        state.Vg = self.xhat[2]
        state.chi = self.xhat[3]
        state.wn = self.xhat[4]
        state.we = self.xhat[5]
        state.psi = self.xhat[6]

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        q = state.q
        r = state.r
        theta = state.theta
        phi = state.phi
        Va = state.Va
        g = MAV.gravity

        Vg  = x[2]
        chi = x[3]
        wn  = x[4]
        we  = x[5]
        psi = x[6]
        cpsi = np.cos(psi)
        cphi = np.cos(phi)
        ctheta = np.cos(theta)
        cchi = np.cos(chi)
        spsi = np.sin(psi)
        sphi = np.sin(phi)
        schi = np.sin(chi)
        tphi = np.tan(phi)
        psi_d = q * sphi/ctheta + r * cphi/ctheta

        _f = np.array([Vg * cchi,
                       Vg * schi,
                       1/Vg*((Va*cpsi + wn)*-Va*psi_d*spsi + (Va*spsi + we)*Va*psi_d*cpsi),
                       g/Vg*tphi*np.cos(chi - psi),
                       0,
                       0,
                       psi_d])
        return _f

    def h_gps(self, x, state):
        # measurement model for gps measurements
        _h = x[:4]
        return _h

    def h_pseudo(self, x, state):
        # measurement model for wind triangale pseudo measurement
        Vg  = x[2]
        chi = x[3]
        wn  = x[4]
        we  = x[5]
        psi = x[6]
        Va = state.Va 

        _h = np.array([Va*np.cos(psi) + wn - Vg * np.cos(chi),
                       Va*np.sin(psi) + we - Vg * np.sin(chi)])
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat += self.Ts * self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
            # convert to discrete time models
            A_d = np.eye(7) + A*self.Ts + A@A * (self.Ts**2)/2
            # update P with discrete time model
            self.P = A_d@self.P@A_d.T + self.Q*self.Ts**2

    def measurement_update(self, state, measurement):
        # always update based on wind triangle pseudu measurement
        h_pseudo = self.h_pseudo(self.xhat, state)
        C = jacobian(self.h_pseudo, self.xhat, state)
        y = np.array([0, 0]) # wn and we
        L = self.P@C.T@np.linalg.inv(self.R_pseudo + C@self.P@C.T)
        
        I_LC = np.eye(7) - L @ C
        self.P = I_LC @ self.P @ I_LC.T + L @ self.R_pseudo @ L.T
        self.xhat += L @ (y - h_pseudo)  # How does this only affect wn and we?

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h_gps = self.h_gps(self.xhat, state)
            C = jacobian(self.h_gps, self.xhat, state)
            y = np.array([measurement.gps_n, 
                          measurement.gps_e, 
                          measurement.gps_Vg, 
                          measurement.gps_course])
            L = self.P @ C.T @ np.linalg.inv(self.R_gps + C @ self.P @ C.T)

            I_LC = np.eye(7) - L @ C
            self.P = I_LC @ self.P @ I_LC.T + L @ self.R_gps @ L.T
            y[3] = self.wrap(y[3], h_gps[3])
            self.xhat += L @ (y - h_gps)  # How does this only affect pn,pe,Vg,chi?

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

    def wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c