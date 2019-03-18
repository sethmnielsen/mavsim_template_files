import sys
import numpy as np
sys.path.append('..')
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
from tools.tools import jacobian

class ekf_attitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.diag([1e-6, 1e-6])
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