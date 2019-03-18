import sys
import numpy as np
sys.path.append('..')
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
from tools.tools import jacobian

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