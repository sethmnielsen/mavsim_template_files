"""
pid_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')

class control_base:
    def __init__(self):
        pass
    
    def _saturate(self, u):
        if self.lower_lim is not None:
            lower_lim = self.lower_lim
        else:
            lower_lim = -self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= lower_lim:
            u_sat = lower_lim
        else:
            u_sat = u
        return u_sat
    
    def _antiwindup(self, u_unsat, u_sat):
        if self.ki != 0:
            self.integrator += self.Ts/self.ki*(u_sat-u_unsat)

    def _integrateError(self, e):
        self.integrator += self.Ts / 2.0 * (e + self.error_delay_1)
        self.error_delay_1 = e


class pid_control(control_base):
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit=1.0, lower_lim=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit = limit
        self.lower_lim = lower_lim
        self.integrator = 0.0
        self.error_delay_1 = 0.0
        self.y_d1 = 0.0
        self.ydot = 0.0
        self.sigma = sigma
        # gains for differentiator
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)

    def update(self, y_ref, y, reset_flag=False):
        error = y_ref - y

        self._integrateError(error)

        # Compute the dirty derivative
        self.ydot = (2*self.sigma-self.Ts)/(2*self.sigma+self.Ts)*self.ydot +\
            2/(2*self.sigma+self.Ts)*(y-self.y_d1)
        self.y_d1 = y

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.ydot
        u_sat = self._saturate(u_unsat)

        self._antiwindup(u_unsat, u_sat)

        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False):
        error = y_ref - y

        self._integrateError(error)

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator - self.ydot
        u_sat = self._saturate(u_unsat)

        self._antiwindup(u_unsat, u_sat)

        return u_sat

class pi_control(control_base):
    def __init__(self, kp=0.0, ki=0.0, Ts=0.01, limit=1.0, lower_lim=None):
        self.kp = kp
        self.ki = ki
        self.Ts = Ts
        self.limit = limit
        self.lower_lim = lower_lim
        self.integrator = 0.0
        self.error_delay_1 = 0.0

    def update(self, y_ref, y, reset_flag=False):
        error = y_ref - y
        if reset_flag:
            while(error > np.pi):
                error -= 2 * np.pi
            while(error <= -np.pi):
                error += 2 * np.pi

        self._integrateError(error)

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator
        u_sat = self._saturate(u_unsat)

        self._antiwindup(u_unsat, u_sat)

        return u_sat

class pd_control_with_rate(control_base):
    # PD control with rate information
    # u = kp*(yref-y) - kd*ydot
    def __init__(self, kp=0.0, kd=0.0, limit=1.0, lower_lim=None):
        self.kp = kp
        self.kd = kd
        self.limit = limit
        self.lower_lim = lower_lim

    def update(self, y_ref, y, ydot, reset_flag=False):
        error = y_ref - y

        # Compute the output
        u_unsat = self.kp*error - self.kd*ydot
        u_sat = self._saturate(u_unsat)

        return u_sat