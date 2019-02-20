"""
pid_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')

class pid_control:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.01, sigma=0.05, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0
        self.y_d1 = 0.0
        self.ydot = 0.0
        self.sigma = sigma
        # gains for differentiator
        self.a1 = (2.0 * sigma - dt) / (2.0 * sigma + dt)
        self.a2 = 2.0 / (2.0 * sigma + dt)

    def update(self, y_ref, y, reset_flag=False):
        error = y_ref - y

        self.integrator += error*self.dt

        # Compute the dirty derivative
        self.ydot = (2*self.sigma-self.dt)/(2*self.sigma+self.dt)*self.ydot +\
            2/(2*self.sigma+self.dt)*(y-self.y_d1)
        self.y_d1 = y

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.ydot
        u_sat = self._saturate(u_unsat)

        # Anti-windup
        if self.ki != 0:
            self.integrator += self.dt/self.ki*(u_sat-u_unsat)
        
        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False):
        error = y_ref - y

        self.integrator += error*self.dt

        self.ydot = ydot

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.ydot
        u_sat = self._saturate(u_unsat)

        # Anti-windup
        if self.ki != 0:
            self.integrator += self.dt/self.ki*(u_sat-u_unsat)        
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

class pi_control:
    def __init__(self, kp=0.0, ki=0.0, dt=0.01, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0

    def update(self, y_ref, y):
        error = y_ref - y

        self.integrator += error*self.dt

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.integrator
        u_sat = self._saturate(u_unsat)

        # Anti-windup
        if self.ki != 0:
            self.integrator += self.dt/self.ki*(u_sat-u_unsat)
            
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

class pd_control_with_rate:
    # PD control with rate information
    # u = kp*(yref-y) - kd*ydot
    def __init__(self, kp=0.0, kd=0.0, limit=1.0):
        self.kp = kp
        self.kd = kd
        self.limit = limit

    def update(self, y_ref, y, ydot):
        error = y_ref - y

        # Compute the output
        u_unsat = self.kp*error - self.kd*ydot
        u_sat = self._saturate(u_unsat)

        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat
