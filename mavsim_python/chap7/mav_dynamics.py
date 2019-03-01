"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/16/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state
from message_types.msg_sensors import msg_sensors

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.rotations import Quaternion2Rotation, Quaternion2Euler
from math import asin, exp, acos

class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        self._state = np.array([MAV.pn0,   # (0)
                                MAV.pe0,   # (1)
                                MAV.pd0,   # (2)
                                MAV.u0,    # (3)
                                MAV.v0,    # (4)
                                MAV.w0,    # (5)
                                MAV.e0,    # (6)
                                MAV.e1,    # (7)
                                MAV.e2,    # (8)
                                MAV.e3,    # (9)
                                MAV.p0,    # (10)
                                MAV.q0,    # (11)
                                MAV.r0])   # (12)
        self._forces = np.zeros(3)
        self.R_vb = Quaternion2Rotation(self._state[6:10])  # Rotation vehicle->body
        self._update_velocity_data()

        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        self.true_state = msg_state()
        self.sensors = msg_sensors()

        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.


    ###################################
    # public functions
    def update_state(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''

        self.R_vb = Quaternion2Rotation(self._state[6:10])

        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state[6]
        e1 = self._state[7]
        e2 = self._state[8]
        e3 = self._state[9]
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6] = self._state[6]/normE
        self._state[7] = self._state[7]/normE
        self._state[8] = self._state[8]/normE
        self._state[9] = self._state[9]/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_true_state()


    def update_sensors(self):
        "Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS"
        self.sensors.gyro_x =
        self.sensors.gyro_y =
        self.sensors.gyro_z =
        self.sensors.accel_x =
        self.sensors.accel_y =
        self.sensors.accel_z =
        self.sensors.static_pressure =
        self.sensors.diff_pressure =
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n =
            self._gps_eta_e =
            self._gps_eta_h =
            self.sensors.gps_n =
            self.sensors.gps_e =
            self.sensors.gps_h =
            self.sensors.gps_Vg =
            self.sensors.gps_course =
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # pn = state[0]
        # pe = state[1]
        # pd = state[2]
        u  = state[3]
        v  = state[4]
        w  = state[5]
        e0 = state[6]
        e1 = state[7]
        e2 = state[8]
        e3 = state[9]
        p  = state[10]
        q  = state[11]
        r  = state[12]
        #   extract forces/moments
        fx = forces_moments[0]
        fy = forces_moments[1]
        fz = forces_moments[2]
        l  = forces_moments[3]
        m  = forces_moments[4]
        n  = forces_moments[5]

        # position kinematics
        pn_dot, pe_dot, pd_dot = self.R_vb @ np.array([u, v, w])

        # position dynamics
        vec_pos = np.array([r*v - q*w, p*w - r*u, q*u - p*v])
        u_dot, v_dot, w_dot = vec_pos + 1/MAV.mass * np.array([fx, fy, fz])

        # rotational kinematics
        mat_rot = np.array([[0, -p, -q, -r],
                            [p, 0, r, -q],
                            [q, -r, 0, p],
                            [r, q, -p, 0]])
        e0_dot, e1_dot, e2_dot, e3_dot = 0.5*mat_rot @ np.array([e0,e1,e2,e3])

        # rotatonal dynamics
        G1 = MAV.gamma1
        G2 = MAV.gamma2
        G3 = MAV.gamma3
        G4 = MAV.gamma4
        G5 = MAV.gamma5
        G6 = MAV.gamma6
        G7 = MAV.gamma7
        G8 = MAV.gamma8

        vec_rot = np.array([G1*p*q - G2*q*r, G5*p*r - G6*(p**2-r**2), G7*p*q - G1*q*r])
        vec_rot2 = np.array([G3*l + G4*n, m/MAV.Jy, G4*l + G8*n])

        p_dot, q_dot, r_dot = vec_rot + vec_rot2

        # collect the derivative of the states
        x_dot = np.array([pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                          e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot])

        return x_dot


    def _update_velocity_data(self, wind=np.zeros(6)):
        # compute airspeed
        V_wb = self.R_vb @ wind[:3] + wind[3:]
        V_ab = self._state[3:6] - V_wb
        self._Va = np.linalg.norm(V_ab) 

        # compute angle of attack
        self._alpha = np.arctan2(V_ab[2],V_ab[0])

        # compute sideslip angle
        self._beta = asin(V_ab[1]/self._Va)

    def _forces_moments(self, delta):

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t):
        return T_p, Q_p


    def _update_true_state(self):
