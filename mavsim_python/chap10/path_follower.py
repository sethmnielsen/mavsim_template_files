import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from message_types.msg_autopilot import msg_autopilot
from message_types.msg_path import msg_path
from message_types.msg_state import msg_state

class path_follower:
    def __init__(self):
        self.chi_inf = np.radians(80)   # approach angle for large distance from straight-line path
        self.k_path = 0.02   # proportional gain for straight-line path following
        self.k_orbit = 2.5   # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = msg_autopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.flag=='line':
            self._follow_straight_line(path, state)
        elif path.flag=='orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path=msg_path(), state=msg_state()):
        q = np.copy(path.line_direction)
        chi_q = atan2(q[1], q[0])
        chi_q = self._wrap(chi_q, state.chi)

        Rp_i = np.array([[cos(chi_q), sin(chi_q), 0],
                         [-sin(chi_q), cos(chi_q), 0],
                         [0, 0, 1]])

        r = path.line_origin
        p = np.array([state.pn, state.pe, -state.h])

        # chi_c: Course direction of path
        ei_p = p - r
        ep = Rp_i @ ei_p
        chi_d = self.chi_inf * (2/np.pi) * atan(self.k_path * ep[1])

        chi_c = chi_q - chi_d

        # h_c: Altitude
        product = np.cross( q, np.array([0,0,1]) )
        n = product / np.linalg.norm(product)
        s = ei_p - (ei_p @ n) * n

        hc = -r[2] - np.sqrt( s[0]**2 + s[1]**2 ) / np.sqrt( q[0]**2 + q[1]**2 ) * q[2]
        
        self.autopilot_commands.airspeed_command = path.airspeed
        self.autopilot_commands.course_command = chi_c
        self.autopilot_commands.altitude_command = hc
        self.autopilot_commands.phi_feedforward = 0

    def _follow_orbit(self, path=msg_path(), state=msg_state()):
        p = np.array([state.pn, state.pe, -state.h])  # NED position
        d = p - path.orbit_center  # radial distance from orbit center
        rho = path.orbit_radius
        if path.orbit_direction == 'CW':
            lmbda = 1
        else:
            lmbda = -1

        var_phi = atan2(d[1], d[0])
        var_phi = self._wrap(var_phi, state.chi)
        chi_0 = var_phi + lmbda*(np.pi/2)
        chi_c = chi_0 + lmbda * atan( self.k_orbit * (np.linalg.norm(d) - rho)/rho )

        Vg = state.Vg
        chi = state.chi
        psi = state.psi
        phi_feedfw = atan(Vg**2 / ( self.gravity * rho * cos( chi - psi ) ))
        
        self.autopilot_commands.airspeed_command = path.airspeed
        self.autopilot_commands.course_command = chi_c
        self.autopilot_commands.altitude_command = -path.orbit_center[2]
        self.autopilot_commands.phi_feedforward = phi_feedfw

    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

