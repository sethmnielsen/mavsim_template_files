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
        q = np.array([path.line_direction[0], path.line_direction[1], path.line_direction[2]])
        chi_q = atan2(q[1], q[0])
        chi_q = self._wrap(chi_q, state.chi)

        Rp_i = np.array([[cos(chi_q), sin(chi_q), 0],
                         [-sin(chi_q), cos(chi_q), 0],
                         [0, 0, 1]])

        r = path.line_origin
        p = np.array([state.pn, state.pe, -state.h])

        ep = Rp_i @ (p - r)
        
        
        self.autopilot_commands.airspeed_command = path.airspeed
        self.autopilot_commands.course_command = 
        self.autopilot_commands.altitude_command =
        self.autopilot_commands.phi_feedforward =

    def _follow_orbit(self, path, state):
        self.autopilot_commands.airspeed_command =
        self.autopilot_commands.course_command =
        self.autopilot_commands.altitude_command =
        self.autopilot_commands.phi_feedforward =

    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

