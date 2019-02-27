"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as AP
from chap6.pid_control import pid_control, pi_control, pd_control_with_rate
from message_types.msg_state import msg_state
from control import TransferFunction as TF


class autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_to_aileron = pd_control_with_rate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_to_roll = pi_control(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.sideslip_to_rudder = pi_control(
                        kp=AP.sideslip_kp,
                        ki=AP.sideslip_ki,
                        Ts=ts_control,
                        limit=np.radians(45))
        self.yaw_damper = TF(
                        np.array([AP.yaw_damper_kp, 0]),
                        np.array([1, 1/AP.yaw_damper_tau_r]),
                        ts_control)

        # instantiate longitudinal controllers
        self.pitch_to_elevator = pd_control_with_rate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_to_pitch = pi_control(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_to_throttle = pi_control(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = msg_state()

    def update(self, cmd, state):

        # lateral autopilot
        # phi_c = np.deg2rad(0)  # roll
        phi_c = self.course_to_roll.update(cmd.course_command, state.chi)
        delta_a = self.roll_to_aileron.update(phi_c, state.phi, state.p)
        # delta_r =
        
        # longitudinal autopilot
        theta_c = self.altitude_to_pitch.update(cmd.altitude_command, state.h)
        # print('\ncmd:', cmd.altitude_command)
        # print('alt:', state.h)
        delta_e = -self.pitch_to_elevator.update(theta_c, state.theta, state.q)
        delta_t = self.airspeed_to_throttle.update(cmd.airspeed_command, state.Va)
        if delta_t < 0:
            delta_t = 0
        print('\ncmd:', cmd.airspeed_command)
        print('Va :', state.Va)
        print('dt :', delta_t)

        # construct output and commanded states
        # delta = np.array([delta_e, delta_a, delta_r, delta_t])
        # print('type:', type(AP.deltas_trim))
        # print(AP.deltas_trim)
        # delta = np.array(AP.deltas_trim)
        delta = np.array([delta_e, delta_a, AP.deltas_trim[2], delta_t])
        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
