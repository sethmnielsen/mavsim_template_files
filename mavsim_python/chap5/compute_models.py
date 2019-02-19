"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.tools import Euler2Quaternion, Quaternion2Euler
from control import TransferFunction as TF
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts

def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    rho = MAV.rho
    S = MAV.S_wing
    Va = mav._Va
    b = MAV.b
    beta = mav._beta
    alpha = mav._alpha
    c = MAV.c
    Jy = MAV.Jy
    g = MAV.gravity

    a_phi_1 = 0.5 * rho * (Va**2) * S * b * MAV.C_p_p * b/(2*Va)
    a_phi_2 = 0.5 * rho * (Va**2) * S * b * MAV.C_p_delta_a
    T_phi_delta_a = TF([a_phi_2], [1, -a_phi_1, 0])
    T_chi_phi = TF(np.array([g/Va]), [1,0])

    beta_dr = (rho * Va * S) / (2 * MAV.mass * np.cos(beta))
    a_beta1 = -beta_dr * MAV.C_Y_beta
    a_beta2 = beta_dr * MAV.C_Y_delta_r
    T_beta_delta_r = TF([a_beta2], [1, a_beta1])

    theta_de = rho * Va**2 * c * S / (2*Jy)
    a_theta1 = -theta_de * MAV.C_m_q * c / (2*Va)
    a_theta2 = -theta_de * MAV.C_m_alpha
    a_theta3 =  theta_de * MAV.C_m_delta_e
    T_theta_delta_e = TF([a_theta3], [1, a_theta1, a_theta2])

    T_h_theta = TF([Va], [1, 0])
    _, theta, _ = Quaternion2Euler(trim_state[6:10])
    T_h_Va = TF([theta], [1, 0])

    C_Ds = MAV.C_D_0 + MAV.C_D_alpha * alpha + MAV.C_D_delta_e * trim_input[0]
    a_V1 = ((rho * Va * S * C_Ds) - dT_dVa(mav, Va, trim_input[1])) / MAV.mass

    return T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_beta_delta_r

def compute_ss_model(mav, trim_state, trim_input):
     return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
     return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    return x_quat

def f_euler(mav, x_euler, input):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    return f_euler_

def df_dx(mav, x_euler, input):
    # take partial of f_euler with respect to x_euler
    return A
    
def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to delta
    return B

def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    return dThrust

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    return dThrust
