#!/usr/bin/env python3
from __future__ import print_function
from rosflight_holodeck_interface import ROSflightHolodeck
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import time
import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors
from IPython.core.debugger import Pdb
import os
import cv2

print( '\nHOLODECK PATH: {}\n'.format( os.path.dirname( holodeck.__file__ ) ) )

np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')

def Quaternion2Euler(quat):
    e0 = quat[0]
    e1 = quat[1]
    e2 = quat[2]
    e3 = quat[3]

    phi = np.arctan2(2*(e0*e1 + e2*e3), (e0**2 + e3**2 - e1**2 - e2**2))
    theta = np.arcsin(2*(e0*e2 - e1*e3))
    psi = np.arctan2(2*(e0*e3 + e1*e2), (e0**2 + e1**2 - e2**2 - e3**2))

    return np.array([phi, theta, psi])

def Euler2Quaternion(eul):
    phi2 = eul[0]/2
    tha2 = eul[1]/2
    psi2 = eul[2]/2

    e0 = np.cos(psi2)*np.cos(tha2)*np.cos(phi2) + np.sin(psi2)*np.sin(tha2)*np.sin(phi2)
    e1 = np.cos(psi2)*np.cos(tha2)*np.sin(phi2) - np.sin(psi2)*np.sin(tha2)*np.cos(phi2)
    e2 = np.cos(psi2)*np.sin(tha2)*np.cos(phi2) + np.sin(psi2)*np.cos(tha2)*np.sin(phi2)
    e3 = np.sin(psi2)*np.cos(tha2)*np.cos(phi2) - np.cos(psi2)*np.sin(tha2)*np.sin(phi2)

    return np.array([e0, e1, e2, e3])

if __name__ == '__main__':
    env = holodeck.make("Ocean")
    RF = ROSflightHolodeck()
    env.reset()
    env.tick()

    # wave intensity: 1-13(int), wave size: 1-8(int), wave direction: 0-360 degreese (float)
    env.set_ocean_state(3, 3, 90)
    env.set_aruco_code(False)

    x0 = np.array([0, 0, 0,     # position [0-2]
                   1, 0, 0, 0,  # attitude [3-6]
                   0, 0, 0,     # velocity [7-9]
                   0, 0, 0,     # omega [10-12]
                   0, 0, 0], dtype=np.float64) # acc [13-15]

    h0 = 41  # initial altitude [m]
    x0[2] = -h0
    # x0[0] = -34.5
    RF.init()
    RF.setState(x0)
    RF.setTime(10)

    uav_cmd = np.array([0, 0, 0, 0])
    boat_cmd = 0
    env.act("uav0", uav_cmd)
    env.act("boat0", boat_cmd)

    pos0 = x0[:3] * [100, 100, -100]  # convert to cm, negate z
    env.teleport("uav0", location=pos0, rotation=[0,0,0])
    env.teleport("boat0", location=[-2000,0,0], rotation=[0,0,0])

    frate = 30      # frame rate of camera/rendering [Hz]
    simrate = 800  # rate of simulated dynamics [Hz]
    n = simrate//frate # Number of sim iterations between frames
    dt = 1.0/simrate
    #'F' order because eigen matrices are column-major while numpy are row-major
    # x_arr = np.zeros((16, n), order='F')
    x = np.zeros(16)
    t = np.zeros(n)
    rf_outputs = np.zeros(4)
    state = np.array([])
    pos = np.zeros(3)
    att = np.zeros(3)
    ground = -0.1
    collision = False
    count = 0
    while 1:  # Main loop: 1 iteration = 1 rendered frame
        if not collision:
            for i in range(n):  # Loop between frames (dynamics/control)
                RF.run(dt)
                time.sleep(dt)

            RF.getState(x)
            # Make sure mav doesn't fall through ground
            if x[2] > ground and x[9] > 0:
                # at ground level and not gaining altitude 
                x_ground = np.copy(x0)
                x_ground[:3] = [x[0], x[1], ground]
                att_eul = Quaternion2Euler(x[3:7])
                ground_eul = np.array([0, 0, att_eul[2]])
                x_ground[3:7] = Euler2Quaternion(ground_eul)

                pos = x_ground[:3] * [100,100,-100]
                att = ground_eul * 180/np.pi
                RF.setState(x_ground)
                state = env.set_state("uav0", pos, att, [0,0,0], [0,0,0])["uav0"]
            else:
                # format state for holodeck
                pos = x[:3] * [100,100,-100] # cm, negate z to convert to LH frame
                att = Quaternion2Euler(x[3:7]) * 180/np.pi
                vel = x[7:10] * [100,100,-100]
                angvel = np.copy(x[10:13])
                
                state = env.set_state("uav0", pos, att, vel, angvel)["uav0"]
                collision = state['CollisionSensor']
        elif collision:
            # Use position given by holodeck
            state = env.tick()["uav0"]
            x = np.copy(x0)
            x[:3] = state['LocationSensor'] * [1,1,-1]
            x[7:10] = state['VelocitySensor'] * [1,1,-1]
            RF.setState(x)
            for k in range(10):
                RF.run(dt)
                time.sleep(dt*(n/10))
            RF.getState(x)  
            RF.getOutputs(rf_outputs)
            if x[9] < 0:
                # gaining altitude, switch back to RF dynamics
                collision = False

        # Show UAV's camera feed
        # frame = state['RGBCamera']
        # cv2.imshow('Camera', frame)
        # cv2.waitKey(1)

        # For debugging
        RF.getState(x)