#!/usr/bin/env python3
import numpy as np
import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors
import os
import sys, traceback
import cv2

import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN
from message_types.msg_waypoints import msg_waypoints
from message_types.msg_state import msg_state

from chap3.data_viewer import data_viewer
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from chap8.mav_dynamics import mav_dynamics
from chap8.observer import observer
from chap10.path_follower import path_follower
from chap11.path_manager import path_manager

print( '\nHOLODECK PATH: {}\n'.format( os.path.dirname( holodeck.__file__ ) ) )

np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')

SHOW_PIXELS = False
DEBUG_HOLO = False
DATA = False
DIST_SCALE = 0.35

OFFSET_N = -45000 * DIST_SCALE
OFFSET_E = -50000 * DIST_SCALE

def main():
    if DATA:
        screen_pos = [2000, 0]  # x, y position on screen
        data_view = data_viewer(*screen_pos)  # initialize view of data plots
    
    # Holodeck Setup
    env = holodeck.make("Ocean", show_viewport=True)
    env.reset()
    # wave intensity: 1-13, wave size: 1-8, wave dir: 0-360 deg
    env.set_ocean_state(6, 3, 90)
    env.set_day_time(5)
    env.set_weather('Cloudy')
    env.set_aruco_code(False)

    alt = -PLAN.MAV.pd0*100

    if DEBUG_HOLO:
        env.set_control_scheme("uav0", ControlSchemes.UAV_ROLL_PITCH_YAW_RATE_ALT)
        uav_cmd = np.array([0, -0.2, 0, alt])

    uav_cmd = np.array([0,0,0,0])
    boat_cmd = 0
    env.act("uav0", uav_cmd)
    env.act("boat0", boat_cmd)

    # Just for getting UAV off the platform
    state = env.set_state("uav0", [-1500, 0, 6000], [0,0,0], [0,0,0], [0,0,0])["uav0"]
    state = env.set_state("uav0", [-1500, 0, 2000], [0,0,0], [0,0,0], [0,0,0])["uav0"]
    env.teleport("boat0", location=[3500, 0, 0], rotation=[0, 0, 0])

    # Initialization
    pos = np.array([OFFSET_N, OFFSET_E, alt])
    att = np.array([0, 0, 0])
    vel = np.array([0, 0, 0]) * 100
    angvel = np.array([0, 0, 0])
    state = env.set_state("uav0", pos, att, vel, angvel)["uav0"]

    # Camera
    cam_alt = 300
    env.teleport_camera([0, 0, cam_alt], [0, 0, -0.5])


    if DEBUG_HOLO:
        pos = state["LocationSensor"]
        # env.teleport_camera([0, 0, cam_alt], [0, 0, -0.5])
        for i in range(1):
            state = env.tick()["uav0"]
            pos = state["LocationSensor"]
            # env.teleport_camera([0, 0, cam_alt], [0, 0, -0.5])

            if SHOW_PIXELS:
                pixels = state["RGBCamera"]
                cv2.imshow('Camera', pixels)
                cv2.waitKey(0)
            

    print("READY FOR SIM")
    # sys.exit(0)

   

    # initialize elements of the architecture
    wind = wind_simulation(SIM.ts_simulation)
    mav = mav_dynamics(SIM.ts_simulation)
    ctrl = autopilot(SIM.ts_simulation)
    obsv = observer(SIM.ts_simulation)
    path_follow = path_follower()
    path_manage = path_manager()

    # waypoint definition
    waypoints = msg_waypoints()
    waypoints.type = 'straight_line'
    waypoints.type = 'fillet'
    waypoints.type = 'dubins'
    waypoints.num_waypoints = 4
    Va = PLAN.Va0
    d = 1000.0
    h = -PLAN.MAV.pd0
    waypoints.ned[:waypoints.num_waypoints] = np.array([[0, 0, -h],
                                                        [d, 0, -h],
                                                        [0, d, -h],
                                                        [d, d, -h]])
    waypoints.airspeed[:waypoints.num_waypoints] = np.array([Va, Va, Va, Va])
    waypoints.course[:waypoints.num_waypoints] = np.array([np.radians(0),
                                                        np.radians(45),
                                                        np.radians(45),
                                                        np.radians(-135)])

    # initialize the simulation time
    sim_time = SIM.start_time

    delta = np.zeros(4)
    mav.update(delta)  # propagate the MAV dynamics
    mav.update_sensors()
    # main simulation loop
    # print("Waiting for keypress")
    # input()
    print("Press Q to exit...")
    while sim_time < SIM.end_time:
        #-------observer-------------
        measurements = mav.update_sensors()  # get sensor measurements
        # estimate states from measurements
        estimated_state = obsv.update(measurements)

        #-------path manager-------------
        path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

        #-------path follower-------------
        autopilot_commands = path_follow.update(path, estimated_state)
        # autopilot_commands = path_follow.update(path, mav.true_state)

        #-------controller-------------
        delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

        #-------physical system------j-------
        current_wind = wind.update()  # get the new wind vector
        mav.update(delta, current_wind)  # propagate the MAV dynamics

        #-------update holodeck-------------
        update_holodeck(env, mav.true_state)
        draw_holodeck(env, waypoints)
        
        #-------update viewer-------------
        if DATA:
            data_view.update(mav.true_state,  # true states
                            estimated_state,  # estimated states
                            commanded_state,  # commanded states
                            SIM.ts_simulation)

        #-------increment time-------------
        sim_time += SIM.ts_simulation

def draw_holodeck(env, waypoints):
    for i in range(4):
        pt = waypoints.ned[i] * [100, 100, -100] * DIST_SCALE
        pt += np.array([OFFSET_N, OFFSET_E, 0])
        env.draw_point(pt.tolist(), [0, 255, 100], 20)
        
        k = i+1
        if i == 3:
            k = 0
        pt2 = waypoints.ned[k] * [100, 100, -100] * DIST_SCALE
        pt2 += np.array([OFFSET_N, OFFSET_E, 0])
        env.draw_line(pt.tolist(), pt2.tolist(), [200, 0, 200], 20)

def update_holodeck(env, true_state=msg_state()):
    # Set up arrays for calling set_state with holodeck
    pos = np.array([true_state.pn, true_state.pe, true_state.h]) * 100 * DIST_SCALE
    pos += np.array([OFFSET_N, OFFSET_E, 0])
    att = np.array([true_state.phi, true_state.theta, true_state.psi]) * 180.0/np.pi
    vel = [0,0,0]
    angvel = [0,0,0]
    
    env.set_state("uav0", pos, att, vel, angvel)["uav0"]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting")
    except SystemExit:
        print("sys.exit() called")
    except Exception as e:
        traceback.print_exc(file=sys.stdout)
        sys.exit(0)
    # main()
    
