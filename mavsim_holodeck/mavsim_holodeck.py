#!/usr/bin/env python3
from message_types.msg_waypoints import msg_waypoints
import numpy as np
import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors
import os
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from chap3.data_viewer import data_viewer
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from chap8.mav_dynamics import mav_dynamics
from chap8.observer import observer
from chap10.path_follower import path_follower
from chap11.path_manager import path_manager

print( '\nHOLODECK PATH: {}\n'.format( os.path.dirname( holodeck.__file__ ) ) )

np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')

# Holodeck Setup
env = holodeck.make("UrbanCity")

DATA = True
if DATA:
    screen_pos = [2000, 0]  # x, y position on screen
    data_view = data_viewer(*screen_pos)  # initialize view of data plots

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
waypoints.ned[:waypoints.num_waypoints] = np.array([[0,    0, -100],
                                                    [1000,    0, -100],
                                                    [0, 1000, -100],
                                                    [1000, 1000, -100]])
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
print("Press Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.update_sensors()  # get sensor measurements
    # estimate states from measurements
    estimated_state = obsv.update(measurements)

    #-------path manager-------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

    #-------path follower-------------
    # autopilot_commands = path_follow.update(path, estimated_state)
    autopilot_commands = path_follow.update(path, mav.true_state)

    #-------controller-------------
    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    if DATA:
        data_view.update(mav.true_state,  # true states
                         estimated_state,  # estimated states
                         commanded_state,  # commanded states
                         SIM.ts_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_simulation
