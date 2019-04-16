"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from chap3.data_viewer import data_viewer
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from chap8.mav_dynamics import mav_dynamics
from chap8.observer import observer
from chap10.path_follower import path_follower
from chap11.path_manager import path_manager
from chap12.world_viewer import world_viewer
from chap12.path_planner import path_planner

# initialize the visualization
world_view = world_viewer()  # initialize the viewer
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
path_plan = path_planner()

from message_types.msg_map import msg_map
map = msg_map(PLAN)


# initialize the simulation time
sim_time = SIM.start_time

delta = np.zeros(4)
mav.update(delta)  # propagate the MAV dynamics
mav.update_sensors()

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.update_sensors()  # get sensor measurements
    estimated_state = obsv.update(measurements)  # estimate states from measurements

    # -------path planner - ----
    if path_manage.flag_need_new_waypoints == 1:
        waypoints = path_plan.update(map, estimated_state)

    #-------path manager-------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

    #-------path follower-------------
    autopilot_commands = path_follow.update(path, estimated_state)

    #-------controller-------------
    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    world_view.update(map, waypoints, path, mav.true_state)  # plot path and MAV
    if DATA:
        data_view.update(mav.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)

    sim_time += SIM.ts_simulation
