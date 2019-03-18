"""
mavsim_python
    - Chapter 8 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import mav_viewer
from chap3.data_viewer import data_viewer
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from chap8.mav_dynamics import mav_dynamics
from chap8.observer import observer
from tools.signals import signals

# initialize the visualization
mav_view = mav_viewer()  # initialize the mav viewer
DATA = True
if DATA:
    pos = [1500, 0]  # x, y position on screen
    data_view = data_viewer(*pos)  # initialize view of data plots

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
obsv = observer(SIM.ts_simulation)
measurements = mav.sensors

# autopilot commands
from message_types.msg_autopilot import msg_autopilot
commands = msg_autopilot()
Va_command = signals(dc_offset=25.0, 
                     amplitude=3.0, 
                     start_time=2.0, 
                     frequency = 0.01)
h_command = signals(dc_offset=100.0, 
                    amplitude=10.0, 
                    start_time=0.0, 
                    frequency = 0.02)
chi_command = signals(dc_offset=np.radians(180), 
                      amplitude=np.radians(45), 
                      start_time=5.0, 
                      frequency = 0.015)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Q to exit...")
while sim_time < SIM.end_time:

    #-------autopilot commands-------------
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = chi_command.square(sim_time)
    commands.altitude_command = h_command.square(sim_time)

    #-------controller-------------
    estimated_state = obsv.update(measurements)  # estimate states from measurements
    estimated_state.pn  = mav.true_state.pn  
    estimated_state.pe  = mav.true_state.pe  
    estimated_state.Vg  = mav.true_state.Vg  
    estimated_state.chi = mav.true_state.chi 
    estimated_state.wn  = mav.true_state.wn  
    estimated_state.wn  = mav.true_state.wn  
    estimated_state.we  = mav.true_state.we  
    estimated_state.psi = mav.true_state.psi 
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics, sensor data
    measurements = mav.update_sensors()  # get sensor measurements

    #-------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)
    if DATA:
        data_view.update(mav.true_state, # true states
                        estimated_state, # estimated states
                        commanded_state, # commanded states
                        SIM.ts_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Finished")




