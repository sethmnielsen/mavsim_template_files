"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import mav_viewer
from chap3.data_viewer import data_viewer
from chap4.mav_dynamics import mav_dynamics
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from tools.signals import signals
from message_types.msg_autopilot import msg_autopilot

# initialize the visualization
mav_view = mav_viewer()  # initialize the mav viewer
DATA = False
if DATA:
    data_view = data_viewer()  # initialize view of data plots

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)

# autopilot commands
commands = msg_autopilot()
Va_command = signals(dc_offset=25.0, amplitude=5.0, start_time=2.0, frequency = 0.1)
h_command = signals(dc_offset=100.0, amplitude=10.0, start_time=0.0, frequency = 0.1)
# chi_command = signals(dc_offset=np.radians(180), amplitude=np.radians(45), start_time=5.0, frequency = 0.015)
chi_command = signals(dc_offset=np.radians(180), amplitude=np.radians(45), start_time=5.0, frequency = 0.1)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    #-------controller-------------
    estimated_state = mav.true_state  # uses true states in the control
    commands.airspeed_command = Va_command.square(sim_time)
    # commands.course_command = chi_command.square(sim_time)
    commands.course_command = 0
    commands.altitude_command = h_command.square(sim_time)
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update_state(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    if DATA:
        data_view.update(mav.true_state, # true states
                        mav.true_state, # estimated states
                        commanded_state, # commanded states
                        SIM.ts_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Finished")