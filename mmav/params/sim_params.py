import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.03  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 50000.  # end time for simulation

ts_plotting = 0.2  # refresh rate for plots

ts_control = ts_simulation  # sample rate for the controller

