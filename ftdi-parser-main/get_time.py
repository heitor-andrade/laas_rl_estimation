import sys
sys.path.append('../')  # Adiciona o diretório pai (pasta_principal) ao PATH
from odri_spi_ftdi import SPIuDriver, PID
import matplotlib.pyplot as plt
import numpy as np
from math import pi
import time
import random

# Initializing Omodri communication 
ud = SPIuDriver(absolutePositionMode=False, waitForInit=True)
ud.transfer()

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

count_print   = 0
ia = ud.current0

# Phase reference
# calculated
R = 0.242
L = 70e-6

# code parameter
R_code = 0.53
L_code = 210e-6

state_rl = "set_rl"
state_current = "set_current"
time_rl = 0

state = state_rl
test = 1

time_rl = time.time() - init_time
time_ref_current = 0
sample_current = 0

while test <= 2:
    

    # Get variables
    now = time.time() - init_time
    resis = ud.velocity0
    inductance = ud.current0

    # Change RL parameters
    if state == state_rl:
        if test == 1 :
            ud.refVelocity0 = R_code
            ud.kd0          = L_code
        elif test == 2:
            ud.refVelocity0 = R
            ud.kd0          = L

        if now - time_rl > 0.1:
            state = state_current
            time_current = now

    elif state == state_current:
        # Send Current reference
        if now - time_ref_current > 0.01:
            if ud.refCurrent0 == 1:
                ud.refCurrent0 = 0
            else:
                ud.refCurrent0 = 1
            
            sample_current += 1
            time_ref_current = now

        if sample_current >= 10:
            state = state_rl
            time_rl = now
            sample_current = 0
            ud.refCurrent0 = 0
            test += 1


    # Prints
    count_print += 1
    if count_print == 10:
        print(
                "resis phase: ", round(ud.velocity0 , 3),
                "inductance phase: ", round(ud.current0, 6),
                "current: ", round(ud.refCurrent0, 1),
                "test: ", test,
                "sample_current: ", sample_current,
                round(now, 3)
                )
        count_print = 0

    # End Loop
    if now > 6:
        break


    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass



ud.stop() # Terminate