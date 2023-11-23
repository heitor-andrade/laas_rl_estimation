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

testing_line = True
if testing_line:
    R = R * 2
    L = L * 2

state_rl = "set_rl"
state_current = "set_current"
time_rl = 0

state = state_rl
test = 1

time_rl = time.time() - init_time
time_ref_current = 0
sample_current = 0
f_init, f_end = 50, 3000
set_frequences = list(np.logspace(np.log10(f_init), np.log10(f_end), 50))

f_index = 0
freq = set_frequences[0]
period = 1/freq

while test <= 2:
    # Get variables
    now = time.time() - init_time
    resis = ud.velocity0
    freq_omodri = ud.current0

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
            time_freq = now
            f_index = 0
            freq = set_frequences[0]

    elif state == state_current:
        # Send frequence
        # apply freq for 2 periods
        ud.kp0 = freq

        # change frequence after 10 periods
        if now - time_freq >  10 * period:
            f_index += 1
            freq = round(set_frequences[f_index])
            period = 1/freq
            time_freq = now
        
        # change state to rl
        if f_index == len(set_frequences) - 1:
            state = state_rl
            time_rl = now
            test += 1
            ud.kp0 = 0


    # Prints
    count_print += 1
    if count_print == 10:
        print(
                "resis: ", round(ud.velocity0 , 3),
                "freq: ", round(freq_omodri),
                "test: ", test,
                round(now, 1)
                )
        count_print = 0

    # End Loop
    if now > 50:
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass



ud.stop() # Terminate