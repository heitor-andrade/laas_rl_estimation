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

# data to be stored
times, vsupplys, ias, ibs, ics, dcas, dcbs, dccs, dc_activated = [], [], [], [], [], [], [], [], []

# Current caracteristics
f = 150           # Hz
w = f*2*np.pi   # rad/s
amplitude = 0.01         # Amplitude
offset = 0.05

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

DEAD_TIME = 0.016
tensions = np.linspace(0, 0.5 + (0.5/200), 100)
index_tension = 0
start_tension = np.linspace(0, offset + (offset/200), 100)
index_start = 0
count_tension = 0
count_print   = 0
case = 0

while True:
    
    now = time.time() - init_time
    
    if index_start < len(start_tension):
        ud.kp0      = start_tension[index_start]
        index_start += 1
    
    else:
        # Store data
        times.append(round(now, 3))
        vsupplys.append(round(ud.supply0, 2))
        ias.append(round(ud.velocity0, 3))
        ibs.append(round(ud.current0, 3))
        ics.append(round(ud.tension0, 3))
        dcas.append(round(ud.velocity1, 3))
        dcbs.append(round(ud.current1, 3))
        dccs.append(round(ud.tension1, 3))
        dc_activated.append(case)


        count_print += 1
        if count_print == 10:
            print("index %: ", round(index_tension * 100/ (len(tensions) - 1)), 
                    "va: ", round(ud.velocity1, 3),
                    "vb: ", round(ud.current1, 3),
                    "vc: ", round(ud.tension1, 3))
            count_print = 0        


        # Set tension value
        count_tension += 1
        if count_tension == 5000:
            break

        ud.kp0              = offset + amplitude*np.sin(w*now)
        ud.refVelocity0     = 0
        ud.refCurrent0      = 0


    # fail safe
    if ud.velocity0 > 100:
        print("Stopped!")
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

# Save data in file
f = open("current_ac.txt", "w")
f.write("Time, Vsupply, ia, ib, ic, dca, dcb, dcc, dc_activated\n")

for i in range(len(times)):
    f.write(str(times[i]) + ", " 
            + str(vsupplys[i]) + ", "
            + str(ias[i]) + ", " 
            + str(ibs[i]) + ", "
            + str(ics[i]) + ", " 
            + str(dcas[i]) + ", " 
            + str(dcbs[i]) + ", " 
            + str(dccs[i]) + ", " 
            + str(dc_activated[i])
            + "\n")
f.close()

sys.exit()