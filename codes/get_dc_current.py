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
f = 2       # Hz
A = 0.4    # Amplitude
offset = 0.35

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

DEAD_TIME = 0.016
LIMIT_DC = 0.15
delta_tension = 0.001 
index_tension = 0
count_tension = 0
count_print   = 0
case = 0

now = time.time() - init_time

LIMIT_CURRENT_WHILE = 5.5
LIMIT_CURRENT_CASE = 5
tension = 0
direction = 1

while ud.velocity0 < LIMIT_CURRENT_WHILE and ud.current0 < LIMIT_CURRENT_WHILE and ud.tension0 < LIMIT_CURRENT_WHILE:
    
    now = time.time() - init_time
    
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
        print(
                "tension ", round(ud.velocity1 * ud.supply0, 2), 
                "inverse", round(ud.supply0 - (ud.velocity1 - 0.016) * (ud.supply0), 2), 
                "va: ", round(ud.velocity1, 3),
                # "vb: ", round(ud.current1, 3),
                # "vc: ", round(ud.tension1, 3),
                "ia: ", round(ud.velocity0, 3)
                # "ib: ", round(ud.current0, 3),
                # "ic: ", round(ud.tension0, 3)
                )
        count_print = 0        


    # # Set tension value
    count_tension += 1
    if count_tension == 10:
        index_tension += 1
        count_tension = 0

        tension += delta_tension*direction

        if case == 0:
            ud.kp0, ud.refVelocity0, ud.refCurrent0 = tension, 0, 0     # va, vb, vc
        if case == 1:
            ud.kp0, ud.refVelocity0, ud.refCurrent0 = 0, tension, 0     # va, vb, vc
        if case == 2:
            ud.kp0, ud.refVelocity0, ud.refCurrent0 = 0, 0, tension     # va, vb, vc


    if ud.velocity0 > LIMIT_CURRENT_CASE or ud.current0 > LIMIT_CURRENT_CASE or ud.tension0 > LIMIT_CURRENT_CASE:
        direction = -1
        # if case == 2:
        #     print("break")
        #     break
        # case += 1
        # tension = 0

    if direction == -1 and tension <= 0:
        break

    # fail safe
    if ud.velocity0 > 100 or ud.velocity0 > LIMIT_CURRENT_WHILE or ud.current0 > LIMIT_CURRENT_WHILE or ud.tension0 > LIMIT_CURRENT_WHILE:
        print("Stopped!")
        print("ud.velocity0,ud.current0, ud.tension0", ud.velocity0,ud.current0, ud.tension0)
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

# Save data in file
f = open("current_dc.txt", "w")
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