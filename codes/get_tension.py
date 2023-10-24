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
LIMIT_DC = 0.35
tensions = np.linspace(0, LIMIT_DC + (LIMIT_DC/200), 200)
index_tension = 0
count_tension = 0
count_print   = 0
case = 0

now = time.time() - init_time

LIMIT_CURRENT = 7

while ud.velocity0 < LIMIT_CURRENT and ud.current0 < LIMIT_CURRENT and ud.tension0 < LIMIT_CURRENT and now < 3:
    
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
                "va: ", round(ud.velocity1, 3),
                "supply", round(ud.supply0, 1), 
                "ia: ", round(ud.velocity0, 3),
                # "ib: ", round(ud.current0, 3),
                "tension ", round((ud.velocity1 - 0.015) * ud.supply0, 2), 
                "inverse", round(ud.supply0 - (ud.velocity1 - 0.015) * (ud.supply0), 2)
                # "vb: ", round(ud.current1, 3),
                # "vc: ", round(ud.tension1, 3),
                # "ic: ", round(ud.tension0, 3)
                )
        count_print = 0        


    # # Set tension value
    # count_tension += 1
    # if count_tension == 5:
    #     index_tension += 1
    #     count_tension = 0

    #     if ud.velocity0 > 5 or ud.current0 < 5 or ud.tension0 < 5:
    #         if case == 2:
    #             break
    #         case += 1
    #         index_tension = 0

    #     if case == 0:
    #         ud.kp0, ud.refVelocity0, ud.refCurrent0 = tensions[index_tension], 0, 0     # va, vb, vc
    #     if case == 1:
    #         ud.kp0, ud.refVelocity0, ud.refCurrent0 = 0, tensions[index_tension], 0     # va, vb, vc
    #     if case == 2:
    #         ud.kp0, ud.refVelocity0, ud.refCurrent0 = 0, 0, tensions[index_tension]     # va, vb, vc

    ud.kp0              = 0.0350     #va # 0.0375     
    ud.refVelocity0     = 0     #vb
    ud.refCurrent0      = 0     #vc

    # Duty cicle, Graph [A] , Multimer [A]
    # 0.0275    , 2         , 0.587
    # 0.0300    , 1.3       , 0.666
    # 0.0325    , 1.3       , 0.744
    # 0.0350    , 1.3       , 0.821
    # 0.0375    , 1.3       , 0.897

    # fail safe
    if ud.velocity0 > 100 or ud.velocity0 > LIMIT_CURRENT or ud.current0 > LIMIT_CURRENT or ud.tension0 > LIMIT_CURRENT:
        print("Stopped!")
        print("ud.velocity0,ud.current0, ud.tension0", ud.velocity0,ud.current0, ud.tension0)
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

measured = np.mean(ias[-1])
print(measured)
# current = float(input("Current of multimetre: "))

# print(
#     "Measured: ", measured,
#     "Multimeter: ", current,
#     "Error: ", measured - current, " Error %= " , round((measured - current) / current * 100)
#     )

# # Save data in file
# f = open("current_dc.txt", "w")
# f.write("Time, Vsupply, ia, ib, ic, dca, dcb, dcc, dc_activated\n")

# for i in range(len(times)):
#     f.write(str(times[i]) + ", " 
#             + str(vsupplys[i]) + ", "
#             + str(ias[i]) + ", " 
#             + str(ibs[i]) + ", "
#             + str(ics[i]) + ", " 
#             + str(dcas[i]) + ", " 
#             + str(dcbs[i]) + ", " 
#             + str(dccs[i]) + ", " 
#             + str(dc_activated[i])
#             + "\n")
# f.close()

# sys.exit()