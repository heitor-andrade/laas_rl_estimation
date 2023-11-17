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

ud.refVelocity0 = 0
count = 0

CURRENT_MAX = 8
CURRENT_MAX_LOOP = CURRENT_MAX + 0.5
DTC_VARIATION = 0.001
dtcs, ias, vbuses, times, takes = [], [], [], [], []
take = 1
direction = 1

actual_vbus = 0
gd = "getting_data"
cv = "changing_vbus"
state = gd


while ud.velocity0 < CURRENT_MAX_LOOP:
    
    # Assign variables
    now = time.time() - init_time
    dtc = ud.velocity0
    ia = ud.current0
    vbus = ud.supply0

    if state == gd:
        # Get Data
        if (ud.refVelocity0 <= 0 and direction == -1):
            # Change state to "changing vbus"
            state = cv
            get_time = True
            last_vbus = vbus

        # Store data
        times.append(round(now, 3))
        dtcs.append(round(dtc, 3))
        ias.append(round(ia, 3))
        vbuses.append(round(vbus, 3))
        takes.append(take)

        # Set dtc
        count += 1
        if count % 5:
            ud.refVelocity0 += DTC_VARIATION * direction
            if ia > CURRENT_MAX and direction == 1:
                direction = -1

            print(  
                    "dtc sended",  round(ud.refVelocity0, 3 ),
                    "dtc",  round(dtc, 3 ),
                    "ias",  round(ia, 3 ),
                    "take",  take,
                    "vbus", round(ud.velocity1, 3)
                    )

    elif state == cv:
        break
        # Change vbus
        if vbus > 17:
            # End test
            break
        if abs(vbus - last_vbus) > 3 or (not get_time):
            # Wait to change state
            if get_time:
                last_time = now
                get_time = False
                print("I will get data in 2 seconds")
            elif now - last_time > 2:
                # change state
                state = gd
                direction = 1
                take += 1
                ud.refVelocity0 = 0
        if get_time:
            print("vbus and last_vbus", round(vbus, 2), round(last_vbus, 2))


    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

# save data
# Save data in file
f = open("vbus.txt", "w")
f.write("Time, Vsupply, dca, ia, take\n")

for i in range(len(times)):
    f.write(str(times[i]) + ", " 
            + str(vbuses[i]) + ", "
            + str(dtcs[i]) + ", " 
            + str(ias[i]) + ", " 
            + str(takes[i])
            + "\n")
f.close()