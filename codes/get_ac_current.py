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
times, vsupplys, ias, dcas, currents, tensions, current_cases, frequences, inductance_lines = [], [], [], [], [], [], [], [], []

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

count_print = 0
current_case = 0

while current_case != -1:
    
    now = time.time() - init_time
    
    # Store data
    times.append(round(now, 3))
    vsupplys.append(round(ud.supply0, 2))
    ias.append(round(ud.velocity0, 3))
    currents.append(round(ud.current0, 3))
    tensions.append(round(ud.tension0, 3))
    dcas.append(round(ud.velocity1, 3))
    current_cases.append(round(ud.current1))
    frequences.append(round(ud.tension1, 3))
    inductance_lines.append(ud.supply1)

    current_case = round(ud.current1)

    count_print += 1
    if count_print == 1:
        print(
                "current_case : ", current_case, 
                "frequence: ", round(ud.tension1),
                "current: ", round(ud.current0, 3),
                "tension: ", round(ud.tension0, 3),
                "inductance: ", ud.supply1
                )
        count_print = 0        


    # fail safe
    if ud.velocity0 > 3:
        print("Stopped!", ud.velocity0)
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

print(current_case)
inductance_line = inductance_lines[-1]
print("inductance line ", inductance_line)
print("inductance phase ", inductance_line*2/3)

# Save data in file
f = open("current_ac.txt", "w")
f.write("Time, Vsupply, ia, dca, current, tension, current_case, frequence, inductance_line\n")

for i in range(len(times)):
    f.write(str(times[i]) + ", " 
            + str(vsupplys[i]) + ", "
            + str(ias[i]) + ", " 
            + str(dcas[i]) + ", " 
            + str(currents[i]) + ", "
            + str(tensions[i]) + ", " 
            + str(current_cases[i]) + ", " 
            + str(frequences[i]) + ", " 
            + str(inductance_lines[i])
            + "\n")
f.close()

sys.exit()