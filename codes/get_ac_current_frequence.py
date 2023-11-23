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
times, vsupplys, ias, dcas, currents, tensions, mean_currents, mean_tensions, frequences, inductance_lines = [], [], [], [], [], [], [], [], [], []

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

count_print = 0

f_init, f_end = 1, 1000
set_frequences = list(np.logspace(np.log10(f_init), np.log10(f_end), 200))
index_freq = 0
set_frequences = [0] + set_frequences
freq = set_frequences[0]
time_freq = time.time() - init_time

time_ampl = 0
max_i, min_i = 0, 0
while index_freq < len(set_frequences):
    
    now = time.time() - init_time
    
    # Set frequence 
    if now - time_freq > 0.1:
        freq = set_frequences[index_freq]
        time_freq = now
        index_freq += 1


    # Define Data to send
    ud.refCurrent0 = 0.06           # offset
    ud.refVelocity0 = 0.01         # amplitude
    # ud.kp0 = 2000                   # frequence
    ud.kd0 = 0.001                  # alpha_flt

    # Store data after 10ms that the frequence changed
    if now - time_freq > 0.01:
        times.append(round(now, 3))
        vsupplys.append(round(ud.supply0, 2))
        ias.append(round(ud.velocity0, 3))
        dcas.append(round(ud.velocity1, 3))
        mean_currents.append(round(ud.current0, 3))
        mean_tensions.append(round(ud.tension0, 3))
        frequences.append(round(ud.tension1, 1))


    # get amplitude maximum of each frequence max - min / 2
    if now - time_ampl > 5:
        max_i = 0
        min_i = 100
        time_ampl = now

    current = ud.velocity0
    if current > max_i:
        max_i = current
    
    if current < min_i:
        min_i = current

    count_print += 1
    if count_print == 10:
        print(
                "frequence: ", round(ud.tension1, 1),
                # "current: ", round(ud.current0, 3),
                # "tension: ", round(ud.tension0, 3),
                # "index_freq: ", index_freq,
                # "len_freq: ", len(set_frequences),
                "dcas: ",  round(ud.velocity1, 3),
                "amplitude tension: ", round(ud.supply0 * ud.refVelocity0, 3),
                "amplitude courant: ", round((max_i - min_i), 3)
                )
        count_print = 0        


    # fail safe
    if ud.velocity0 > 6:
        print("Stopped!", ud.velocity0)
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

# Save data in file
f = open("current_ac_frequence_1.txt", "w")
f.write("Time, Vsupply, ia, dca, current, tension, frequence\n")

for i in range(len(times)):
    f.write(str(times[i]) + ", " 
            + str(vsupplys[i]) + ", "
            + str(ias[i]) + ", " 
            + str(dcas[i]) + ", " 
            + str(mean_currents[i]) + ", "
            + str(mean_tensions[i]) + ", " 
            + str(frequences[i])
            + "\n")
f.close()

sys.exit()


# f,    L line,     Lphase,     ac current,     dc current
# 500,  31.1e-6,    20  e-6,    2.36            2,46
# 1000, 18.5e-6,    10.4e-6,    2.35            2.46
# 4000, 4.83e-6,    3.22e-6,    
# 5000, 3.21e-6,    2.14e-6,    2.372