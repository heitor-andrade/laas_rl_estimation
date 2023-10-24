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
current_case = 0

f_init, f_end = 100, 5000
frequences = np.logspace(np.log10(f_init), np.log10(f_end))
index_freq = 0

# Set offset and amplitude of duty cicle
offset = 0.05
amplitude = 0.01


while current_case != -1 index_freq <= len(frequences):
    
    now = time.time() - init_time
    
    # Set frequence and tension waveform
    freq = frequences[index_freq]
    if now - time_freq > 0.1:
        index_freq += 1
        time_freq = now

    # Define Data to send
    ud.# offset
    ud.# amplitude
    ud.# frequence

    # Store data after 10ms that the frequence changed
    if now - time_freq > 0.01:
        times.append(round(now, 3))
        vsupplys.append(round(ud.supply0, 2))
        ias.append(round(ud.velocity0, 3))
        currents.append(round(ud.current0, 3))
        tensions.append(round(ud.tension0, 3))
        dcas.append(round(ud.velocity1, 3))
        inductance_lines.append(ud.supply1)

        mean_currents.append(round(ud.current0, 3))
        mean_tensions.append(round(ud.tension0, 3))
        frequences.append(round(freq))


    count_print += 1
    if count_print == 1:
        print(
                "current_case : ", current_case, 
                "frequence: ", freq,
                "current: ", round(ud.current0, 3),
                "tension: ", round(ud.tension0, 3),
                "inductance: ", ud.supply1,
                "reactance: ",  ud.velocity1
                )
        count_print = 0        


    # fail safe
    if ud.velocity0 > 5.5:
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
f.write("Time, Vsupply, ia, dca, current, tension, frequence, inductance_line\n")

for i in range(len(times)):
    f.write(str(times[i]) + ", " 
            + str(vsupplys[i]) + ", "
            + str(ias[i]) + ", " 
            + str(dcas[i]) + ", " 
            + str(currents[i]) + ", "
            + str(tensions[i]) + ", " 
            + str(frequences[i]) + ", " 
            + str(inductance_lines[i])
            + "\n")
f.close()

sys.exit()


# f,    L line,     Lphase,     ac current,     dc current
# 500,  31.1e-6,    20  e-6,    2.36            2,46
# 1000, 18.5e-6,    10.4e-6,    2.35            2.46
# 4000, 4.83e-6,    3.22e-6,    
# 5000, 3.21e-6,    2.14e-6,    2.372