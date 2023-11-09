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
tests, vsupplys, ias, ias_dc, dcas = [], [], [], [], []

# Principal loop 
dt          = 0.001  # Period Loop
t           = time.perf_counter()
init_time   = time.time()

count_print = 0
data_size = 30
data_index = 0
last_data_index = -1

test = 0

state = 0
ud.refVelocity0 = 0
ud.refCurrent0 = 0.01
while True:
    
    now = time.time() - init_time
    
    # if state == 0:  # Define Data to send and get dc_current
    #     ud.refCurrent0 = 0.09           # duty_cicle
    #     ud.refVelocity0 = 0             # test_case
    #     dc_current = ud.velocity0

    #     if now > 0.5 :
    #         state = 1
    #         time_start = now

    #         ud.refVelocity0 = 1         # test_case
    #         ud.kp0 = 0                  # data_index

    # elif now - time_start > 1 and state == 1:      # Get_data after tests finished [2 seconds]
    #     ud.refVelocity0 = 3             # test_case
        
    #     if data_index == last_data_index:
    #         data_index += 1
    #         ud.kp0 = data_index                   # data_index
    #         if data_index == data_size:
    #             # ud.kp0 = data_index
    #             break
    #     else:
    #         print(
    #                 "current", round(ud.current0, 3),
    #                 "test", ud.current1,
    #                 "test_case", ud.tension0
    #                 )

    #         tests.append(test)
    #         vsupplys.append(round(ud.supply0, 2))
    #         ias.append(round(ud.current0, 3))
    #         ias_dc.append(round(dc_current, 3))
    #         dcas.append(round(ud.velocity1, 3))
            
    #         last_data_index = data_index
    

    # count_print += 1
    # if count_print == 1:
    #     print(
    #             "dc", ud.velocity1, 
    #             "current", ud.velocity0,
    #             "index desired", ud.kp0,
    #             "index obtained", ud.current1)
    #     count_print = 0        


    # fail safe
    if ud.velocity0 > 4:
        print("Stopped!", ud.velocity0)
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate

# Save data in file
f = open("current_ac.txt", "w")
f.write("tests, Vsupply, ia, ia continous, dca\n")

for i in range(len(tests)):
    f.write(str(tests[i]) + ", " 
            + str(vsupplys[i]) + ", "
            + str(ias[i]) + ", " 
            + str(ias_dc[i]) + ", " 
            + str(dcas[i])
            + "\n")
f.close()

sys.exit()


# f,    L line,     Lphase,     ac current,     dc current
# 500,  31.1e-6,    20  e-6,    2.36            2,46
# 1000, 18.5e-6,    10.4e-6,    2.35            2.46
# 4000, 4.83e-6,    3.22e-6,    
# 5000, 3.21e-6,    2.14e-6,    2.372