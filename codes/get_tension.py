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
DTC_VARIATION = 0.01

LIMIT_CURRENT = 10
ia = ud.current0
time_dtc = time.time() - init_time

ud.refVelocity0 = 0.04 # duty cicle iniciale 


while ia < LIMIT_CURRENT:


    now = time.time() - init_time
    dtc = ud.velocity0
    ia = ud.current0
    vbus = ud.supply0

    count_print += 1
    if count_print == 10:
        print(
                "dtc: ", round(dtc, 3),
                "vbus", round(vbus, 1),
                "ia: ", round(ia, 3),
                now
                )
        count_print = 0        

    # if now - time_dtc > 1:
    #     break
        # time_dtc = now
        # ud.refVelocity0 += DTC_VARIATION


    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate