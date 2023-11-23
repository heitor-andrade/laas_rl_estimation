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

while ia < 10:

    now = time.time() - init_time
    resis = ud.velocity0
    inductance = ud.current0
    frequence = ud.current1

    count_print += 1
    if count_print == 10:
        print(
                "resis phase: ", round(resis, 3),
                "inductance phase: ", round(inductance, 6),
                "resis line: ", round(resis * 2, 3),
                "inductance line: ", round(inductance * 2, 6),
                "frequence: ", round(frequence),
                round(now, 1)
                )
        count_print = 0        

    if now > 2:
        break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass



ud.stop() # Terminate