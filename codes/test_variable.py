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

count = 0
while True:
    
    now = time.time() - init_time
    
    count += 1
    if count == 10:
        print(  "inductance",  ud.velocity0,
                "resistance", ud.current0
                # "frequence", round(ud.velocity1, 3)
                )
        count = 0
    
    # if (ud.velocity0 != 0 and ud.current0 != 0):
    #     print(  "inductance",  ud.velocity0,
    #     "resistance", ud.current0
    #     # "frequence", round(ud.velocity1, 3)
    #     )
    #     print(now)
    #     break

    # # FAIL SAFE
    # if ud.velocity0 > 4:
    #     print("Stopped!", ud.velocity0)
    #     break

    ud.transfer()
    t += dt
    while(time.perf_counter()-t<dt):
        pass

ud.stop() # Terminate