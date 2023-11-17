import matplotlib.pyplot as plt
import numpy as np
from utils import get_data

path = "./"
filename = "vbus.txt"
times, vbuses, dcas, ias, takes = get_data(filename, path)

takes_set = list(set(takes)).sort

# Compute vbuses_set and resistance
vbuses_set = []
resistances = []

last_take = 1
last_i = 0
current_max = 0
current_min = 0
for i in range(len(takes)):
    # Get current and tension min and max
    if ias[i] > 6.5 and current_min == 0:
        current_min = ias[i]
        tension_min = dcas[i] * vbuses[i]
    
    if ias[i] > 7.5 and current_max == 0:
        current_max = ias[i]
        tension_max = dcas[i] * vbuses[i]

    if i == (len(takes) - 1) or takes[i + 1] != last_take:
        # compute vbuses_set and resistances
        vbuses_set.append(np.mean(vbuses[last_i: i]))
        resistances.append((tension_max - tension_min) / (current_max - current_min))
        last_take += 1
        last_i = i+1
        current_max = 0
        current_min = 0

# Plot data
plt.plot(vbuses_set, resistances, '.')
plt.title(f"Vbuses X Resistance")
plt.xlabel("Vbuses [V]")
plt.ylabel("Resistance [Ohms]")
plt.grid()
plt.legend()
plt.show()


# Separate data based in takes
times1, vbuses1, dcas1, ias1 = [], [], [], []
times2, vbuses2, dcas2, ias2 = [], [], [], []
for i in range(len(takes)):
    if takes[i] == 1:
        times1.append(times[i])
        vbuses1.append(vbuses[i])
        dcas1.append(dcas[i])
        ias1.append(ias[i])
    elif takes[i] == takes[-1]:
        times2.append(times[i])
        vbuses2.append(vbuses[i])
        dcas2.append(dcas[i])
        ias2.append(ias[i])


vas1 = [dcas1[i] * vbuses1[i] for i in range(len(dcas1))]
vas2 = [dcas2[i] * vbuses2[i] for i in range(len(dcas2))]


# Plot data
plt.plot(vas1, ias1, '.', label = f"vbus = {vbuses1[0]}")
plt.plot(vas2, ias2, '.', label = f"vbus = {vbuses2[0]}")
plt.title(f"Tension X Current")
plt.xlabel("Tension")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
plt.show()
