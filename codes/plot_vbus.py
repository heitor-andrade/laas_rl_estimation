import matplotlib.pyplot as plt
import numpy as np
from utils import get_data



dcas_multi = [0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17]
ias_multi = [0, 0.004, 0.314, 0.74, 1.087, 1.468, 1.823, 2.202, 2.545, 2.92, 3.26, 3.6, 3.9, 4.22, 4.53, 4.8, 5.05, 5.35]
vbus_multi = 16.3

path = "./"
sampling_windows = [29, 59, 79, 119, 179]
times, vbuses, dcas, ias, takes = [], [], [], [], []

for sampling in sampling_windows:
    filename = f"vbus_{sampling}m.txt"
    _times, _vbuses, _dcas, _ias, _takes = get_data(filename, path)
    times.append(_times)
    vbuses.append(_vbuses)
    dcas.append(_dcas)
    ias.append(_ias)

    plt.plot(_dcas, _ias, label = f"Sampling Window = {sampling}")

plt.plot(dcas_multi, ias_multi, label = "Current multimeter")
plt.title(f"Duty cicle X Current")
plt.xlabel("Duty cicle")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
plt.show()