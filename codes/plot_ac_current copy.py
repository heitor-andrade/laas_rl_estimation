import matplotlib.pyplot as plt
import numpy as np
from utils import get_data

path = "./"
filename = "current_ac.txt"
times, vsupplys, ias, dcas, mean_currents, mean_tensions, frequences = get_data(filename, path)


fig, ax = plt.subplots(2)
ax[0].plot(times, ias, '.', label = "Ia [A]")
ax[0].plot(times, mean_currents, '.', label = "Mean Ia [A]")

ax[0].set_title(f"Time X Current")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Current [A]")
ax[0].grid()
ax[0].legend()


# ax[1].plot(times, frequences, '.', label = "Frequence [Hz]")
ax[1].plot(frequences, ias, '.', label = "Frequence [Hz]")
ax[1].legend()
plt.show()

# print("impedance: ", impedance)
# print("reactance: ", reactance)
# print("inductance_line: ", inductance_line)
# print("inductance_phase: ", inductance_phase)
