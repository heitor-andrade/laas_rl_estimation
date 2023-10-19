import matplotlib.pyplot as plt
import numpy as np
from utils import get_data

path = "./"
filename = "current_ac.txt"
times, vsupplys, ias, ibs, ics, dcas, dcbs, dccs, dc_activated = get_data(filename, path)


vas = [ics[i]*vsupplys[i] for i in range(len(dcas))]
# vbs = [dcbs[i]*vsupplys[i] for i in range(len(dcbs))]
# vcs = [dccs[i]*vsupplys[i] for i in range(len(dccs))]

fig, ax = plt.subplots(2)
ax[0].plot(times, ias, '.', label = "ia [A]")
ax[0].plot(times, ibs, '.', label = "mean ia [A]")


ax[0].set_title(f"Tension X Current")
ax[0].set_xlabel("Tension [V]")
ax[0].set_ylabel("Current [A]")
ax[0].grid()
ax[0].legend()


ax[1].plot(times, dcas, '.', label = "Duty cicle a")
ax[1].plot(times, ics, '.', label = "Duty cicle a")
ax[1].legend()
plt.show()

print("impedance: ", impedance)
print("reactance: ", reactance)
print("inductance_line: ", inductance_line)
print("inductance_phase: ", inductance_phase)
