import matplotlib.pyplot as plt
import numpy as np
import math
from utils import get_data

path = "./"
filename = "current.txt"
times, id, id_ref, resis, induc = get_data(filename, path)

id_old = []
id_ref_old = []
times_old = []
id_new = []
id_ref_new = []
times_new = []

getting_values = False

for i in range(len(times)):
    if resis[i] != 0 and not math.isnan(resis[i]):
        getting_values = True

        if resis[i] > 0.3: # old resistance
            id_old.append(id[i])
            id_ref_old.append(id_ref[i])
            times_old.append(times[i])
            resis_old = resis[i]
        else:
            id_new.append(id[i])
            id_ref_new.append(id_ref[i])
            times_new.append(times[i])
            resis_new = resis[i]
    
    if getting_values and resis[i] == 0:
        break

# plt.plot(times, resis, label = "resis")
plt.plot(times, id , label = "id")
# plt.plot(times_old, id_old, label = "Old")
# plt.plot(times_new, id_new, label = "New")
plt.legend()
plt.show()

# Readjust times
for i in range(len(times_old)):
    if id_ref_old[i+10] != 0:
        initial_index_old = i
        break

times_old = times_old[initial_index_old :]
id_old = id_old[initial_index_old :]
id_ref_old = id_ref_old[initial_index_old :]

for i in range(len(times_new)):
    if id_ref_new[i+10] != 0:
        initial_index_new = i
        break

times_new = times_new[initial_index_new :]
id_new = id_new[initial_index_new :]
id_ref_new = id_ref_new[initial_index_new :]


times_old = [times_old[i] - times_old[0] for i in range(len(times_old))]
times_new = [times_new[i] - times_new[0] for i in range(len(times_new))]

times_old = [times_old[i] for i in range(len(times_old)) if times_old[i] < 0.004]
id_old = id_old[:len(times_old)]
id_ref_old = id_ref_old[:len(times_old)]

times_new = [times_new[i] for i in range(len(times_new)) if times_new[i] < 0.004]
id_new = id_new[:len(times_new)]
id_ref_new = id_ref_new[:len(times_new)]


# plt.plot(times, resis, label = "Old parameters: R = 0.53, L = 210e-6")
plt.plot(times_old, id_old, label = "Old parameters: R = 0.53, L = 210e-6")
plt.plot(times_new, id_new, label = "New parameters: R = 0.24, L = 70e-6")
# plt.plot(times_old, id_ref_old, label = "Reference Old")
plt.plot(times_new, id_ref_new, label = "Reference")
plt.title(f"Time X Current")
plt.xlabel("Time [s]")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
plt.show()