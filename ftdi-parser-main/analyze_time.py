import matplotlib.pyplot as plt
import numpy as np
import math
from utils import get_data

path = "./"
filename = "time.txt"
times, ids, ids_ref, resistances, tests = get_data(filename, path)



# code, phase, line
_ids = [[], [], []]
_ids_ref = [[], [], []]
_times = [[], [], []]

for i in range(len(tests)):
    if not math.isnan(tests[i]):
        tests[i] = int(round(tests[i]))
    else:
        tests[i] = 0

# plt.plot(times, ids , label = "id")
# plt.plot(times, tests , label = "test")
# plt.legend()
# plt.show()


# Separate code, phase and line values
getting_values = False
for i in range(len(times)):
    if tests[i] != 0:
        _ids[tests[i] - 1].append(ids[i])
        _ids_ref[tests[i] - 1].append(ids_ref[i])
        _times[tests[i] - 1].append(times[i])

ids = _ids
ids_ref = _ids_ref
times = _times



ids_code        = ids[0]
ids_ref_code    = ids_ref[0]
times_code      = times[0]

ids_phase        = ids[1]
ids_ref_phase    = ids_ref[1]
times_phase      = times[1]

ids_line        = ids[2]
ids_ref_line    = ids_ref[2]
times_line      = times[2]

# Readjust times
def readjust_time(times, ids, ids_ref):
    for i in range(len(times)):
        if ids_ref[i+10] != 0:
            initial_index = i
            break

    times = times[initial_index :]
    ids = ids[initial_index :]
    ids_ref = ids_ref[initial_index :]

    times = [times[i] - times[0] for i in range(len(times))]

    times = [times[i] for i in range(len(times)) if times[i] < 0.004]
    ids = ids[:len(times)]
    ids_ref = ids_ref[:len(times)]

    return times, ids, ids_ref

times_code, ids_code, ids_ref_code = readjust_time(times_code, ids_code, ids_ref_code)
times_phase, ids_phase, ids_ref_phase = readjust_time(times_phase, ids_phase, ids_ref_phase)
times_line, ids_line, ids_ref_line = readjust_time(times_line, ids_line, ids_ref_line)

plt.plot(times_code, ids_code , label = "Code paramaters")
plt.plot(times_phase, ids_phase , label =   "Computed paramaters")
# plt.plot(times_line, ids_line , label =     "Line phase paramaters")
plt.plot(times_code, ids_ref_code , label = "Refence")
plt.plot(times_code, [0.1]*len(ids_ref_code), 'r')
plt.plot(times_code, [0.9]*len(ids_ref_code), 'r')
# plt.plot(times_phase, ids_ref_phase , '.', label = "Refence phase")
# plt.plot(times_line, ids_ref_line , label = "refence line")
plt.title(f"Time response of the current controller to a step")
plt.xlabel("Time [s]")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
plt.show()


# # plt.plot(times, resis, label = "Old parameters: R = 0.53, L = 210e-6")
# plt.plot(times_old, id_old, label = "Old parameters: R = 0.53, L = 210e-6")
# plt.plot(times_new, id_new, label = "New parameters: R = 0.24, L = 70e-6")
# # plt.plot(times_old, id_ref_old, label = "Reference Old")
# plt.plot(times_new, id_ref_new, label = "Reference")
# plt.title(f"Time X Current")
# plt.xlabel("Time [s]")
# plt.ylabel("Current [A]")
# plt.grid()
# plt.legend()
# plt.show()