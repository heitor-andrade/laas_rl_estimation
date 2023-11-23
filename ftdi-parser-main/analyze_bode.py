import matplotlib.pyplot as plt
import numpy as np
from utils import get_data
import math

path = "./"
filename = "bode.txt"
times, ids_ref, ids, resistances, frequences = get_data(filename, path)

# plt.plot(times, ids, label = 'id')
# plt.plot(times, ids_ref, label = 'id ref')
# plt.plot(times, resistances, '.', label = 'resis')
# plt.legend()
# plt.show()

ids_old = []
ids_ref_old = []
times_old = []
frequences_old = []

ids_new = []
ids_ref_new = []
times_new = []
frequences_new = []

getting_values = False
for i in range(len(times)):
    if resistances[i] != 0 and not math.isnan(resistances[i]):
        getting_values = True

        if resistances[i] > 0.5: # old resistance
            ids_old.append(ids[i])
            ids_ref_old.append(ids_ref[i])
            times_old.append(times[i])
            frequences_old.append(frequences[i])
        else:
            ids_new.append(ids[i])
            ids_ref_new.append(ids_ref[i])
            times_new.append(times[i])
            frequences_new.append(frequences[i])
    
    if getting_values and resistances[i] == 0:
        break

# plt.plot(frequences_old, ids_old,'.', label = 'id old')
# plt.plot(times_old, ids_old, '.', label = 'id old')
# plt.plot(times_old, ids_ref_old, '.', label = 'id old')
plt.plot(times, resistances, '.', label = 'resis')
plt.legend()
plt.show()

# filter f = 0
def filter_0frequence(frequences, times, ids):
    i = 0
    while i < len(frequences):
        if frequences[i] == 0:
            frequences.pop(i)
            ids.pop(i)
            times.pop(i)
        else:
            i+=1
    return frequences, times, ids

frequences_old, times_old, ids_old = filter_0frequence(frequences_old, times_old, ids_old)
frequences_new, times_new, ids_new = filter_0frequence(frequences_new, times_new, ids_new)


# filter begin of sinusoidal
# because the frequence change causes noise
def filter_beg_sin(frequences, times, ids):
    last_frequence = frequences[0]
    begin = True
    i = 0
    counter_begin = 0
    while i < len(frequences):
        if frequences[i] != last_frequence:
            last_frequence = frequences[i]
            begin = True
        
        if begin:
            frequences.pop(i)
            ids.pop(i)
            times.pop(i)
            counter_begin += 1
        else:
            i+=1
            
        if counter_begin >= 25:
            begin = False
            counter_begin = 0


    return frequences, times, ids

frequences_old, times_old, ids_old = filter_beg_sin(frequences_old, times_old, ids_old)
frequences_new, times_new, ids_new = filter_beg_sin(frequences_new, times_new, ids_new)

# plt.plot(times_old, ids_old,'.', label = 'id old')
# plt.plot(times_new, ids_new,'.', label = 'id new')
# plt.plot(frequences_new, ids_new,'.', label = 'id new')
# plt.legend()
# plt.show()

# # get amplitude maximum of each frequence
def get_amplitudes(frequences, ids):
    fs = list(set(frequences))
    fs.sort()
    amplitudes = [0] * len(fs)
    index_amplitude = 0
    last_frequence = frequences[0]
    max_i = 0

    for i in range(len(frequences)):

        if frequences[i] != last_frequence:
            amplitudes[index_amplitude] = max_i
            index_amplitude += 1
            last_frequence = frequences[i]
            max_i = 0
        
        if ids[i] > max_i:
            max_i = ids[i]

    amplitude = max_i
    amplitudes[index_amplitude] = amplitude

    return fs, amplitudes

frequences_set_old, amplitudes_old = get_amplitudes(frequences_old, ids_old)
frequences_set_new, amplitudes_new = get_amplitudes(frequences_new, ids_new)

# plt.plot(frequences_old, ids_old,'.', label = 'id old')
# plt.plot(frequences_set_old, amplitudes_old,'.', label = 'id old')
# plt.plot(frequences_new, ids_new,'.', label = 'id new')
# plt.plot(frequences_set_new, amplitudes_new,'.', label = 'id new')
# plt.legend()
# plt.show()

# incomplete current from data
frequences_set_new.pop(0)
amplitudes_new.pop(0)

# # plot data
plt.plot(frequences_set_old, 20*np.log10(amplitudes_old), label = "Code constants")
plt.plot(frequences_set_new, 20*np.log10(amplitudes_new), label = "Computed constants [Line to Line]")
plt.xlabel("Frequence [Hz]")
plt.ylabel("Gain Current [Db]")
plt.title("Gain Current Bode Plot")
plt.xscale("log")
plt.grid()
plt.legend()
plt.show()