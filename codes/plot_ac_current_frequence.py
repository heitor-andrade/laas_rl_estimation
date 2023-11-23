import matplotlib.pyplot as plt
import numpy as np
from utils import get_data

path = "./"
filename = "current_ac.txt"
times, vsupplys, ias, dcas, mean_currents, mean_tensions, frequences = get_data(filename, path)

_times, _frequences, _ias = [], [], []

for i in range(len(times)):
    if frequences[i] == 106:
        _times.append(times[i])
        _frequences.append(frequences[i])
        _ias.append(ias[i])

fig, ax = plt.subplots(2)
ax[0].plot(_times, _ias, '.',label = "Ia [A]")
ax[0].set_title(f"Time X Current")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Current [A]")
ax[0].grid()
ax[0].legend()


ax[1].plot(_times, _frequences)
ax[1].set_title(f"Time X Frequence")
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Frequence [Hz]")
ax[1].legend()
plt.show()

# filter f = 0
while frequences[0] == 0:
    frequences.pop(0)
    ias.pop(0)

# get amplitude maximum of each frequence max - min / 2
fs = list(set(frequences))
amplitudes = [0] * len(fs)

index_amplitude = 0
last_frequence = frequences[0]
max_i = 0
min_i = 100
for i in range(len(frequences)):

    if frequences[i] != last_frequence:
        amplitude = (max_i - min_i)/2
        amplitudes[index_amplitude] = amplitude
        index_amplitude += 1
        last_frequence = frequences[i]
        max_i = 0
        min_i = 100
    
    if ias[i] > max_i:
        max_i = ias[i]
    
    if ias[i] < min_i:
        min_i = ias[i]

amplitude = (max_i - min_i)/2
amplitudes[index_amplitude] = amplitude
# get first amplitude, f = 100
a0 = amplitudes[0]

# calculate gain in amplitude, amplitude / a0
gain_currents = [amp/a0 for amp in amplitudes]
fs.sort()


# Compute inductance with motor data
R = 0.242
F_DESIRED = 500

for i in range(len(fs)):
    if fs[i] > F_DESIRED:
        index_f = i
        break
f = fs[index_f]
gain_current = gain_currents[index_f]

resistance = R
impedance_ac = resistance/gain_current                      # impedance gain is the invese of impedan
reactance = np.sqrt(impedance_ac**2 - resistance**2)
inductance = reactance / (2 * np.pi * f)

print(inductance)

# Compute Bode with inductance found in data
def compute_impedance(R, f, L = 151e-6):
    return np.sqrt(R**2 + (2*np.pi*f*L)**2)
L = inductance
Zs = [compute_impedance(R, f, L) for f in fs]
U = 1
currents_ac = [U/Z for Z in Zs]
current_dc = U/R
gain_currents_theorical = [current/current_dc for current in currents_ac]


# Compute Bode with inductance found in code
INDUCTANCE_CODE = 70e-6
R = 0.242
current_dc = U/R
Zs = [compute_impedance(R, f, INDUCTANCE_CODE) for f in fs]
currents_ac = [U/Z for Z in Zs]
gain_currents_code = [current/current_dc for current in currents_ac]

# Plot data
plt.plot(fs, 20*np.log10(gain_currents), label = "Data")
plt.plot(fs, 20*np.log10(gain_currents_theorical), label = f"Theorical with inductance of data. L = {round(L*1e6)}e-6")
plt.plot(fs, 20*np.log10(gain_currents_code), label = f"Theorical with inductance of code. L = {round(INDUCTANCE_CODE * 1e6)}e-6")
plt.xlabel("Frequence [Hz]")
plt.ylabel("Gain Current [Db]")
plt.title("Gain Current Bode Plot")
plt.xscale("log")
plt.grid()
plt.legend()
plt.show()

inductances = []
for i in range(len(fs)):
    impedance_ac = resistance/gain_currents[i]                      # impedance gain is the invese of impedan
    reactance = np.sqrt(impedance_ac**2 - resistance**2)
    inductance = reactance / (2 * np.pi * fs[i])
    inductances.append( inductance )

# plt.plot(fs, inductances)
# plt.show()