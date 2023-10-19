import matplotlib.pyplot as plt
import numpy as np
from utils import get_data

path = "./"
filename = "current_dc.txt"
times, vsupplys, ias, ibs, ics, dcas, dcbs, dccs, dc_activated = get_data(filename, path)

# dcas = [dcas[i] * 4 for i in range(len(dcas))]

vas = [dcas[i]*vsupplys[i] for i in range(len(dcas))]
vbs = [dcbs[i]*vsupplys[i] for i in range(len(dcbs))]
vcs = [dccs[i]*vsupplys[i] for i in range(len(dccs))]

fig, ax = plt.subplots(2)
ax[0].plot(dcas, ias, '.', label = "ias")
ax[0].plot(dcas, ibs, label = "ibs")
ax[0].plot(dcas, ics, '.', label = "ics")
# ax[0].plot(vas, ias, '.', label = "ias")
# ax[0].plot(dcbs, ibs, '.', label = "ibs")
# ax[0].plot(dccs, ics, '.', label = "ics")
ax[0].set_title(f"Tension X Current")
ax[0].set_xlabel("Duty Cicle")
ax[0].set_ylabel("Current [A]")
ax[0].grid()
ax[0].legend()


# ax[1].plot(times, vas, '.', label = "va")
# ax[1].plot(times, vbs, '.', label = "vb")
# ax[1].plot(times, vcs, '.', label = "vc")
ax[1].plot(times, vsupplys, '.', label = "vsupplys")
# ax[1].plot(times, vsupplys, '.', label = "vsupplys")
ax[1].legend()
plt.show()

### Calculate Resistance
# Take out non linear piece
init_dc = 0.05
for i in range(len(dcas)):
    # if dcas[i] >= init_dc:
    if ias[i] >= 4:
        init_dc_index = i
        break

for i in range(init_dc_index, len(dc_activated)):
    if dc_activated[i] == 1:
        end_dc_index = i
        break
    elif ias[i] > 5:
        end_dc_index = i
        break
    else:
        end_dc_index = -1


dcas = dcas[init_dc_index:end_dc_index]
vas = vas[init_dc_index:end_dc_index]
ias = ias[init_dc_index:end_dc_index]

def lstq(x, y):
    n = len(x)
    
    # Calcular as médias de x e y
    x_medio = sum(x) / n
    y_medio = sum(y) / n

    # Calcular os coeficientes da reta m e b
    numerador_m = sum((x[i] - x_medio) * (y[i] - y_medio) for i in range(n))
    denominador_m = sum((x[i] - x_medio) ** 2 for i in range(n))
    
    m = numerador_m / denominador_m
    b = y_medio - m * x_medio

    return m, b

# Compute resistance by the tangent
m, offset = lstq(ias, vas)
xs = np.linspace(ias[0], ias[-1])
plt.plot(ias, vas, '.', label = "vas")
plt.plot(xs, xs*m + offset, label = f"i = {round(m, 2)}u + {round(offset, 2)}")
plt.title(f"Tension X Current")
plt.xlabel("Tension [V] = (Duty cicle * V supply)")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
plt.show()

res_eq = m
r1 = 2*res_eq/3

print("Resistance equivalent and phase: ", round(res_eq, 2), round(r1, 2))
print("Multimetre: 0.37 " + "error = ", (0.37-r1) / 0.37)

