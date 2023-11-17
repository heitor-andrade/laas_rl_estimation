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

ias_multi = [0.587, 0.666, 0.744, 0.821, 0.897]
dcas_multi = [0.0275, 0.03, 0.0325, 0.0350, 0.0375]

ias_multi2 = [1.448, 1.797, 2.176, 2.528, 3.25, 3.6, 3.96, 4.33]
dcas_multi2 = [0.05, 0.06, 0.07, 0.08, 0.1, 0.11, 0.12, 0.13]


plt.plot(dcas, ias, '.', label = "Current")
plt.plot(dcas_multi + dcas_multi2, ias_multi + ias_multi2, label = "Current multimeter")
# plt.plot(dcas, ibs, label = "ibs")
# plt.plot(dcas, ics, '.', label = "ics")
plt.title(f"Duty Cicle X Current")
plt.xlabel("Duty Cicle")
plt.ylabel("Current [A]")
plt.grid()
plt.legend()
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
plt.title(f"Current x Tension")
plt.ylabel("Tension [V] = (Duty cicle * V supply)")
plt.xlabel("Current [A]")
plt.grid()
plt.legend()
plt.show()

dx = ias[-1] - ias[0]
dy = vas[-1] - vas[0]

res_eq = dy/dx
r1 = 2*res_eq/3

print("Resistance equivalent and phase: ", round(res_eq, 2), round(r1, 2))
print("Multimetre: 0.37 " + "error = ", (0.37-r1) / 0.37)

