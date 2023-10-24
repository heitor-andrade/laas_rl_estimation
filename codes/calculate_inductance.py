import numpy as np

dci = 2.481
aci = 2.373
dcv = 0.943

acv = dcv

dc_impedance = dcv/dci

ac_impedance = acv/aci

reactance = np.sqrt(ac_impedance**2 - dc_impedance**2)

f = 1000

inductance = reactance / (2 * np.pi * f)

print(inductance, dc_impedance)