import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from calculate_powers import calculate_powers

PKtot = 300e3
FL = np.linspace(0,1,50)
FS = np.linspace(0,1,50)
eta_pe = 0.98
eta_mot = 0.95
eta_fan = 0.9

PKe = np.zeros((50, 50))
PKm = np.zeros((50, 50))
PfanE = np.zeros((50, 50))
PfanM = np.zeros((50, 50))
Pmot = np.zeros((50, 50))
Pinv = np.zeros((50, 50))
Pbat = np.zeros((50, 50))
Pturb = np.zeros((50, 50))
Pgen = np.zeros((50, 50))
Pconv = np.zeros((50, 50))
Plink = np.zeros((50, 50))

for i, fL in enumerate(FL):
    for j, fS in enumerate(FS):
        vars_out = calculate_powers(PKtot, fL, fS, eta_pe, eta_mot, eta_fan)

        PKe[i, j] = vars_out[0]
        PKm[i, j] = vars_out[1]
        PfanE[i, j] = vars_out[2]
        PfanM[i, j] = vars_out[3]
        Pmot[i, j] = vars_out[4]
        Pinv[i, j] = vars_out[5]
        Pbat[i, j] = vars_out[6]
        Pturb[i, j] = vars_out[7]
        Pgen[i, j] = vars_out[8]
        Pconv[i, j] = vars_out[9]
        Plink[i, j] = vars_out[10]

fL = np.linspace(0,1,50)
fS = np.linspace(0,1,50)
FL, FS = np.meshgrid(fL, fS)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# ax.plot_surface(FL, FS, PKe)
# ax.plot_surface(FL, FS, PKm)
# ax.plot_surface(FL, FS, PfanE)
# ax.plot_surface(FL, FS, PfanM)
# ax.plot_surface(FL, FS, Pmot)
# ax.plot_surface(FL, FS, Pinv)
# ax.plot_surface(FL, FS, Pbat)
# ax.plot_surface(FL, FS, Pturb)
ax.plot_surface(FL, FS, Pgen)  # Non-smooth
# ax.plot_surface(FL, FS, Pconv)  # Non-smooth
# ax.plot_surface(FL, FS, Plink)  # Non-smooth

ax.set_xlabel('fL')
ax.set_ylabel('fS')
ax.set_zlabel('Pgen')

plt.show()
