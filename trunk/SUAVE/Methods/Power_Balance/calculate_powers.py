# @ingroup Methods-Power_Balance
# calculate_powers.py
#
# Created:  Oct 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np

# ----------------------------------------------------------------------
#   Unified Propulsion System - Powers
# ----------------------------------------------------------------------

# @ingroup Methods-Power_Balance

def calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan):
    # Determine link properties - generator or motor
    if ((1 - fS) * fL) > (eta_pe * eta_mot * fS * (1 - fL)):
        arch = 'SeriesPartialHybrid'  # Link is generator
    else: # parallel partial, fL,fS specified
        arch = 'ParallelPartialHybrid'  # Link is motor

    # System of equations for link as generator
    if arch == 'SeriesPartialHybrid':
        A = np.array((
                     [1,          1,          0,          0, 0,         0, 0,       0, 0, 0,          0],
                     [-1/eta_fan, 0,          1,          0, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          -1/eta_mot, 0, 1,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, -1/eta_pe, 1, 0,       0, 0, 0,          0],
                     [0,          -1/eta_fan, 0,          1, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, 0,         0, 0,       0, 0, 1,          -1],
                     [0,          0,          0,          0, 0,         0, 0,       0, 1, -1/eta_mot, 0],
                     [0,          0,          0,          0, 0,         1, -1,      0, 0, 0,          -1],
                     [0,          0,          0,          1, 0,         0, 0,      -1, 1, 0,          0],
                     [0,          0,          0,          0, 0,         0, (fS-1), fS, 0, 0, 0],
                     [(fL-1),     fL,         0,          0, 0,         0, 0,       0, 0, 0, 0]
                    ))

        b = np.array((
                     [PKtot],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                    ))

        # Solve system
        [PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pgen, Pconv, Plink] = np.linalg.solve(A, b)

        return PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pgen, Pconv, Plink

    else:
        A = np.array((
                     [1,          1,          0,          0, 0,         0, 0,       0, 0, 0,          0],
                     [-1/eta_fan, 0,          1,          0, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          -1/eta_mot, 0, 1,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, -1/eta_pe, 1, 0,       0, 0, 0,          0],
                     [0,          -1/eta_fan, 0,          1, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, 0,         0, 0,       0, 0, 1,          -1/eta_pe],
                     [0,          0,          0,          0, 0,         0, 0,       0, 1, -eta_mot,   0],
                     [0,          0,          0,          0, 0,         1, -1,      0, 0, 0,          1],
                     [0,          0,          0,          1, 0,        0, 0,       -1,-1, 0,          0],
                     [0,          0,          0,          0,  0,        0, (fS-1), fS, 0, 0, 0],
                     [(fL-1),     fL,         0,          0,  0,        0, 0,       0, 0, 0, 0]
                    ))

        b = np.array((
                     [PKtot],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                    ))

        # Solve system
        [PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pmot_link, Pconv, Plink] = np.linalg.solve(A, b)

        return PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pmot_link, Pconv, Plink
