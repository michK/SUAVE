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

def remove_negatives(data_in):
    """Replace negative values in array with zero"""
    data_out = []
    for i, item in enumerate(data_in):
        if item >= 0:
            data_out.append(item)
        else:
            data_out.append(0.0)
    return data_out


def calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan):
    # Determine link properties - generator or motor
    if ((1 - fS) * fL) > (eta_pe * eta_mot * fS * (1 - fL)):  # Series - Link is generator        
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
        # Solution vars (in order):
        # PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pgen, Pconv, Plink
        sol = np.linalg.solve(A, b)

        return remove_negatives(sol)

    else: # Parallel - Link is motor
        A = np.array((
                     [1,          1,          0,          0, 0,         0, 0,       0, 0, 0,          0],
                     [-1/eta_fan, 0,          1,          0, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          -1/eta_mot, 0, 1,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, -1/eta_pe, 1, 0,       0, 0, 0,          0],
                     [0,          -1/eta_fan, 0,          1, 0,         0, 0,       0, 0, 0,          0],
                     [0,          0,          0,          0, 0,         0, 0,       0, 0, 1,          -1/eta_pe],
                     [0,          0,          0,          0, 0,         0, 0,       0, 1, -eta_mot,   0],
                     [0,          0,          0,          0, 0,         1, -1,      0, 0, 0,          1],
                     [0,          0,          0,          1, 0,         0, 0,       -1,-1, 0,         0],
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
        # Solution vars (in order):
        # PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pmot_link, Pconv, Plink
        sol = np.linalg.solve(A, b)

        return remove_negatives(sol)
