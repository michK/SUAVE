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

    A = np.array((
                 [1,          0,          0,            0,            0,          0,           0,          0,             0],
                 [0,          1,          0,            0,            0,          0,           0,          0,             0],
                 [-1/eta_fan, 0,          0,            0,            1,          0,           0,          0,             0],
                 [0,          -1/eta_fan, 0,            0,            0,          1,           0,          0,             0],
                 [0,          0,          0,            0,            0,          -1/eta_mot,  1,          0,             0],
                 [0,          0,          0,            0,            0,          0,           -1/eta_pe,  1,             0],
                 [0,          0,          -1,           0,            1,          0,           0,          0,             -eta_pe*eta_mot],
                 [0,          0,          0,            -1,           0,          0,           0,          1,             1],
                 [0,          0,          -(fS/(1-fS)), 1,            0,          0,           0,          0,             0],
                ))

    b = np.array((
                 [(1-fL)*PKtot],
                 [fL*PKtot],
                 [0],
                 [0],
                 [0],
                 [0],
                 [0],
                 [0],
                 [0],
                ))

    # PKm, PKe, Pturb, Pbat, PfanM, PfanE, Pmot, Pinv, Plink
    sol = np.linalg.solve(A, b)

    return sol
