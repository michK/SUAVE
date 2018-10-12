# @ingroup Methods-Weights-Correlations-Propulsion
# unified_propsys.py
#
# Created:  Oct 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# ----------------------------------------------------------------------
#   Integrated Propulsion
# ----------------------------------------------------------------------

# @ingroup Methods-Weights-Correlations-Propulsion


def unified_propsys(mdotm, mdote, mdot_core):
    """ Calculate the weight of the entire propulsion system

    Assumptions:
            N/A       

    Source: 
        Kruger, Michael, et al.
        "Electrified Aircraft Trade-Space Exploration."
        2018 Aviation Technology, Integration, and Operations Conference.
        2018.

    Inputs:
            TODO - Populate

    Outputs:
            weight - weight of the full propulsion system                                     [kilograms]

    Properties Used:
            N/A
    """
    # Sizing constants from LEARN. FIXME - Some of these should maybe be specified in inputs
    Kcore = 45.605
    Kfan = 1.2972
    Knace = 4.5641
    cmnace = 1.0

    # Constants FIXME - Some of these should maybe be specified in inputs
    # Assumed efficiencies
    eta_fan = 0.9  # Fan
    eta_mot = 0.95  # Motor
    eta_pe = 0.98  # Power electronics   
    Psp = 400.0 * Units['kJ/kg']

    # Core weight
    mcore = (Kcore * mdot_core**1.2).sum()

    # Mechanical subsystem weights
    mfanm = (Kfan * mdotm**1.2).sum()
    mnacm = (cmnace * Knace * mdotm).sum()

    # Electrical subsystem weights
    mfane = (Kfan * mdote**1.2).sum()
    mnace = (cmnace * Knace * mdote).sum()

    # Calculate turbine core requirements
    Pmfan = PKm / eta_fan
    Pcore = Pmfan
    mdot_core = Pcore / Psp

    mgen = Pgen / PMgen
    mrect = Prect / PMrect
    minv = Pinv / PMinv
    mmot = Pmot / PMmmot

    mprop = mcore + mfanm + mfane + mnacm + mnace + mgen + mrect + minv + mmot

    # mprop = nr_eng * (mcore + mgen + mrect + mmfan + mmnace) +
    # nr_elec_fans * (minv + mmot + mefan + menace) + mTMS

    weight_propsys = mprop * 9.81

    return weight_propsys
