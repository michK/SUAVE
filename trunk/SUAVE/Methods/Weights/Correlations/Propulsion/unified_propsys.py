# @ingroup Methods-Weights-Correlations-Propulsion
# unified_propsys.py
#
# Created:  Oct 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
from SUAVE.Core import Units

# package imports
import numpy as np

# ----------------------------------------------------------------------
#   Integrated Propulsion
# ----------------------------------------------------------------------

# @ingroup Methods-Weights-Correlations-Propulsion


def unified_propsys(mdotm, mdote, PKtot, fL, fS, weight_factor=1.3):
    """ Calculate the weight of the entire propulsion system

    Assumptions:
            N/A

    Source:
        Kruger, Michael, et al.
        "Electrified Aircraft Trade-Space Exploration."
        2018 Aviation Technology, Integration, and Operations Conference.
        2018.

    Inputs:
            mdotm - array of mechanical fan mass flows [kg/s]
            mdote - array of electrical fan mass flows [kg/s]
            PKtot - Total installed PK [W]
            fL    - Load electrification factor [-]
            fS    - Source electrification factor [-]

    Outputs:
            weight - weight of the full propulsion system [kg]

    Properties Used:
            N/A
    """
    # Sizing constants from LEARN. FIXME - Some of these should maybe be specified in inputs
    Kcore = 45.605
    Kfan = 1.2972
    Knace = 4.5641
    cmnace = 1.0
    c_core = 400.0 * Units['kJ/kg']
    pm_mot = 8.0 * Units['hp/lb']
    pm_pe = 10.0 * Units['hp/lb']
    e_sbat = 500.0 * Units['Wh/kg']  # battery specific energy
    p_sbat = 2985.0 * Units['W/kg']  # battery specific power

    # Constants FIXME - Some of these should maybe be specified in inputs
    # Assumed efficiencies
    eta_fan = 0.9  # Fan
    eta_mot = 0.95  # Motor
    eta_pe = 0.98  # Power electronics
    eta_bat = 0.85  # TODO - This should come from Ragone relation

    # Fan and nacelle weights
    m_fanm = (Kfan * mdotm**1.2).sum()
    m_nacm = (cmnace * Knace * mdotm).sum()
    m_fane = (Kfan * mdote**1.2).sum()
    m_nace = (cmnace * Knace * mdote).sum()

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

        mdot_core = Pturb / c_core

        m_core        = Kcore * mdot_core**1.2
        m_prop_mot    = Pmot / pm_mot
        m_pe_prop_mot = Pinv / pm_pe
        m_bat         = Pbat / (p_sbat * eta_bat)
        m_gen         = Pgen / pm_mot
        m_pe_link     = Pconv / pm_pe

        mprop = m_core + m_fanm + m_fane + m_nacm + m_nace + m_prop_mot + m_pe_prop_mot + \
                m_bat + m_gen + m_pe_link

    elif arch == 'ParallelPartialHybrid':
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

        mdot_core = Pturb / c_core

        m_core = Kcore * mdot_core**1.2
        m_prop_mot    = Pmot / pm_mot
        m_pe_prop_mot = Pinv / pm_pe
        m_bat         = Pbat / (p_sbat * eta_bat)
        m_mot_link    = Pmot_link / pm_mot
        m_pe_link     = Pconv / pm_pe

        mprop = m_core + m_fanm + m_fane + m_nacm + m_nace + m_prop_mot + m_pe_prop_mot + \
                m_bat + m_mot_link + m_pe_link

    weight_propsys = mprop * 9.81 * weight_factor

    return weight_propsys
