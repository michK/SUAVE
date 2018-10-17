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


def unified_propsys(mdotm, mdote, PKm, PKe, fL, fS):
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
            PKm - array of mechanical propulsive power streams [W]
            PKe - array of electrical propulsive power streams [W]

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

    # Fan and nacelle weights
    m_fanm = (Kfan * mdotm**1.2).sum()
    m_nacm = (cmnace * Knace * mdotm).sum()
    m_fane = (Kfan * mdote**1.2).sum()
    m_nace = (cmnace * Knace * mdote).sum()

    # Fan powers
    p_mfan = PKm / eta_fan
    p_efan = PKe / eta_fan

    # Propulsive motor weights
    p_mot = p_efan / eta_mot
    m_prop_mot = (p_mot / pm_mot).sum()

    # Propulsive motor power electronics weights
    p_pe_prop_mot = p_mot.sum() / eta_pe
    m_pe_prop_mot = p_pe_prop_mot / pm_pe

    # NOTE - Everything correct up to here

    # Determine link properties - generator or motor
    if ((1 - fS) * fL) > (eta_pe * eta_mot * fS * (1 - fL)):
        arch = 'SeriesPartialHybrid'  # Link is generator
    else: # parallel partial, fL,fS specified
        arch = 'ParallelPartialHybrid'  # Link is motor

    if arch == 'SeriesPartialHybrid':
        p_link = p_pe_prop_mot - p_bat
        p_pe_link = p_link / eta_pe
        p_gen_link = p_pe_link / eta_mot  # TODO - split this up into array of individual generators

        p_core = p_mot_link_tot + 


    # Turbine core weights
    p_core = p_mfan
    p_core_tot = p_core.sum()
    mdot_core = p_core / c_core
    m_core = (Kcore * mdot_core**1.2).sum()

    # Battery sizing - Size based on power requirement for now. NOTE - Figure out how to do this properly
    p_bat = fS / (1 - fS) * p_core_tot
    m_bat = p_bat / p_sbat


    import pdb; pdb.set_trace()  # breakpoint 6fef5f72 //


    mprop = m_core + m_fanm + m_fane + m_nacm + m_nace + m_prop_mot + m_pe_prop_mot + \
            m_bat + m_mot_link + m_pe_link

    # mprop = nr_eng * (mcore + mgen + mrect + mmfan + mmnace) +
    # nr_elec_fans * (minv + mmot + mefan + menace) + mTMS
    import pdb; pdb.set_trace()  # breakpoint ea000030 //


    weight_propsys = mprop * 9.81

    return weight_propsys
