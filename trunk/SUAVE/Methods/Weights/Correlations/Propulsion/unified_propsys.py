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
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers
from SUAVE.Methods.Utilities.soft_max import soft_max

# package imports
import numpy as np

# ----------------------------------------------------------------------
#   Unified Propulsion System
# ----------------------------------------------------------------------

# @ingroup Methods-Weights-Correlations-Propulsion


def unified_propsys(vehicle, PKtot, mdottot, weight_factor=1):
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

    # TODO Add loop that loops over fS and fL
    # and calculates max power/mass flow required
    # by different components

    # Create arrays of fL and fS to loop over
    fL_arr = np.array([vehicle.fL_climb, vehicle.fL_cruise, vehicle.fL_descent])
    fS_arr = np.array([vehicle.fS_climb, vehicle.fS_cruise, vehicle.fS_descent])

    # Create empty lists to store tentative component weights

    m_fanm_store        = []
    m_nacm_store        = []
    m_fane_store        = []
    m_nace_store        = []
    m_core_store        = []
    m_prop_mot_store    = []
    m_pe_prop_mot_store = []
    m_gen_store         = []
    m_pe_link_store     = []
    m_tms_store         = []
    
    for fL in fL_arr:
        for fS in fS_arr:

            # Ensure mass flows are arrays
            mdotm = (1 - fL) * mdottot
            mdote = fL * mdottot

            # Sizing constants from LEARN.
            Kcore  = 45.605
            Kfan   = 1.2972
            Knace  = 4.5641
            cmnace = 1.0
            c_core = 400.0 * Units['kJ/kg']
            pm_mot = 8.0 * Units['hp/lb']
            pm_pe  = 10.0 * Units['hp/lb']
            pm_tms = 8.0 * Units['hp/lb']

            # Constants
            # Assumed efficiencies
            eta_fan = 0.9  # Fan
            eta_mot = 0.95  # Motor
            eta_pe  = 0.98  # Power electronics
            eta_bat = 0.5  # For sizing condition battery is at max power, thus eta = 0.5

            # Fan and nacelle weights
            m_fanm = (Kfan * mdotm**1.2).sum()
            m_nacm = (cmnace * Knace * mdotm).sum()
            m_fane = (Kfan * mdote**1.2).sum()
            m_nace = (cmnace * Knace * mdote).sum()

            # Calculate powers  NOTE - Negative values set to zeros
            [PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pgenmot, Pconv, Plink] = \
            calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan)

            mdot_core = Pturb / c_core

            m_core        = Kcore * mdot_core**1.2
            m_prop_mot    = Pmot / pm_mot
            m_pe_prop_mot = Pinv / pm_pe
            m_gen         = Pgenmot / pm_mot
            m_pe_link     = Pconv / pm_pe            

            # Thermal management system
            q_bat  = (1.0 - eta_bat) * Pbat
            q_gen  = (1.0 - eta_mot) * Pgenmot
            q_conv = (1.0 - eta_pe)  * Pconv
            q_inv  = (1.0 - eta_pe)  * Pinv
            q_mot  = (1.0 - eta_mot) * Pmot
            
            q_tot  = q_bat + q_gen + q_conv + q_inv + q_mot

            mass_tms = q_tot / pm_tms

            # Add calculated mass to storage lists
            m_fanm_store.append(m_fanm)
            m_nacm_store.append(m_nacm)
            m_fane_store.append(m_fane)
            m_nace_store.append(m_nace)
            m_core_store.append(m_core)
            m_prop_mot_store.append(m_prop_mot)
            m_pe_prop_mot_store.append(m_pe_prop_mot)
            m_gen_store.append(m_gen)
            m_pe_link_store.append(m_pe_link)
            m_tms_store.append(mass_tms)

        # Find max values of component masses
        m_fanm =        soft_max(np.sort(m_fanm_store)[-1], np.sort(m_fanm_store)[-2])
        m_nacm =        soft_max(np.sort(m_nacm_store)[-1], np.sort(m_nacm_store)[-2])
        m_fane =        soft_max(np.sort(m_fane_store)[-1], np.sort(m_fane_store)[-2])
        m_nace =        soft_max(np.sort(m_nace_store)[-1], np.sort(m_nace_store)[-2])
        m_core =        soft_max(np.sort(m_core_store)[-1], np.sort(m_core_store)[-2])
        m_prop_mot =    soft_max(np.sort(m_prop_mot_store)[-1], np.sort(m_prop_mot_store)[-2])
        m_pe_prop_mot = soft_max(np.sort(m_pe_prop_mot_store)[-1], np.sort(m_pe_prop_mot_store)[-2])
        m_gen =         soft_max(np.sort(m_gen_store)[-1], np.sort(m_gen_store)[-2])
        m_pe_link =     soft_max(np.sort(m_pe_link_store)[-1], np.sort(m_pe_link_store)[-2])
        m_tms =         soft_max(np.sort(m_tms_store)[-1], np.sort(m_tms_store)[-2])

        mprop = m_core + m_fanm + m_fane + m_nacm + m_nace + m_prop_mot + m_pe_prop_mot + \
                m_gen + m_pe_link + mass_tms

        mass_propsys = mprop * weight_factor

    return mass_propsys
