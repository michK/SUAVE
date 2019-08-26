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


def unified_propsys(vehicle, PKtot, weight_factor=1.0):
    """ Calculate the weight of the entire propulsion system

    Assumptions:
            N/A

    Source:
            Kruger, Michael, et al.
            "Electrified Aircraft Trade-Space Exploration."
            2018 Aviation Technology, Integration, and Operations Conference.
            2018.

    Inputs:            
            PKtot - Total installed PK [W]
            fL    - Load electrification factor [-]
            fS    - Source electrification factor [-]

    Outputs:
            weight - weight of the full propulsion system [kg]

    Properties Used:
            N/A
    """

    # unpack
    propsys = vehicle.propulsors.unified_propsys

    # Create arrays of fL and fS to loop over
    segments = ["Climb", "Cruise", "Descent"]
    fL_arr = np.array([vehicle.fL_climb, vehicle.fL_cruise, vehicle.fL_descent])
    fS_arr = np.array([vehicle.fS_climb, vehicle.fS_cruise, vehicle.fS_descent])

    # NOTE These max statements might make design space non-smooth
    # fL_max = np.amax(fL_arr)
    # fL_max_segment = segments[np.argmax(fL_arr)]
    # fS_max = np.amax(fS_arr)
    # fS_max_segment = segments[np.argmax(fS_arr)]

    fL_max = soft_max(np.sort(fL_arr)[-1], np.sort(fL_arr)[-2])
    fL_max_segment = segments[np.argmax(fL_arr)]
    fS_max = soft_max(np.sort(fS_arr)[-1], np.sort(fS_arr)[-2])
    fS_max_segment = segments[np.argmax(fS_arr)]

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

            # Sizing constants from LEARN.
            Kcore  = 45.605
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

            [PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pgenmot, Pconv, Plink] = \
            calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan)

            # Split power between different components for proper sizing
            PfanM   = PfanM / propsys.number_of_engines_mech
            PfanE   = PfanE / propsys.number_of_engines_elec
            Pmot    = Pmot  / propsys.number_of_engines_elec
            Pinv    = Pinv  / propsys.number_of_engines_elec
            Pturb   = Pturb / propsys.number_of_engines_mech
            Pbat    = Pbat
            Pgenmot = Pgenmot
            Pconv   = Pconv
            Plink   = Plink

            mdot_core = Pturb / c_core

            m_core        = Kcore * mdot_core**1.2
            m_prop_mot    = Pmot / pm_mot
            m_pe_prop_mot = Pinv / pm_pe
            m_gen         = Pgenmot / pm_mot
            m_pe_link     = Pconv / pm_pe

            # Fan weights
            # Mechanical
            kp  = 0.108
            Np  = 1  # nr of propellers in single propulsor unit
            Dp  = propsys.mech_fan_dia / Units.ft
            Pto = PfanM / Units.hp
            Bp  = 10  # Number of blades
            m_fanm = (kp * Np * (Dp * Pto * np.sqrt(Bp))**0.78174).sum() * Units.lbs

            # Electrical
            kp  = 0.108
            Np  = 1  # nr of propellers in single propulsor unit
            Dp  = propsys.elec_fan_dia / Units.ft
            Pto = PfanE / Units.hp
            Bp  = 10  # Number of blades
            m_fane = (kp * Np * (Dp * Pto * np.sqrt(Bp))**0.78174).sum() * Units.lbs

            # Nacelle weights
            # Mechanical
            Kng  = 1.017  # For pylon mounted nacelle (i.e. mechanical in this framework)
            NLt  = propsys.nacelle_length_mech / Units.ft
            Nw   = propsys.mech_nac_dia / Units.ft
            Nz   = vehicle.envelope.ultimate_load
            Weng = m_core / Units.lbs
            Kp   = 1.0  # Propeller/fan book-kept separately
            Ktr  = 1.18  # With thrust reverser, 1.0 without
            Wec  = 2.331 * Weng**0.901 * Kp * Ktr
            Nen  = propsys.number_of_engines_mech
            Sn   = propsys.areas_wetted_mech / Units['ft^2']
            m_nacm = (0.6724 * Kng * NLt**0.1 * Nw**0.294 * Nz**0.119 *
                Wec**0.611 * Nen**0.984 * Sn**0.224).sum() * Units.lbs
            # Electrical
            Kng  = 1.0  # For non-pylon mounted nacelle (i.e. electrical in this framework)
            NLt  = propsys.nacelle_length_elec / Units.ft
            Nw   = propsys.elec_nac_dia / Units.ft
            Nz   = vehicle.envelope.ultimate_load
            Weng = m_prop_mot / Units.lbs
            Kp   = 1.0  # Propeller/fan book-kept separately
            Ktr  = 1.0  # With thrust reverser, 1.0 without
            Wec  = 2.331 * Weng**0.901 * Kp * Ktr
            Nen  = propsys.number_of_engines_elec
            Sn   = propsys.areas_wetted_elec / Units['ft^2']
            m_nace = (0.6724 * Kng * NLt**0.1 * Nw**0.294 * Nz**0.119 *
                Wec**0.611 * Nen**0.984 * Sn**0.224).sum() * Units.lbs

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
        m_fanm        = soft_max(np.sort(m_fanm_store)[-1], np.sort(m_fanm_store)[-2])
        m_nacm        = soft_max(np.sort(m_nacm_store)[-1], np.sort(m_nacm_store)[-2])
        m_fane        = soft_max(np.sort(m_fane_store)[-1], np.sort(m_fane_store)[-2])
        m_nace        = soft_max(np.sort(m_nace_store)[-1], np.sort(m_nace_store)[-2])
        m_core        = soft_max(np.sort(m_core_store)[-1], np.sort(m_core_store)[-2])
        m_prop_mot    = soft_max(np.sort(m_prop_mot_store)[-1], np.sort(m_prop_mot_store)[-2])
        m_pe_prop_mot = soft_max(np.sort(m_pe_prop_mot_store)[-1], np.sort(m_pe_prop_mot_store)[-2])
        m_gen         = soft_max(np.sort(m_gen_store)[-1], np.sort(m_gen_store)[-2])
        m_pe_link     = soft_max(np.sort(m_pe_link_store)[-1], np.sort(m_pe_link_store)[-2])
        m_tms         = soft_max(np.sort(m_tms_store)[-1], np.sort(m_tms_store)[-2])

        mprop = propsys.number_of_engines_mech * (m_gen + m_pe_link + m_core + m_fanm + m_nacm) + \
                propsys.number_of_engines_elec * (m_prop_mot + m_pe_prop_mot + m_fane + m_nace) + \
                mass_tms

        propsys.info.m_core         = m_core
        propsys.info.m_fanm         = m_fanm
        propsys.info.m_fane         = m_fane
        propsys.info.m_nacm         = m_nacm
        propsys.info.m_nace         = m_nace
        propsys.info.m_prop_mot     = m_prop_mot
        propsys.info.m_pe_prop_mot  = m_pe_prop_mot
        propsys.info.m_gen          = m_gen
        propsys.info.m_pe_link      = m_pe_link
        propsys.info.mass_tms       = mass_tms
        propsys.info.weight_factor  = weight_factor
        propsys.info.weight_total   = mprop * weight_factor
        propsys.info.fS_max         = fS_max
        propsys.info.fL_max_segment = fL_max_segment
        propsys.info.fL_max         = fL_max
        propsys.info.fS_max_segment = fS_max_segment

        mass_propsys = mprop * weight_factor

    return mass_propsys
