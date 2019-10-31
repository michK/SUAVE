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


def unified_propsys(vehicle, weight_factor=1.0):
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
            mass - mass of the full propulsion system [kg]

    Properties Used:
            N/A
    """

    # unpack
    propsys = vehicle.propulsors.unified_propsys
    PKtot = vehicle.PKtot
    mdottot = vehicle.mdottot

    # Create arrays of fL and fS to loop over
    segments = ["Climb", "Cruise", "Descent"]
    fL_arr = np.unique(np.array([vehicle.fL_climb, vehicle.fL_cruise, vehicle.fL_descent]))
    fS_arr = np.unique(np.array([vehicle.fS_climb, vehicle.fS_cruise, vehicle.fS_descent]))

    # Find segment electrifications for result reporting
    fL_max = np.amax(np.array([vehicle.fL_climb, vehicle.fL_cruise, vehicle.fL_descent]))
    fL_max_segment = segments[np.argmax(fL_arr)]
    fS_max = np.amax(np.array([vehicle.fS_climb, vehicle.fS_cruise, vehicle.fS_descent]))
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

    m_acc_store         = []
    m_air_store         = []
    m_exhaust_store     = []
    m_oilsys_store      = []
    m_thrust_rev_store  = []

    for fL in fL_arr:
        for fS in fS_arr:

            # Sizing constants from LEARN.
            Kcore  = 45.605
            Kfan = 1.3
            c_core = 400.0 * Units['kJ/kg']
            pm_mot = 16  * Units['kW/kg']
            pm_pe  = 19  * Units['kW/kg']
            pm_tms = 13.2  * Units['kW/kg']

            # Constants
            # Assumed efficiencies
            eta_fan = propsys.eta_fan
            eta_mot = propsys.eta_mot
            eta_pe  = propsys.eta_pe
            eta_bat = 0.5  # For sizing condition battery is at max power, thus eta = 0.5  FIXME This isn't always true

            # Find mass flows
            mdotm = (1 - fL) * mdottot
            mdote = fL * mdottot

            [PKm, PKe, Pturb, Pbat, PfanM, PfanE, Pmot, Pinv, Plink] = \
            calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan)

            # Check if serial or parallel  NOTE Perhaps this can be implemented directly in model?
            if Plink >= 0:  # Parallel
                Pconv = Plink * eta_pe
                Pgenmot = Plink * eta_pe * eta_mot
            else:  # Series
                Pgenmot = abs(Plink) / eta_mot / eta_pe
                Pconv = abs(Plink) / eta_mot

            # Split power between different components for proper sizing, catching missing propulsors
            try:
                PfanM  = PfanM / propsys.nr_engines_mech
                Pturb  = Pturb / propsys.nr_engines_mech
            except ZeroDivisionError:
                PfanM  = 0.0
                Pturb  = 0.0

            try:
                PfanE  = PfanE / propsys.nr_engines_elec
                Pmot   = Pmot  / propsys.nr_engines_elec
                Pinv   = Pinv  / propsys.nr_engines_elec
            except ZeroDivisionError:
                PfanE  = 0.0
                Pmot   = 0.0
                Pinv   = 0.0

            try:
                mdotm = mdotm / propsys.nr_engines_mech
            except ZeroDivisionError:
                mdotm = 0.0

            try:
                mdote = mdote / propsys.nr_engines_elec
            except ZeroDivisionError:
                mdote = 0.0

            # These remain unchanged since there is only one of each
            Pbat    = Pbat
            Pgenmot = Pgenmot
            Pconv   = Pconv
            Plink   = Plink

            m_core = (1.67 * (Pturb / Units.hp)**0.803) * Units.lbs  # Raymer - p.323 (5th Ed.)

            m_prop_mot    = Pmot / pm_mot
            m_pe_prop_mot = Pinv / pm_pe
            m_gen         = Pgenmot / pm_mot
            m_pe_link     = Pconv / pm_pe

            #########################
            # Mechanical propulsors #
            #########################
            if propsys.is_turboprop: # Turboprop mechanical propulsors
                # Fan/propeller weights
                kp  = 0.108
                Np  = 1  # nr of propellers in single propulsor unit
                Dp  = propsys.mech_fan_dia / Units.ft
                # Dp  = 2.6 / Units.ft
                Pto = PfanM / Units.hp
                Bp  = 3  # Number of blades
                m_fanm = (kp * Np * (Dp * Pto * np.sqrt(Bp))**0.78174).sum() * Units.lbs
                # Nacelle weights
                Kng  = 1.017  # For pylon mounted nacelle (i.e. mechanical in this framework)
                NLt  = propsys.nacelle_length_mech / Units.ft
                Nw   = propsys.mech_nac_dia / Units.ft
                Nz   = vehicle.envelope.ultimate_load
                Weng = m_core / Units.lbs
                Kp   = 1.0  # Propeller/fan book-kept separately
                Ktr  = 1.18  # 1.18 with thrust reverser, 1.0 without
                Wec  = 2.331 * Weng**0.901 * Kp * Ktr
                Nen  = 1
                Sn   = propsys.areas_wetted_mech / Units['ft^2']
                m_nacm = (0.6724 * Kng * NLt**0.1 * Nw**0.294 * Nz**0.119 *
                    Wec**0.611 * Nen**0.984 * Sn**0.224).sum() * Units.lbs  # Raymer - p.589 (5th Ed.)
            else: # Turbofan mechanical propulsors
                # Fan weights
                m_fanm = 0.1902 * (mdotm / Units['lbs/s'])**1.143 * (1351/1000)**2 * (1 - 0.406**2) * Units.lbs  # From Waters, 1997
                # Nacelle weights
                Kng  = 1.017  # For pylon mounted nacelle (i.e. mechanical in this framework)
                NLt  = propsys.nacelle_length_mech / Units.ft
                Nw   = propsys.mech_nac_dia / Units.ft
                Nz   = vehicle.envelope.ultimate_load
                Weng = m_core / Units.lbs
                Kp   = 1.0  # Propeller/fan book-kept separately
                Ktr  = 1.18  # 1.18 with thrust reverser, 1.0 without
                Wec  = 2.331 * Weng**0.901 * Kp * Ktr
                Nen  = 1
                Sn   = propsys.areas_wetted_mech / Units['ft^2']
                m_nacm = (0.6724 * Kng * NLt**0.1 * Nw**0.294 * Nz**0.119 *
                    Wec**0.611 * Nen**0.984 * Sn**0.224).sum() * Units.lbs  # Raymer - p.589 (5th Ed.)

            #######################################
            # Electrical propulsors - Podded fans #
            #######################################
            # Fan weights
            m_fane = 0.1902 * (mdote / Units['lbs/s'])**1.143 * (1351/1000)**2 * (1 - 0.406**2) * Units.lbs  # From waters - 1997
            # Nacelle weights            
            Kng  = 1.0  # For non-pylon mounted nacelle (i.e. electrical in this framework)
            NLt  = propsys.nacelle_length_elec / Units.ft
            Nw   = propsys.elec_nac_dia / Units.ft
            Nz   = vehicle.envelope.ultimate_load
            Weng = m_prop_mot / Units.lbs
            Kp   = 1.0  # Propeller/fan book-kept separately
            Ktr  = 1.0  # 1.18 with thrust reverser, 1.0 without
            Wec  = 2.331 * Weng**0.901 * Kp * Ktr
            Nen  = propsys.nr_engines_elec
            Sn   = propsys.areas_wetted_elec / Units['ft^2']
            m_nace = (0.6724 * Kng * NLt**0.1 * Nw**0.294 * Nz**0.119 *
                Wec**0.611 * Nen**0.984 * Sn**0.224).sum() * Units.lbs  # Raymer - p.589 (5th Ed.)

            # Thermal management system
            q_bat  = (1.0 - eta_bat) * Pbat
            q_gen  = (1.0 - eta_mot) * Pgenmot
            q_conv = (1.0 - eta_pe)  * Pconv
            q_inv  = (1.0 - eta_pe)  * Pinv
            q_mot  = (1.0 - eta_mot) * Pmot

            q_tot  = q_bat + q_gen + q_conv + q_inv + q_mot
            if (fS <= 0.01) and (fL <= 0.01):  # i.e. if conventional
                q_tot = 0

            mass_tms = q_tot / pm_tms

            if propsys.is_turboprop:
                m_acc = 0.181 * propsys.nr_engines_mech * (Pturb / 1000)**0.8
                m_air = 0  # Included in nacelle
                m_exhaust = 0.1 * propsys.areas_wetted_mech * 14.63  # Assuming exhaust is 10% of nacelle area
                m_oilsys = 0.07 * propsys.nr_engines_mech * m_core
                m_thrust_rev = 0

            else:
                mdot_fuel = vehicle.Cp * Pturb
                m_acc = 36 * propsys.nr_engines_mech * mdot_fuel
                m_air = 0  # Included in nacelle for podded propulsors
                m_exhaust = 0.1 * propsys.areas_wetted_mech * 14.63  # Assuming exhaust is 10% of nacelle area
                m_oilsys = 0.07 * propsys.nr_engines_mech * m_core
                m_thrust_rev = 0.18 * propsys.nr_engines_mech * m_core

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
            m_acc_store.append(m_acc)
            m_air_store.append(m_air)
            m_exhaust_store.append(m_exhaust)
            m_oilsys_store.append(m_oilsys)
            m_thrust_rev_store.append(m_thrust_rev)

    # Find max values of component masses
    m_fanm        = np.amax(np.atleast_1d(m_fanm_store))
    m_nacm        = np.amax(np.atleast_1d(m_nacm_store))
    m_fane        = np.amax(np.atleast_1d(m_fane_store))
    m_nace        = np.amax(np.atleast_1d(m_nace_store))
    m_core        = np.amax(np.atleast_1d(m_core_store))
    m_prop_mot    = np.amax(np.atleast_1d(m_prop_mot_store))
    m_pe_prop_mot = np.amax(np.atleast_1d(m_pe_prop_mot_store))
    m_gen         = np.amax(np.atleast_1d(m_gen_store))
    m_pe_link     = np.amax(np.atleast_1d(m_pe_link_store))
    m_tms         = np.amax(np.atleast_1d(m_tms_store))
    m_acc         = np.amax(np.atleast_1d(m_acc_store))
    m_air         = np.amax(np.atleast_1d(m_air_store))
    m_exhaust     = np.amax(np.atleast_1d(m_exhaust_store))
    m_oilsys      = np.amax(np.atleast_1d(m_oilsys_store))
    m_thrust_rev  = np.amax(np.atleast_1d(m_thrust_rev_store))

    m_bare = propsys.nr_engines_mech * (m_gen + m_pe_link + m_core + m_fanm + m_nacm) + \
             propsys.nr_engines_elec * (m_prop_mot + m_pe_prop_mot + m_fane + m_nace) + \
             mass_tms

    m_add = m_acc + m_air + m_exhaust + m_oilsys + m_thrust_rev

    m_trans = 0.2 * (m_gen + m_pe_link + m_prop_mot + m_pe_prop_mot)

    mprop = m_bare + m_add + m_trans

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
    propsys.info.m_add          = m_add
    propsys.info.m_trans        = m_trans
    propsys.info.weight_factor  = weight_factor
    propsys.info.weight_total   = mprop * weight_factor
    propsys.info.fS_max         = fS_max
    propsys.info.fL_max_segment = fL_max_segment
    propsys.info.fL_max         = fL_max
    propsys.info.fS_max_segment = fS_max_segment

    mass_propsys = mprop * weight_factor

    return mass_propsys
