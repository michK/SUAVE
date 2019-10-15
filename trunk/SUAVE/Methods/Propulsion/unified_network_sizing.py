# unified_network_sizing.py
#
# Created:  Feb 2018, M. Kruger
# Modified: May 2019, M. Kruger

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np

from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers

def unified_network_sizing(propsys, vehicle, f_KED_wing=0.5):
    """
    This function takes the total mass flow through all propulsors
    and sizes the fans appropriately, based on the number of propulsors

    Method for fan and nacelle sizing from Raymer

    Inputs:
        propsys - Propulsion system model
        vehicle - Vehicle model
        f_KED_wing - Fraction of wing kinetic energy defect relative to full aircraft
    """


    nr_fans_mech = propsys.nr_engines_mech
    nr_fans_elec = propsys.nr_engines_elec

    fL = vehicle.fL_cruise  # Size propulsors for cruise
    fS = vehicle.fS_cruise  # Size propulsors for cruise

    # Check for edge cases where components should 'disappear'
    if fL <= 0.02:  # Propulsion only from mechanical side
        nr_fans_elec = 0
    elif fL >= 0.98:  # Propulsion only from electrical side
        nr_fans_mech = 0

    # Run unified model
    eta_pe = propsys.eta_pe
    eta_mot = propsys.eta_mot
    eta_fan = propsys.eta_fan
    unified_propsys_outputs = calculate_powers(vehicle.PKtot, fS, fL, eta_pe, eta_mot, eta_fan)
    Pturb = unified_propsys_outputs[2]

    propsys.mdot_cruise = vehicle.mdottot_cruise
   
    mdotm_tot = (1 - fL) * propsys.mdot_cruise
    mdote_tot = fL * propsys.mdot_cruise

    # These have to be separate since they are not mutually dependent
    try:
        mdotm = mdotm_tot / nr_fans_mech
    except ZeroDivisionError:
        mdotm = 0

    try:
        mdote = mdote_tot / nr_fans_elec
    except ZeroDivisionError:
        mdote = 0

    try:
        Pturb = Pturb / nr_fans_mech
    except ZeroDivisionError:
        Pturb = 0

    Acapm = 0.00515 * mdotm  # Raymer Chapter 10.3.4 for M <= 0.8
    Acape = 0.00515 * mdote

    if vehicle.cruise_mach <= 0.4:
        Afanm = Acapm
        Afane = Acape
    else:
        mach_inlet = 0.4 + (vehicle.cruise_mach - 0.4) / 2

        A_Astar_inlet = 1 / mach_inlet * ((1 + 0.2 * mach_inlet**2) / 1.2)**3
        A_Astar_face = 1 / 0.4 * ((1 + 0.2 * 0.4**2) / 1.2)**3

        Afanm = Acapm / (A_Astar_inlet / A_Astar_face)
        Afane = Acape / (A_Astar_inlet / A_Astar_face)

    # Fan diameters
    propsys.mech_fan_dia = Dfanm = np.sqrt(4 * Afanm / np.pi)
    propsys.elec_fan_dia = Dfane = np.sqrt(4 * Afane / np.pi)

    #########################
    # Mechanical propulsors #
    #########################
    # Nacelle diameters and lengths
    if propsys.is_turboprop:
        engine_dia = 0.25 * (Pturb / 1000)**0.120  # Raymer - p.323 (5th Ed.)
        engine_len = 0.12 * (Pturb / 1000)**0.373  # Raymer - p.323 (5th Ed.)
        propsys.mech_nac_dia = 1.2 * engine_dia
        propsys.nacelle_length_mech = 1.2 * engine_len
    else:
        propsys.mech_nac_dia = Dfanm / 0.8  # Raymer Chapter 10.3.4 for M <= 0.8
        propsys.nacelle_length_mech = 1.5 * propsys.mech_nac_dia

    #########################
    # Electrical propulsors #
    #########################
    propsys.elec_nac_dia = Dfane / 0.8
    propsys.nacelle_length_elec = 1.5 * propsys.elec_nac_dia

    # Wetted areas
    propsys.areas_wetted_mech = 1.1 * propsys.nacelle_length_mech * np.pi * propsys.mech_nac_dia
    propsys.areas_wetted_elec = 1.1 * 0.5 * propsys.nacelle_length_elec * np.pi * propsys.elec_nac_dia

    # Update BLI amounts
    wingspan_projected = vehicle.wings.main_wing.spans.projected
    fuselage_effective_diameter = vehicle.fuselages.fuselage.effective_diameter
    propsys.fBLIe = f_KED_wing * (nr_fans_elec * propsys.elec_nac_dia) / (wingspan_projected - fuselage_effective_diameter)

    # Set summary information
    propsys.info.nr_fans_mech = nr_fans_mech
    propsys.info.nr_fans_elec = nr_fans_elec
    propsys.info.mech_fan_dia = Dfanm
    propsys.info.elec_fan_dia = Dfane
    propsys.info.areas_wetted_mech_tot = vehicle.propulsors.unified_propsys.areas_wetted_mech * nr_fans_mech
    propsys.info.areas_wetted_elec_tot = vehicle.propulsors.unified_propsys.areas_wetted_elec * nr_fans_elec
    propsys.info.areas_wetted_mech_pylons = propsys.info.areas_wetted_mech_tot * 0.2
    propsys.info.areas_wetted_elec_pylons = 0
    propsys.info.fBLIe = propsys.fBLIe
    propsys.info.fBLIm = propsys.fBLIm

    return propsys
