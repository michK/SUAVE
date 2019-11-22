# unified_network_sizing.py
#
# Created:  Feb 2018, M. Kruger
# Modified: May 2019, M. Kruger

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np

from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers

def unified_network_sizing(propsys, vehicle, f_KED_wing=0.4):
    """
    This function takes the total mass flow through all propulsors
    and sizes the fans appropriately, based on the number of propulsors

    Method for fan and nacelle sizing from Raymer

    Inputs:
        propsys - Propulsion system model
        vehicle - Vehicle model
        f_KED_wing - Fraction of wing kinetic energy defect relative to full aircraft
                     For top surface only: 80% of total wing defect
                     For bottom surface only: 20% of total wing defect
                     Default value assumes 50% of airframe defect from
                     wing, and propulsors mounted on top surface only,
                     thus f_KED_wing = 0.8 * 0.5  = 0.4
    """

    nr_fans_mech = vehicle.nr_engines_mech
    nr_fans_elec = vehicle.nr_engines_elec
    nr_turbines = vehicle.nr_turbines

    # Size propulsors based on cruise
    fL = vehicle.fL_cruise  # Size propulsors for cruise
    fS = vehicle.fS_cruise  # Size propulsors for cruise

    # Run unified model
    eta_pe = propsys.eta_pe
    eta_mot = propsys.eta_mot
    eta_fan = propsys.eta_fan
    unified_propsys_outputs = calculate_powers(vehicle.PKtot, fS, fL, eta_pe, eta_mot, eta_fan)
    Pturb = unified_propsys_outputs[2]
    PMfan = unified_propsys_outputs[4]

    mdottot = vehicle.mdottot

    mdotm_tot = (1 - fL) * mdottot
    mdote_tot = fL * mdottot

    # Calculate total required capture area
    Acap = 1.66666667 * (0.00515 * mdottot)

    # Divide between mechanical and electrical streams based on fL
    propsys.Acapm = Acapm = (1 - fL) * Acap
    propsys.Acape = Acape = fL * Acap

    # Catch zero division errors in case propulsors don't exist
    try:
        mdotm = mdotm_tot / nr_fans_mech
    except ZeroDivisionError:
        mdotm = 0

    try:
        mdote = mdote_tot / nr_fans_elec
    except ZeroDivisionError:
        mdote = 0

    try:
        PMfan = PMfan / nr_fans_mech
    except ZeroDivisionError:
        PMfan = 0

    try:
        Pturb = Pturb / nr_turbines
    except ZeroDivisionError:
        Pturb = 0

    if vehicle.cruise_mach <= 0.4:
        # Divide between individual propulsors
        try:
            Afanm =  Acapm / nr_fans_mech
        except ZeroDivisionError:
            Afanm = 0
        try:
            Afane =  Acape / nr_fans_elec
        except ZeroDivisionError:
            Afane = 0
    else:
        mach_inlet = 0.4 + (vehicle.cruise_mach - 0.4) / 2

        A_Astar_inlet = 1 / mach_inlet * ((1 + 0.2 * mach_inlet**2) / 1.2)**3
        A_Astar_face = 1 / 0.4 * ((1 + 0.2 * 0.4**2) / 1.2)**3
        A_A = A_Astar_inlet / A_Astar_face

        try:
            Afanm = Acapm / A_A / nr_fans_mech
        except ZeroDivisionError:
            Afanm = 0
        try:
            Afane = Acape / A_A / nr_fans_elec
        except ZeroDivisionError:
            Afane = 0

    #########################
    # Mechanical propulsors #
    #########################
    # Mechanical fan
    if vehicle.is_turboprop:
        if vehicle.prop_nr_blades == 2:
            Kp_prop = 0.56
        elif vehicle.prop_nr_blades == 3:
            Kp_prop = 0.52
        elif vehicle.prop_nr_blades >= 4:
            Kp_prop = 0.49
        vehicle.prop_diameter = Kp_prop * (PMfan / 1000)**(1/4)  # Raymer - p.315 (5th Ed.)
        propsys.prop_diameter = propsys.mech_fan_dia = Dfanm = vehicle.prop_diameter
    else:
        propsys.mech_fan_dia = Dfanm = np.sqrt(4 * Afanm / np.pi)

    # Mechanical nacelle
    if vehicle.is_turboprop:
        engine_dia = 0.25 * (Pturb / 1000)**0.120  # Raymer - p.323 (5th Ed.)
        engine_len = 0.12 * (Pturb / 1000)**0.373  # Raymer - p.323 (5th Ed.)
        propsys.mech_nac_dia = 1.2 * engine_dia
        propsys.nacelle_length_mech = 1.2 * engine_len
        if vehicle.external_turb:
            propsys.areas_wetted_mech = 1.1 * propsys.nacelle_length_mech * np.pi * propsys.mech_nac_dia
        else:
            propsys.areas_wetted_mech = 0
    else:
        propsys.mech_nac_dia = Dfanm / 0.8  # Raymer Chapter 10.3.4 for M <= 0.8
        propsys.nacelle_length_mech = 1.5 * propsys.mech_nac_dia
        if vehicle.external_turb:
            propsys.areas_wetted_mech = 1.1 * propsys.nacelle_length_mech * np.pi * propsys.mech_nac_dia
        else: # If the mech system is turbine only housed inside the aircraft
            propsys.areas_wetted_mech = 0

    #########################
    # Electrical propulsors #
    #########################
    # Electrical fan
    propsys.elec_fan_dia = Dfane = np.sqrt(4 * Afane / np.pi)

    # Electrical nacelle
    propsys.elec_nac_dia = Dfane / 0.8
    propsys.nacelle_length_elec = 1.5 * propsys.elec_nac_dia

    # Wetted areas
    propsys.areas_wetted_elec = 0.5 * 1.1 * propsys.nacelle_length_elec * np.pi * propsys.elec_nac_dia

    # Update BLI amounts
    wingspan_projected = vehicle.wings.main_wing.spans.projected
    fuselage_effective_diameter = vehicle.fuselages.fuselage.effective_diameter
    propsys.fBLIe = f_KED_wing * (nr_fans_elec * propsys.elec_nac_dia) / (wingspan_projected - fuselage_effective_diameter)
    # Print warning if propulsors cannot fit on wings
    if (nr_fans_elec * propsys.elec_nac_dia) > (wingspan_projected - fuselage_effective_diameter):
        raise ValueError("Number of electric propulsors cannot fit on wing")

    # Set summary information
    propsys.info.nr_fans_mech = nr_fans_mech
    propsys.info.nr_fans_elec = nr_fans_elec
    propsys.info.mech_fan_dia = Dfanm
    propsys.info.elec_fan_dia = Dfane
    propsys.info.areas_wetted_mech_tot = vehicle.propulsors.unified_propsys.areas_wetted_mech * nr_fans_mech
    propsys.info.areas_wetted_elec_tot = vehicle.propulsors.unified_propsys.areas_wetted_elec * nr_fans_elec
    if vehicle.has_mech_pylons:
        propsys.info.areas_wetted_mech_pylons = propsys.info.areas_wetted_mech_tot * 0.2
    else:
        propsys.info.areas_wetted_mech_pylons = np.zeros_like(propsys.info.areas_wetted_mech_tot)
    propsys.info.areas_wetted_elec_pylons = 0
    propsys.info.fBLIe = propsys.fBLIe
    propsys.info.fBLIm = propsys.fBLIm

    return propsys
