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

    # Determine size of propulsors to cover 80% of upper wing surface
    wingspan_projected = vehicle.wings.main_wing.spans.projected
    fuselage_effective_diameter = vehicle.fuselages.fuselage.effective_diameter
    span_coverable = 0.8 * (wingspan_projected - fuselage_effective_diameter)

    # Catch zero division errors in case propulsors don't exist
    if nr_fans_mech >= 1:
        mdotm = mdotm_tot / nr_fans_mech
        PMfan = PMfan     / nr_fans_mech
    else:
        mdotm = 0
        PMfan = 0

    if nr_fans_elec >= 1:
        mdote = mdote_tot / nr_fans_elec
    else:
        mdote = 0

    if nr_turbines >= 1:
        Pturb = Pturb / nr_turbines
    else:
        Pturb = 0

    #########################
    # Mechanical propulsors #
    #########################
    # Mechanical fan
    vehicle.prop_diameter = vehicle.Dfanm
    propsys.prop_diameter = propsys.mech_fan_dia = Dfanm = vehicle.prop_diameter

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
            propsys.areas_wetted_mech = (1 - fL) * 1.1 * propsys.nacelle_length_mech * np.pi * propsys.mech_nac_dia
        else: # If the mech system is turbine only housed inside the aircraft
            propsys.areas_wetted_mech = 0

    #########################
    # Electrical propulsors #
    #########################
    # Electrical fan
    if vehicle.specify_Dfane is True:
        propsys.elec_fan_dia = Dfane = vehicle.Dfane
    else:
        propsys.elec_fan_dia = Dfane = 0.8 * span_coverable / nr_fans_elec  # 0.8 factor to account for nacelle
                                                                            # that is wider than fan
    # Electrical nacelle
    propsys.elec_nac_dia = Dfane / 0.8
    propsys.nacelle_length_elec = 1.5 * propsys.elec_nac_dia

    # Wetted areas
    propsys.areas_wetted_elec = fL * 0.5 * 1.1 * propsys.nacelle_length_elec * np.pi * propsys.elec_nac_dia

    # Divide between mechanical and electrical streams based on fL
    Acap = (nr_fans_mech * np.pi/4 * Dfanm**2) + (nr_fans_elec * np.pi/4 * Dfane**2)
    propsys.Acapm = (1 - fL) * Acap
    propsys.Acape = fL * Acap

    # Update BLI amounts
    propsys.fBLIe = fL * f_KED_wing * (nr_fans_elec * propsys.elec_nac_dia) / (wingspan_projected - fuselage_effective_diameter)
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
