# unified_network_sizing.py
#
# Created:  Feb 2018, M. Kruger
# Modified: May 2019, M. Kruger

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np


def unified_network_sizing(vehicle):
    """
    This function takes the total mass flow through all propulsors
    and sizes the fans appropriately, based on the number of propulsors

    Method for fan and nacelle sizing from Raymer
    """

    propsys = vehicle.propulsors.unified_propsys

    nr_fans_mech = propsys.number_of_engines_mech
    nr_fans_elec = propsys.number_of_engines_elec

    fL = vehicle.fL_cruise  # Size propulsors for cruise

    propsys.mdot_cruise = vehicle.mdottot_cruise
   
    mdotm_tot = (1 - fL) * propsys.mdot_cruise
    mdote_tot = fL * propsys.mdot_cruise

    mdotm = mdotm_tot / nr_fans_mech
    mdote = mdote_tot / nr_fans_elec

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

    # Nacelle diameters and lengths
    propsys.mech_nac_dia = Dnacm = Dfanm / 0.8  # RaymerChapter 10.3.4 for M <= 0.8
    propsys.elec_nac_dia = Dnace = Dfane / 0.8
    propsys.nacelle_length_mech = 1.5 * Dnacm
    propsys.nacelle_length_elec = 1.5 * Dnace

    # Wetted areas
    propsys.areas_wetted_mech = 1.1 * propsys.nacelle_length_mech * np.pi * Dnacm
    # print("Sizing: {}".format(propsys.mech_fan_dia))
    propsys.areas_wetted_elec = 1.1 * 0.5 * propsys.nacelle_length_elec * np.pi * Dnace

    # Set summary information
    propsys.info.mech_fan_dia = Dfanm
    propsys.info.elec_fan_dia = Dfane
    propsys.info.areas_wetted_mech_tot = vehicle.propulsors.unified_propsys.areas_wetted_mech * nr_fans_mech
    propsys.info.areas_wetted_elec_tot = vehicle.propulsors.unified_propsys.areas_wetted_elec * nr_fans_elec
    propsys.info.areas_wetted_mech_pylons = propsys.info.areas_wetted_mech_tot * 0.2
    propsys.info.areas_wetted_elec_pylons = 0

    return propsys
