## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
# parasite_drag_pylon.py
#
# Created:  Jan 2014, T. Orra
# Modified: Jan 2016, E. Botero

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np

# Suave imports
from SUAVE.Core import Data

# ----------------------------------------------------------------------
#  Computes the pyloan parasite drag
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
def parasite_drag_pylon_unified(state, settings, vehicle):
    """
    Adapted from parasite_drag_pylon. Computes the parasite drag due to pylons as a proportion of the propulsor drag
    for unified propulsion system

    Assumptions:
    Basic fit

    Source:
    adg.stanford.edu (Stanford AA241 A/B Course Notes)

    Inputs:
    conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].
      form_factor                                                   [Unitless]
      compressibility_factor                                        [Unitless]
      skin_friction_coefficient                                     [Unitless]
      wetted_area                                                   [m^2]
      parasite_drag_coefficient                                     [Unitless]
      reynolds_number                                               [Unitless]
    vehicle.reference_area                                         [m^2]
    vehicle.propulsors.
      nacelle_diameter                                              [m]
      number_of_engines_mech                                               [Unitless]
      number_of_engines_elec                                               [Unitless]

    Outputs:
    propulsor_parasite_drag                                         [Unitless]

    Properties Used:
    N/A
    """

    # unpack
    conditions = state.conditions
    propulsor = vehicle.propulsors.unified_propsys

    pylon_factor_mech        =  0.2 # 20% of propulsor drag/wetted area
    pylon_factor_elec        =  0.1 # No pylons, but still installation effects
    n_propulsors             =  len(vehicle.propulsors)  # number of propulsive system in vehicle (NOT # of ENGINES)

    # Estimating pylon drag
    ref_area_mech            = np.pi / 4.0 * propulsor.mech_nac_dia**2.0
    ref_area_elec            = np.pi / 4.0 * propulsor.elec_nac_dia**2.0
    if vehicle.has_mech_pylons:
        pylon_parasite_drag_mech = pylon_factor_mech * conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient_mech * ref_area_mech / vehicle.reference_area * propulsor.nr_engines_mech
        pylon_wetted_area_mech   = pylon_factor_mech * conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].wetted_area_mech * propulsor.nr_engines_mech
    else:
        pylon_parasite_drag_mech = np.zeros_like(conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient_mech)
        pylon_wetted_area_mech = np.zeros_like(conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient_mech)
    pylon_parasite_drag_elec = pylon_factor_elec * conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient_elec * ref_area_elec / vehicle.reference_area * propulsor.nr_engines_elec
    pylon_wetted_area_elec   = pylon_factor_elec * conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].wetted_area_elec * propulsor.nr_engines_elec
    pylon_cf                 = conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].skin_friction_coefficient
    pylon_compr_fact         = conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].compressibility_factor
    pylon_rey_fact           = conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].reynolds_factor
    pylon_FF                 = conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].form_factor

    pylon_cf            /= n_propulsors
    pylon_compr_fact    /= n_propulsors
    pylon_rey_fact      /= n_propulsors
    pylon_FF            /= n_propulsors

    # consolidate mech and elec propulsors
    pylon_parasite_drag_coefficient = pylon_parasite_drag_mech + pylon_parasite_drag_elec
    pylon_wetted_area = pylon_wetted_area_mech + pylon_wetted_area_elec

    # dump data to conditions
    pylon_result = Data(
        wetted_area                    = pylon_wetted_area,
        reference_area                 = vehicle.reference_area,
        parasite_drag_coefficient      = pylon_parasite_drag_coefficient,
        skin_friction_coefficient      = pylon_cf,
        compressibility_factor         = pylon_compr_fact,
        reynolds_factor                = pylon_rey_fact,
        form_factor                    = pylon_FF,
    )

    conditions.aerodynamics.drag_breakdown.parasite['pylon'] = pylon_result

    return pylon_parasite_drag_coefficient
