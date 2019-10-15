## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
# parasite_drag_unified_propulsors.py
#
# Created:  Nov 2018, M. Kruger
# Modified:

#Sources: Stanford AA241 Course Notes
#         Raymer: Aircraft Design: A Conceptual Approach

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
from SUAVE.Core import Data
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Helper_Functions import compressible_turbulent_flat_plate

# package imports
import numpy as np

# ----------------------------------------------------------------------
#   Parasite Drag Propulsor
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
def parasite_drag_propulsors_unified(state, settings, geometry):
    """Computes the parasite drag due to the unified model propulsion system
       Modified from parasite_drag_propulsor.py
       The main difference is that there are now two types of propulsors, one type
       for mechanical and one for electrical; there can however be 0 to many of each
       type

    Assumptions:
    Basic fit

    Source:
    adg.stanford.edu (Stanford AA241 A/B Course Notes)

    Inputs:
    state.conditions.freestream.
      mach_number                                [Unitless]
      temperature                                [K]
      reynolds_number                            [Unitless]
    geometry.
      nacelle_diameter_mech                      [m^2]
      nacelle_diameter_elec                      [m^2]
      areas.wetted_mech                          [m^2]
      areas.wetted_elec                          [m^2]
      engine_length_mech                         [m]
      engine_length_elec                         [m]

    Outputs:
    propulsor_parasite_drag                      [Unitless]

    Properties Used:
    N/A
    """
    # unpack inputs
    conditions    = state.conditions

    propsys = geometry

    # mechanical propulsors
    Sref_mech      = np.pi / 4.0 * propsys.mech_nac_dia**2.0
    Swet_mech      = propsys.areas_wetted_mech
    # print("Drag: {}".format(propsys.mech_fan_dia))
    l_nacelle_mech = propsys.nacelle_length_mech
    d_nacelle_mech = propsys.mech_nac_dia
    nr_fans_mech   = propsys.nr_engines_mech
    f_embed_mech   = 1.0

    # electrical propulsors
    Sref_elec      = np.pi / 4.0 * propsys.elec_nac_dia**2.0
    Swet_elec      = propsys.areas_wetted_elec
    l_nacelle_elec = propsys.nacelle_length_elec
    d_nacelle_elec = propsys.elec_nac_dia
    nr_fans_elec   = propsys.nr_engines_elec
    f_embed_elec   = 1.0

    # conditions
    freestream = conditions.freestream
    Mc         = freestream.mach_number
    Tc         = freestream.temperature
    re         = freestream.reynolds_number

    # mechanical propulsors
    try:
        # Reynolds number
        Re_nacelle_mech = re*l_nacelle_mech
        # skin friction coefficient
        cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_nacelle_mech, Mc, Tc)
        # form factor according to Raymer equation (pg 283 of Aircraft Design: A Conceptual Approach)
        k_prop = 1 + 0.35 / (float(l_nacelle_mech) / float(d_nacelle_mech))
        parasite_drag_coefficient_mech = f_embed_mech * k_prop * cf_prop * Swet_mech / Sref_mech
    except:
        # parasite_drag_coefficient_mech = [np.zeros(np.shape(Mc)[0])]
        parasite_drag_coefficient_mech = np.zeros_like(Mc)

    # electrical propulsors
    try:
        # Reynolds number
        Re_nacelle_elec = re*l_nacelle_elec
        # skin friction coefficient
        cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_nacelle_elec, Mc, Tc)
        ## form factor according to Raymer equation (pg 283 of Aircraft Design: A Conceptual Approach)
        k_prop = 1 + 0.35 / (float(l_nacelle_elec) / float(d_nacelle_elec))
        parasite_drag_coefficient_elec = f_embed_elec * k_prop * cf_prop * Swet_elec / Sref_elec
    except:
        # parasite_drag_coefficient_elec = [np.zeros(np.shape(Mc)[0])]
        parasite_drag_coefficient_elec = np.zeros_like(Mc)

    # consolidate mech and elec propulsors
    # parasite_drag_coefficient = parasite_drag_coefficient_mech + parasite_drag_coefficient_elec
    wetted_area =  (nr_fans_mech * Swet_mech) + (nr_fans_elec * Swet_elec)

    # dump data to conditions
    propulsor_result = Data(
        wetted_area                    = wetted_area,
        wetted_area_mech               = Swet_mech,
        wetted_area_elec               = Swet_elec,
        reference_area_mech            = Sref_mech,
        reference_area_elec            = Sref_elec,
        parasite_drag_coefficient_mech = parasite_drag_coefficient_mech,
        parasite_drag_coefficient_elec = parasite_drag_coefficient_elec,
        skin_friction_coefficient      = cf_prop,
        compressibility_factor         = k_comp,
        reynolds_factor                = k_reyn,
        form_factor                    = k_prop,
    )
    conditions.aerodynamics.drag_breakdown.parasite[propsys.tag] = propulsor_result

    return parasite_drag_coefficient_mech, parasite_drag_coefficient_elec
