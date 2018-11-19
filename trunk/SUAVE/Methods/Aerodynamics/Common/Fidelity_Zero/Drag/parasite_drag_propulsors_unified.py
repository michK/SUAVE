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
def parasite_drag_propulsors_unified(state,settings,geometry):
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

    # mechanical propulsors;
    Sref_mech      = propsys.nacelle_diameter_mech**2. / 4. * np.pi
    Swet_mech      = propsys.areas_wetted_mech
    l_prop_mech = propsys.engine_length_mech
    d_prop_mech = propsys.nacelle_diameter_mech

    # electrical propulsors
    Sref_elec      = propsys.nacelle_diameter_elec**2. / 4. * np.pi
    Swet_elec      = propsys.areas_wetted_elec
    l_prop_elec = propsys.engine_length_elec
    d_prop_elec = propsys.nacelle_diameter_elec

    # conditions
    freestream = conditions.freestream
    Mc  = freestream.mach_number
    Tc  = freestream.temperature
    re  = freestream.reynolds_number

    # mechanical propulsors
    # reynolds number
    Re_prop_mech = re*l_prop_mech

    # skin friction coefficient
    cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_prop_mech,Mc,Tc)

    ## form factor according to Raymer equation (pg 283 of Aircraft Design: A Conceptual Approach)
    k_prop = 1 + 0.35 / (float(l_prop_mech)/float(d_prop_mech))

    # find the final result
    propulsor_parasite_drag_mech = k_prop * cf_prop * Swet_mech / Sref_mech

    # electrical propulsors
    # reynolds number
    Re_prop_elec = re*l_prop_elec

    # skin friction coefficient
    cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_prop_elec,Mc,Tc)

    ## form factor according to Raymer equation (pg 283 of Aircraft Design: A Conceptual Approach)
    k_prop = 1 + 0.35 / (float(l_prop_elec)/float(d_prop_elec))

    # find the final result
    propulsor_parasite_drag_elec = k_prop * cf_prop * Swet_elec / Sref_elec

    # dump data to conditions
    propulsor_result = Data(
        wetted_area_mech          = Swet_mech    ,
        wetted_area_elec          = Swet_elec    ,
        reference_area_mech       = Sref_mech    ,
        reference_area_wlwx       = Sref_elec    ,
        parasite_drag_coefficient_mech = propulsor_parasite_drag_mech ,
        parasite_drag_coefficient_elec = propulsor_parasite_drag_elec ,
        skin_friction_coefficient = cf_prop ,
        compressibility_factor    = k_comp  ,
        reynolds_factor           = k_reyn  ,
        form_factor               = k_prop  ,
    )
    conditions.aerodynamics.drag_breakdown.parasite[propsys.tag] = propulsor_result

    return propulsor_parasite_drag_mech, propulsor_parasite_drag_elec
