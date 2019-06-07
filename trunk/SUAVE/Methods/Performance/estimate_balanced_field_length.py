## @ingroup Methods-Performance
# estimate_balanced_field_length.py
#
# Created: June 2019, M. Kruger
# Modified: 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports
import SUAVE
from SUAVE.Core import Data, Units

# Package imports
import numpy as np

# ----------------------------------------------------------------------
#  Compute field length required for takeoff
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def estimate_balanced_field_length(vehicle, analyses, airport):
    """ Estimate balanced takeoff field length for unified propsys energy network
        Some parts based on estimate_take_off_field_length

    Source:
    Torenbeek, E.,
    2013.
    Advanced aircraft design: conceptual design,
    analysis and optimization of subsonic civil airplanes.
    John Wiley & Sons.

    Inputs:
    # TODO Populate

    Outputs:
    takeoff_field_length                   [m]

    Properties Used:
    N/A
    """        

    # ==============================================
    # Unpack
    # ==============================================
    atmo            = analyses.base.atmosphere
    altitude        = airport.altitude * Units.ft
    delta_isa       = airport.delta_isa
    mass_to         = vehicle.mass_properties.takeoff
    reference_area  = vehicle.reference_area
    PKtot           = vehicle.PKtot
    AR              = vehicle.wings.main_wing.aspect_ratio

    try:
        V2_VS_ratio = vehicle.V2_VS_ratio
    except:
        V2_VS_ratio = 1.2

    # ==============================================
    # Computing atmospheric conditions
    # ==============================================
    atmo_values       = atmo.compute_values(altitude,delta_isa)
    conditions        = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    
    p   = atmo_values.pressure
    T   = atmo_values.temperature
    rho = atmo_values.density
    a   = atmo_values.speed_of_sound
    mu  = atmo_values.dynamic_viscosity
    sea_level_gravity = atmo.planet.sea_level_gravity
    
    # ==============================================
    # Determining vehicle maximum lift coefficient
    # ==============================================
    try:   # aircraft maximum lift informed by user
        maximum_lift_coefficient = vehicle.maximum_lift_coefficient
    except:
        # Using semi-empirical method for maximum lift coefficient calculation
        from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift import compute_max_lift_coeff

        # Condition to CLmax calculation: 90KTAS @ 10000ft, ISA
        conditions  = atmo.compute_values(10000. * Units.ft)
        conditions.freestream=Data()
        conditions.freestream.density   = conditions.density
        conditions.freestream.dynamic_viscosity = conditions.dynamic_viscosity
        conditions.freestream.velocity  = 90. * Units.knots
        try:
            maximum_lift_coefficient, induced_drag_high_lift = compute_max_lift_coeff(vehicle,conditions)
            vehicle.maximum_lift_coefficient = maximum_lift_coefficient
        except:
            raise ValueError("Maximum lift coefficient calculation error. Please, check inputs")

    # ==============================================
    # Computing speeds (Vs, V2, 0.7*V2)
    # ==============================================
    Vstall = (2.0 * mass_to * sea_level_gravity / (rho * reference_area * maximum_lift_coefficient)) ** 0.5
    V2 = V2_VS_ratio * Vstall
    # speed_for_thrust  = 0.70 * V2

    # ==============================================
    # Calculate takeoff distance
    # ==============================================
    TV2   = PKtot / V2  # NOTE Should account for BLI, conservative otherwise
    CL2   = mass_to * sea_level_gravity / (0.5 * rho * V2**2 * reference_area)
    kT    = 0.85
    hTO   = 21 * Units.m    
    C0    = 0.025  # See Torenbeek p. 106
    E     = 0.85  # See Torenbeek p. 106
    weight = mass_to * sea_level_gravity
    balanced_field_length = weight**2 / (rho * sea_level_gravity * reference_area * CL2 * kT * TV2) \
        + hTO * (TV2 / weight - (C0 + CL2 / (np.pi * AR * E)))**-1

    return balanced_field_length
