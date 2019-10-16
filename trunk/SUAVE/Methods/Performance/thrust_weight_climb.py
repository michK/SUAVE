## @ingroup Methods-Performance
# thrust_weight_climb.py
#
# Created: Oct 2019, M. Kruger
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
#  Compute thrust to weight ratio required for a given rate of climb
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def thrust_weight_climb(vehicle, results, analyses, rate_of_climb):
    """ Estimate thrust to weight ratio required for a given rate of climb

    Source:
    Raymer

    Inputs:
    # vehicle
      fuel_fraction - Fraction of total wing volume usable for fuel storage

    Outputs:
    wing_fuel_vol                   [m^3]

    Properties Used:
    N/A
    """        

    # ==============================================
    # Unpack
    # ==============================================
    atmo = analyses.base.atmosphere
    mass_to = vehicle.mass_properties.takeoff
    reference_area  = vehicle.reference_area
    CL = results.mission.segments['climb'].conditions.aerodynamics.lift_coefficient[0]
    CD = results.mission.segments['climb'].conditions.aerodynamics.drag_breakdown.total[0]
    velocity = results.mission.segments['climb'].conditions.freestream.velocity[0]

    # ==============================================
    # Computing atmospheric conditions
    # ==============================================
    atmo_values       = atmo.compute_values(0, 0)
    conditions        = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    
    rho = atmo_values.density

    # ==============================================
    # Computations
    # ==============================================
    CL_CD = CL / CD

    thrust_weight_climb = (1 / CL_CD) + (rate_of_climb / velocity)

    power_climb = thrust_weight_climb * (mass_to * 9.81) * velocity

    return thrust_weight_climb, power_climb
