## @ingroup Methods-Performance
# estimate_wing_fuel_vol.py
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
def estimate_wing_fuel_vol(vehicle, fuel_fraction=0.5):
    """ Estimate wing volume that can be used to store fuel

    Source:
    Torenbeek, E.,
    2013.
    Advanced aircraft design: conceptual design,
    analysis and optimization of subsonic civil airplanes.
    John Wiley & Sons.

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
    wing = vehicle.wings.main_wing
    t_c = wing.thickness_to_chord
    lambda_w = wing.sweeps.quarter_chord
    Sw = vehicle.reference_area
    AR = wing.aspect_ratio
    kQ = 0.95

    wing_fuel_vol = fuel_fraction * kQ * t_c / np.sqrt(1 + lambda_w) * Sw * np.sqrt(Sw / AR)
    
    return wing_fuel_vol
