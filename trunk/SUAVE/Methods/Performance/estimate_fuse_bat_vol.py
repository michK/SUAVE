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
def estimate_fuse_bat_vol(vehicle, vol_frac=0.05):
    """ Estimate wing volume that can be used to store fuel

    Source:
    Torenbeek, E.,
    2013.
    Advanced aircraft design: conceptual design,
    analysis and optimization of subsonic civil airplanes.
    John Wiley & Sons.

    Inputs:
    vehicle
    vol_frac - Fraction of total fuselage volume usable for battery storage

    Outputs:
    fuse_bat_vol                   [m^3]

    Properties Used:
    N/A
    """

    # ==============================================
    # Unpack
    # ==============================================
    fuse = vehicle.fuselages.fuselage
    area_front = fuse.areas.front_projected
    length = fuse.lengths.total
    width = fuse.width
    vol_fuse = area_front * (length - 2 * width)

    fuse_bat_vol = vol_frac * vol_fuse

    return fuse_bat_vol
