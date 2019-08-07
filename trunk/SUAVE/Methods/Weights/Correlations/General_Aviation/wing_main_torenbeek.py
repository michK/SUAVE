## @ingroup Methods-Weights-Correlations-General_Aviation
# wing_main.py
#
# Created:  Feb 2018, M. Vegh
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units
import numpy as np

# ----------------------------------------------------------------------
#   Wing Main
# ----------------------------------------------------------------------
## @ingroup Methods-Weights-Correlations-General_Aviation
def wing_main_torenbeek(b_wing, S_wing, m_to, m_zf):
    """
        Calculate the weight of the main wing of an aircraft

        Source:
            Torenbeek, E., 2013.
            Advanced aircraft design: conceptual design, analysis and optimization of subsonic civil airplanes.
            John Wiley & Sons.

        Inputs:
            b_wing- wing span            [m]
            S_wing - Wing planform area  [m**2]
            m_to - takeoff mass          [kg]
            m_zf - zero fuel mass        [radians]            

        Outputs:
            wt_main_wing  [kg]
    """
    
    W_to = 9.81 * m_to
    W_zf = 9.81 * m_zf
    weight = 0.86 *  b_wing * (S_wing * W_zf * W_to)**0.25
    return weight / 9.81
