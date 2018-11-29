## @ingroup Components-Energy-Converters
# Battery_Simple.py
#
# Created:  Nov 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Components.Energy.Energy_Component import Energy_Component

# ----------------------------------------------------------------------
#  Motor Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Battery_Simple(Energy_Component):
    """This is a simple battery component to be used in the Unified energy network.
    
    Assumptions:
    None

    Source:
    None
    """      
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """           
        self.efficiency = 0.90

    def power(self):
        """Calculates the battery's output power for a given input power

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:        
        self.inputs.power   [W]

        Outputs:
        self.outputs.power  [W]

        Properties Used:
        self.efficiency     [-]
        """           

        # Unpack
        efficiency = self.efficiency
        power = self.inputs.power
    
        power_out = power / efficiency

        self.outputs.power = power_out

        return power_out
        