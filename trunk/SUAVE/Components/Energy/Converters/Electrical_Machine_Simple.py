## @ingroup Components-Energy-Converters
# Electrical_Machine_Simple.py
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
class Electrical_Machine_Simple(Energy_Component):
    """This is a simple electrical machine component to be used in the Unified energy network.
       This component can function as a motor or generator
    
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
        self.efficiency = 0.95

    def power(self):
        """Calculates the machine's output power for a given input power

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
        