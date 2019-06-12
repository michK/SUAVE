## @ingroup Components-Energy-Converters
# Power_Electronics_Simple.py
#
# Created:  Feb 2019, M. Kruger
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
#  Power Electronics Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Power_Electronics_Simple(Energy_Component):
    """ This is a simple power electronics component.
        Can function as AC->DC or DC->AC converter.

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

        self.tag = 'Power_Electronics_Simple'
        self.efficiency = 0.98

    def power(self):
        """Calculates the component's output power for a given input power

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

        power_out = efficiency * power

        self.outputs.power = power_out

        return power_out
