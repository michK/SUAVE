## @ingroup Components-Energy-Converters
# Fan_Simple.py
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
#  Simple Fan Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Fan_Simple(Energy_Component):
    """ This is a simple fan component.
        Converts shaft power to flow power

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

        self.tag = 'Fan_Simple'
        self.efficiency = 0.9

    def spin(self, conditions):
        """Calculates the flow power and thrust

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        self.inputs.power       [W]
        conditions.freestream.
          velocity              [m/s]

        Outputs:
        self.outputs.power      [W]
        self.outputs.thrust     [W]

        Properties Used:
        self.efficiency         [-]
        """

        # Unpack
        efficiency = self.efficiency
        power = self.inputs.power
        V = conditions.freestream.velocity[:, 0, None]

        power_out = efficiency * power
        thrust = power_out / V

        self.outputs.power = power_out
        self.outputs.thrust = thrust

        return power_out
