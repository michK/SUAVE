## @ingroup Components-Energy-Converters
# Gas_Turbine_Simple.py
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
#  Simple Turbine Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Gas_Turbine_Simple(Energy_Component):
    """ This is a simple gas turbine component.
        Converts energy stored in fuel into shaft power

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

        self.tag = 'Gas_Turbine_Simple'
        self.efficiency_thermal = 0.5
        self.h_fuel = 43 * Units['MJ/kg']

    def power(self):
        """Calculates the turbines fuel consumption base on a given power requirement

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:        
        self.inputs.power   [W]

        Outputs:
        self.outputs.power      [W]
        self.outputs.fuel_burn  [kg/s]

        Properties Used:
        self.efficiency     [-]
        """

        # Unpack
        efficiency_thermal = self.efficiency_thermal
        h_fuel = self.h_fuel
        power = self.inputs.power

        fuel_burn = power / efficiency_thermal / h_fuel

        self.outputs.fuel_burn = fuel_burn

        return fuel_burn
