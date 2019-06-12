## @ingroup Components-Energy-Processes
# Unified_Thrust.py
#
# Created:  Feb 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports

from SUAVE.Core import Units
from SUAVE.Components.Energy.Energy_Component import Energy_Component


# ----------------------------------------------------------------------
#  Thrust Process
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Processes
class Unified_Thrust(Energy_Component):
    """A class that handles computation of thrust and other outputs for a gas turbine engine.

    Assumptions:
    Perfect gas

    Source:
    https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    """

    def __defaults__(self):
        """This sets the default value.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """
        self.tag = 'Thrust'
        self.total_design = 0.0
        self.outputs.thrust = 0.0
        self.outputs.non_dimensional_thrust = 0.0
        self.outputs.power = 0.0

    def compute(self, conditions):
        """Computes thrust and other properties as below.

        Assumptions:
        Perfect gas

        Source:
        None

        Inputs:
        None

        Outputs:
        self.outputs.
          thrust                             [N]
          non_dimensional_thrust             [-]
          power                              [W]

        Properties Used:
        self.
          reference_temperature              [K]
          reference_pressure                 [Pa]
          compressor_nondimensional_massflow [-]
          SFC_adjustment                     [-]
        """

        # Unpack the values
        # Unpack from inputs
        total_design = self.total_design

        # Unpack from conditions        
        u_inf = conditions.freestream.velocity
        throttle = conditions.propulsion.throttle

        # calculate thrust
        thrust = throttle * total_design

        #computing the power
        power = thrust * u_inf

        # pack outputs

        self.outputs.thrust = thrust
        self.outputs.power = power
        self.outputs.fuel_flow_rate = 0 * Units['kg/s']
        self.outputs.specific_impulse = 0

    __call__ = compute