## @ingroup Components-Energy-Networks
# Unified.py
#
# Created:  Nov 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
from SUAVE.Components.Propulsors.Propulsor import Propulsor
from SUAVE.Components.Energy.Networks import Turbofan

from SUAVE.Core import Data, Units

# package imports
import numpy as np

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Unified(Propulsor):
    """ A unified network that is capable of modeling any permutation of an electrified architecture,
        as well as a conventionally powered system

        Assumptions:
        None

        Source:
        Kruger, M., Byahut, S., Uranga, A., Gonzalez, J., Hall, D.K. and Dowdle, A.,
        2018,
        Electrified Aircraft Trade-Space Exploration.
        Aviation Technology, Integration, and Operations Conference (p. 4227).
    """
    def __defaults__(self):
        """ This sets the default values for the network to function.

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

        self.nr_fans           = None
        self.nacelle_diameter  = None
        self.propeller         = None
        self.motor             = None
        self.inverter          = None
        self.battery           = None
        self.link_converter    = None
        self.link_elec_machine = None
        self.nr_engines        = None
        self.gas_turbine       = None
        self.avionics          = None
        self.tag               = 'network'


    # manage process with a driver function
    def evaluate_power(self, state):
        """ Calculate power given the current state of the vehicle

            Assumptions:
            N/A

            Source:
            N/A

            Inputs:
            state [state()]

            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            conditions.propulsion:
                TODO - Populate

            Properties Used:
            Defaulted values
        """

        # unpack
        conditions = state.conditions
        propeller = self.propeller
        motor     = self.motor
        inverter  = self.inverter
        battery   = self.battery

        # Create network by linking together different components

        # # Set battery energy
        # battery.current_energy = conditions.propulsion.battery_energy

        # Step 1 - battery power
        battery.inputs.power = 300.0 * Units['kW']

        # Step 2 - run battery
        battery.power()

        # Step 3 - link battery and inverter
        inverter.inputs.power = battery.outputs.power

        # Step 4 - run inverter
        inverter.power()

        # Step 5 - link inverter and motor
        motor.inputs.power = inverter.outputs.power

        # Step 6 - run motor
        motor.power()

        # Step 7 - link propeller and motor
        propeller.inputs.power = motor.outputs.power

        # Step 8 - run propeller
        propeller.power()

        # Pack the conditions for outputs
        pk = propeller.outputs.power

        # Create the outputs
        F    = pk / conditions.freestream.velocity
        mdot = np.zeros_like(F)

        results = Data()
        results.power_required = pk * np.ones_like(F)
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot
        results.max_power   = results.power_required
        results.Pbat = battery.inputs.power * np.ones_like(F)

        self.max_power = results.max_power
        self.max_bat_power = results.Pbat

        # store data
        results_conditions = Data
        conditions.propulsion = results_conditions(
                                                   Pbat = results.Pbat
                                                   )

        return results
