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

from SUAVE.Core import Data

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
        nr_fans           = self.nr_fans
        nacelle_diameter  = self.nacelle_diameter
        propeller         = self.propeller
        motor             = self.motor
        inverter          = self.inverter
        battery           = self.battery
        link_converter    = self.link_converter
        link_elec_machine = self.link_elec_machine
        nr_engines        = self.nr_engines
        gas_turbine       = self.gas_turbine
        avionics          = self.avionics

        # Create network by linking together different components

        # Set battery energy
        battery.current_energy = conditions.propulsion.battery_energy


        










