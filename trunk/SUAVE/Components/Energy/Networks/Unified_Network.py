## @ingroup Components-Energy-Networks
# Battery_Ducted_Fan.py
#
# Created:  Feb 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np
from SUAVE.Core import Data
from SUAVE.Methods.Power.Battery.Variable_Mass import find_mass_gain_rate
from SUAVE.Components.Propulsors.Propulsor import Propulsor

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------


## @ingroup Components-Energy-Networks
class Unified_Network(Propulsor):
    """ Network that can model various combinations of components
        to model conventional, all-electric, hybrid- or turbo-
        electric aircraft

        Assumptions:
        None

        Source:
        None
    """

    def __defaults__(self):
        """ This sets the default values for the network to function.
            This network operates slightly different than most as it attaches a propulsor to the net.

            Assumptions:
            None

            Source:
            Kruger, M., Byahut, S., Uranga, A., Gonzalez, J., Hall, D.K. and Dowdle, A.,
            "Electrified Aircraft Trade-Space Exploration",
            June 2018, AIAA Aviation Technology, Integration, and Operations Conference (AIAA AVIATION).
            Atlanta, GA, USA

            Inputs:
            None

            Outputs:
            None

            Properties Used:
            N/A
        """

        # Components
        self.propulsor = None
        self.battery = None
        self.inverter = None
        self.motor = None
        self.turbine = None
        self.machine_link = None
        self.power_electronics_link = None
        self.fan_elec = None
        self.fan_mech = None
        self.thrust = None
        # Other
        self.fS = 0
        self.fL = 0
        self.max_thrust = 0
        self.tag = 'Network'

    # manage process with a driver function
    def evaluate_thrust(self, state):
        """ Calculate thrust given the current state of the vehicle

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            state [state()]

            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]

            Properties Used:
            N/A
        """

        # if ((1 - fS) * fL) > (eta_pe * eta_mot * fS * (1 - fL)):  # Series - Link is generator
        # else: # Parallel - Link is motor
        # unpack

        propulsor = self.propulsor
        battery = self.battery
        inverter = self.inverter
        motor = self.motor
        turbine = self.turbine
        machine_link = self.machine_link
        power_electronics_link = self.power_electronics_link
        fan_elec = self.fan_elec
        fan_mech = self.fan_mech

        conditions = state.conditions
        numerics = state.numerics

        results = propulsor.evaluate_thrust(state)
        Pe = np.multiply(results.thrust_force_vector[:, 0],
                         conditions.freestream.velocity[0])

        # Set battery energy
        battery.current_energy = conditions.propulsion.battery_energy

        pbat = -Pe / self.motor_efficiency
        battery_logic = Data()
        battery_logic.power_in = pbat
        battery_logic.current = 90.  #use 90 amps as a default for now; will change this for higher fidelity methods

        battery.inputs = battery_logic

        battery.energy_calc(numerics)

        #allow for mass gaining batteries
        try:
            mdot = find_mass_gain_rate(battery, -(
                pbat - battery.resistive_losses))  #put in transpose for solver
        except AttributeError:
            mdot = np.zeros_like(results.thrust_force_vector[:, 0])
        mdot = np.reshape(mdot, np.shape(conditions.freestream.velocity))
        #Pack the conditions for outputs
        battery_draw = battery.inputs.power_in
        battery_energy = battery.current_energy

        conditions.propulsion.battery_draw = battery_draw
        conditions.propulsion.battery_energy = battery_energy

        results.vehicle_mass_rate = mdot
        return results

    __call__ = evaluate_thrust
