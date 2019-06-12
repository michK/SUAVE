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
            Kruger, M., Byahut, S., Uranga, A., Gonzalez, J., Hall, D.K. and Dowdle, A.,
            "Electrified Aircraft Trade-Space Exploration",
            2018, AIAA AVIATION, Atlanta, GA

            Inputs:
            state [state()]

            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            conditions.propulsion:
                PKm                  [watts]
                PKe                  [watts]
                battery_energy       [joules]
                battery_draw         [watts]

            Properties Used:
            N/A
        """

        # unpack
        conditions             = state.conditions
        fS                     = self.fS
        fL                     = self.fL
        battery                = self.battery
        inverter               = self.inverter
        motor                  = self.motor
        turbine                = self.turbine
        machine_link           = self.machine_link
        power_electronics_link = self.power_electronics_link
        fan_elec               = self.fan_elec
        fan_mech               = self.fan_mech

        # Create network by manually linking different components

        #############################################
        # if series (power in link flows downwards) #
        #############################################
        if ((1 - fS) * fL) > (power_electronics_link.efficiency * machine_link.eff * fS * (1 - fL)):

            # Set battery energy
            battery.current_energy = conditions.propulsion.battery_energy

            # step 1
            turbine.power()
            # link
            fan_mech.inputs.power = turbine.outputs.power - state.unknowns.machine_link_input_power
            # step 2
            fan_mech.power()
            # link
            machine_link.inputs.power = state.unknowns.machine_link_input_power
            # step 3
            machine_link.power()
            # link
            power_electronics_link.inputs.power = machine_link.outputs.power
            # step 4
            power_electronics_link.power()
            # link
            inverter.inputs.power = state.unknowns.battery_input_power + \
                power_electronics_link.outputs.power
            # step 5
            inverter.power()
            # link
            motor.inputs.power = inverter.outputs.power
            # step 6
            motor.power()
            # link
            fan_elec.inputs.power = motor.outputs.power
            # step 7
            fan_elec.power()
            

            # link
            battery.inputs.current  = esc.outputs.currentin*self.number_of_engines + avionics_payload_current
            battery.inputs.power_in = -(esc.outputs.voltageout*esc.outputs.currentin*self.number_of_engines + avionics_payload_power)
            battery.energy_calc(numerics)
            

        else:  # parallel (power in link flows downwards)
            pass  # TODO - Populate later


        # compute thrust
        thrust = (fan_elec.outputs.power + fan_mech.outputs.power) / conditions.freestream.velocity

        #Pack the conditions for outputs
        battery_energy = battery.current_energy
        battery_draw = battery.inputs.power_in
        PKm = fan_mech.outputs.power
        PKe = fan_elec.outputs.power

        conditions.propulsion.battery_energy = battery_energy
        conditions.propulsion.battery_draw = battery_draw
        conditions.propulsion.PKm = PKm
        conditions.propulsion.PKe = PKe

        #Create the outputs
        F = thrust * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        mdot = np.zeros_like(F) # TODO - Update

        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot

        return results


    def unpack_unknowns(self, segment):
        """ This is an extra set of unknowns which are unpacked from the mission solver and send to the network.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.unknowns.machine_link_input_power  [W]
    
            Outputs:
            state.conditions.propulsion.machine_link_input_power  [W]
    
            Properties Used:
            N/A
        """                  
        
        # Unpack unknowns provided for this network
        segment.state.conditions.propulsion.machine_link_input_power = segment.state.unknowns.machine_link_input_power
        segment.state.conditions.propulsion.battery.inputs.power_in = segment.state.unknowns.battery_input_power

        return


    def residuals(self,segment):
        """ This packs the residuals to be sent to the mission solver.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.conditions.propulsion:
                None
            state.unknowns.machine_link_input_power [W]
            
            Outputs:
            None
    
            Properties Used:
            N/A
        """        

        # Unpack residuals from network

        # Unpack
        machine_link_input_power_actual = segment.state.conditions.propulsion.machine_link_input_power
        machine_link_input_power_predict = segment.state.unknowns.machine_link_input_power

        battery_input_power_actual = segment.state.conditions.propulsion.battery.inputs.power_in
        battery_input_power_predict = segment.state.unknowns.battery_input_power
        
        # Return the residuals
        segment.state.residuals.network[:,0] = machine_link_input_power_predict[:,0] - \
            machine_link_input_power_actual[:,0]
        segment.state.residuals.network[:,1] = battery_input_power_predict[:,0] - \
            battery_input_power_actual[:,0]
        
        return

    __call__ = evaluate_thrust
