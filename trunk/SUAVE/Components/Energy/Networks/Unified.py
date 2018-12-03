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

        self.nr_fans_elec      = None
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
        propeller  = self.propeller
        motor      = self.motor
        inverter   = self.inverter
        battery    = self.battery
        turbofan   = self.turbofan

        ram                       = self.turbofan.ram
        inlet_nozzle              = self.turbofan.inlet_nozzle
        low_pressure_compressor   = self.turbofan.low_pressure_compressor
        high_pressure_compressor  = self.turbofan.high_pressure_compressor
        fan                       = self.turbofan.fan
        combustor                 = self.turbofan.combustor
        high_pressure_turbine     = self.turbofan.high_pressure_turbine
        low_pressure_turbine      = self.turbofan.low_pressure_turbine
        core_nozzle               = self.turbofan.core_nozzle
        fan_nozzle                = self.turbofan.fan_nozzle
        thrust                    = self.turbofan.thrust
        bypass_ratio              = self.turbofan.bypass_ratio
        number_of_engines         = self.turbofan.number_of_engines

        # Create network by linking together different components

        # Electrical side

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
        pk_elec = propeller.outputs.power

        # Create the outputs
        F_elec    = pk_elec / conditions.freestream.velocity
        mdot = np.zeros_like(F_elec)
        P_elec = pk_elec

        # Conventional side

        #set the working fluid to determine the fluid properties
        ram.inputs.working_fluid                               = self.turbofan.working_fluid

        #Flow through the ram , this computes the necessary flow quantities and stores it into conditions
        ram(conditions)

        #link inlet nozzle to ram
        inlet_nozzle.inputs.stagnation_temperature             = ram.outputs.stagnation_temperature
        inlet_nozzle.inputs.stagnation_pressure                = ram.outputs.stagnation_pressure

        #Flow through the inlet nozzle
        inlet_nozzle(conditions)

        #--link low pressure compressor to the inlet nozzle
        low_pressure_compressor.inputs.stagnation_temperature  = inlet_nozzle.outputs.stagnation_temperature
        low_pressure_compressor.inputs.stagnation_pressure     = inlet_nozzle.outputs.stagnation_pressure

        #Flow through the low pressure compressor
        low_pressure_compressor(conditions)

        #link the high pressure compressor to the low pressure compressor
        high_pressure_compressor.inputs.stagnation_temperature = low_pressure_compressor.outputs.stagnation_temperature
        high_pressure_compressor.inputs.stagnation_pressure    = low_pressure_compressor.outputs.stagnation_pressure

        #Flow through the high pressure compressor
        high_pressure_compressor(conditions)

        #Link the fan to the inlet nozzle
        fan.inputs.stagnation_temperature                      = inlet_nozzle.outputs.stagnation_temperature
        fan.inputs.stagnation_pressure                         = inlet_nozzle.outputs.stagnation_pressure

        #flow through the fan
        fan(conditions)

        #link the combustor to the high pressure compressor
        combustor.inputs.stagnation_temperature                = high_pressure_compressor.outputs.stagnation_temperature
        combustor.inputs.stagnation_pressure                   = high_pressure_compressor.outputs.stagnation_pressure

        #flow through the high pressure compressor
        combustor(conditions)

        # link the shaft power output to the low pressure compressor
        try:
            shaft_power = self.Shaft_Power_Off_Take
            shaft_power.inputs.mdhc                            = thrust.compressor_nondimensional_massflow
            shaft_power.inputs.Tref                            = thrust.reference_temperature
            shaft_power.inputs.Pref                            = thrust.reference_pressure
            shaft_power.inputs.total_temperature_reference     = low_pressure_compressor.outputs.stagnation_temperature
            shaft_power.inputs.total_pressure_reference        = low_pressure_compressor.outputs.stagnation_pressure

            shaft_power(conditions)
        except:
            pass

        #link the high pressure turbine to the combustor
        high_pressure_turbine.inputs.stagnation_temperature    = combustor.outputs.stagnation_temperature
        high_pressure_turbine.inputs.stagnation_pressure       = combustor.outputs.stagnation_pressure
        high_pressure_turbine.inputs.fuel_to_air_ratio         = combustor.outputs.fuel_to_air_ratio

        #link the high pressure turbine to the high pressure compressor
        high_pressure_turbine.inputs.compressor                = high_pressure_compressor.outputs

        #link the high pressure turbine to the fan
        high_pressure_turbine.inputs.fan                       = fan.outputs
        high_pressure_turbine.inputs.bypass_ratio              = 0.0 #set to zero to ensure that fan not linked here

        #flow through the high pressure turbine
        high_pressure_turbine(conditions)

        #link the low pressure turbine to the high pressure turbine
        low_pressure_turbine.inputs.stagnation_temperature     = high_pressure_turbine.outputs.stagnation_temperature
        low_pressure_turbine.inputs.stagnation_pressure        = high_pressure_turbine.outputs.stagnation_pressure

        #link the low pressure turbine to the low_pressure_compresor
        low_pressure_turbine.inputs.compressor                 = low_pressure_compressor.outputs

        #link the low pressure turbine to the combustor
        low_pressure_turbine.inputs.fuel_to_air_ratio          = combustor.outputs.fuel_to_air_ratio

        #link the low pressure turbine to the fan
        low_pressure_turbine.inputs.fan                        = fan.outputs

        # link the low pressure turbine to the shaft power, if needed
        try:
            low_pressure_turbine.inputs.shaft_power_off_take   = shaft_power.outputs
        except:
            pass

        #get the bypass ratio from the thrust component
        low_pressure_turbine.inputs.bypass_ratio               = bypass_ratio

        #flow through the low pressure turbine
        low_pressure_turbine(conditions)

        #link the core nozzle to the low pressure turbine
        core_nozzle.inputs.stagnation_temperature              = low_pressure_turbine.outputs.stagnation_temperature
        core_nozzle.inputs.stagnation_pressure                 = low_pressure_turbine.outputs.stagnation_pressure

        #flow through the core nozzle
        core_nozzle(conditions)

        #link the fan nozzle to the fan
        fan_nozzle.inputs.stagnation_temperature               = fan.outputs.stagnation_temperature
        fan_nozzle.inputs.stagnation_pressure                  = fan.outputs.stagnation_pressure

        # flow through the fan nozzle
        fan_nozzle(conditions)

        # compute the thrust using the thrust component
        #link the thrust component to the fan nozzle
        thrust.inputs.fan_exit_velocity                        = fan_nozzle.outputs.velocity
        thrust.inputs.fan_area_ratio                           = fan_nozzle.outputs.area_ratio
        thrust.inputs.fan_nozzle                               = fan_nozzle.outputs

        #link the thrust component to the core nozzle
        thrust.inputs.core_exit_velocity                       = core_nozzle.outputs.velocity
        thrust.inputs.core_area_ratio                          = core_nozzle.outputs.area_ratio
        thrust.inputs.core_nozzle                              = core_nozzle.outputs

        #link the thrust component to the combustor
        thrust.inputs.fuel_to_air_ratio                        = combustor.outputs.fuel_to_air_ratio

        #link the thrust component to the low pressure compressor
        thrust.inputs.total_temperature_reference              = low_pressure_compressor.outputs.stagnation_temperature
        thrust.inputs.total_pressure_reference                 = low_pressure_compressor.outputs.stagnation_pressure
        thrust.inputs.number_of_engines                        = number_of_engines
        thrust.inputs.bypass_ratio                             = bypass_ratio
        thrust.inputs.flow_through_core                        = 1./(1.+bypass_ratio) #scaled constant to turn on core thrust computation
        thrust.inputs.flow_through_fan                         = bypass_ratio/(1.+bypass_ratio) #scaled constant to turn on fan thrust computation

        #compute the thrust
        thrust(conditions)

        # Get network outputs from the thrust outputs
        F_mech            = thrust.outputs.thrust*[1,0,0]
        mdot         = thrust.outputs.fuel_flow_rate
        output_power = thrust.outputs.power
        F_vec        = conditions.ones_row(3) * 0.0
        F_vec[:,0]   = F_mech[:,0]
        F_mech       = F_vec
        P_mech = F_vec * conditions.freestream.velocity

        # Combined results

        F_total = F_mech + F_elec
        P_total = P_mech + P_elec

        results = Data()
        results.power_required_elec = pk_elec * np.ones_like(F_elec)
        results.thrust_force_vector = F_total
        results.vehicle_mass_rate   = mdot
        results.max_power   = P_total
        results.Pbat = battery.inputs.power * np.ones_like(F_elec)
        results.throttle = 1.0  # FIXME - Placeholder

        self.max_power = results.max_power
        self.max_bat_power = results.Pbat

        # store data
        results_conditions = Data
        conditions.propulsion = results_conditions(
                                                   Pbat = results.Pbat,
                                                   throttle = results.throttle
                                                   )

        return results
