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
from SUAVE.Core import Data, Units
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

        self.tag = 'Network'
        self.info = Data()

    # manage process with a driver function
    def evaluate_thrust(self, state):
        """ Calculate power given the current state of the vehicle

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            state [state()]

            Outputs:
            results.vehicle_mass_rate   [kg/s]

            Properties Used:
            Defaulted values
        """

        #Unpack
        conditions = state.conditions
        numerics   = state.numerics
        thrust = self.thrust

        # Set battery energy
        battery = self.battery
        battery.current_energy = conditions.propulsion.battery_energy

        # Constants
        hfuel = 43.0 * Units['MJ/kg']
        eta_th = 0.5

        # Unpack inputs
        nr_fans_mech  = self.number_of_engines_mech
        # mech_nac_dia  = self.mech_nac_dia
        nr_fans_elec  = self.number_of_engines_elec
        # elec_nac_dia  = self.elec_nac_dia
        fL            = self.fL
        fS            = self.fS
        fBLIe         = self.fBLIe
        fBLIm         = self.fBLIm
        max_bat_power = self.battery.max_power
        Cp            = self.Cp

        # Efficiencies
        # Propulsive efficiencies
        eta_propm = 0.9
        eta_prope = 0.9
        # Component efficiencies
        eta_pe  = 0.98
        eta_mot = 0.95
        eta_fan = 0.9


        CD_par = conditions.aerodynamics.drag_breakdown.parasite.total
        CD_tot = conditions.aerodynamics.drag_breakdown.total

        Vinf    = conditions.freestream.velocity
        rho_inf = conditions.freestream.density

        fsurf = 0.9

        # Calculate total drag
        qinf = 0.5 * rho_inf * Vinf**2.0
        Dp = CD_tot * qinf * self.reference_area
        Dpar = CD_par * qinf * self.reference_area
        Dpp_DP = CD_par / CD_tot

        thrust.inputs.nr_elements       = np.shape(CD_tot)[0]
        thrust.inputs.vertical_velocity = - conditions.frames.inertial.velocity_vector[:,2]
        thrust.inputs.fS                = fS
        thrust.inputs.fL                = fL
        thrust.inputs.eta_propm         = eta_propm
        thrust.inputs.eta_prope         = eta_prope
        thrust.inputs.eta_th            = eta_th
        thrust.inputs.eta_pe            = eta_pe
        thrust.inputs.eta_mot           = eta_mot
        thrust.inputs.eta_fan           = eta_fan
        thrust.inputs.Vinf              = Vinf
        thrust.inputs.rho_inf           = rho_inf
        thrust.inputs.Dp                = Dp
        thrust.inputs.Dpar              = Dpar
        thrust.inputs.Dpp_DP            = Dpp_DP
        thrust.inputs.fBLIe             = fBLIe
        thrust.inputs.fBLIm             = fBLIm
        thrust.inputs.fsurf             = fsurf
        thrust.inputs.nr_fans_elec      = nr_fans_elec
        thrust.inputs.nr_fans_mech      = nr_fans_mech
        thrust.inputs.hfuel             = hfuel
        thrust.inputs.max_bat_power     = max_bat_power
        thrust.inputs.Cp                = Cp

        #compute the thrust
        thrust(conditions, numerics)

        # Set battery power and current
        battery.inputs.current  = thrust.outputs.Pbat / battery.max_voltage
        battery.inputs.power_in = -thrust.outputs.Pbat
        battery.energy_calc(numerics)

        #getting the network outputs from the thrust outputs
        F            = thrust.outputs.thrust*[1,0,0]
        mdot         = thrust.outputs.mdot
        output_power = thrust.outputs.power
        F_vec        = conditions.ones_row(3) * 0.0
        F_vec[:,0]   = F[:,0]
        F            = F_vec

        #Pack the conditions for outputs
        conditions.propulsion.PKm = thrust.outputs.PKm_tot
        conditions.propulsion.PKe = thrust.outputs.PKe_tot
        conditions.propulsion.mdotm = thrust.outputs.mdotm_tot
        conditions.propulsion.mdote = thrust.outputs.mdote_tot
        conditions.propulsion.Vjetm = thrust.outputs.Vjetm_tot
        conditions.propulsion.Vjete = thrust.outputs.Vjete_tot
        conditions.propulsion.battery_energy = battery.current_energy
        conditions.propulsion.battery_draw = thrust.outputs.Pbat

        results = Data()
        results.vehicle_mass_rate = mdot
        results.thrust_force_vector = F

        return results

    __call__ = evaluate_thrust
