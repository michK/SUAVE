## @ingroup Components-Energy-Networks
# Unified_Propsys.py
#
# Created:  Oct 2018, Michael Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
from SUAVE.Core import Units, Data
from SUAVE.Components.Propulsors.Propulsor import Propulsor
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers, remove_negatives

# package imports
import numpy as np
from scipy.optimize import fsolve

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Unified_Propsys(Propulsor):
    """ Unified propulsion system model

        Assumptions:
        None

        Source:
        Hall, D. K., Huang, A. C., Uranga, A., Greitzer, E. M., Drela, M., and Sato, S.,
        "Boundary Layer Ingestion Propulsion Benefit for Transport Aircraft",
        Journal of Propulsion and Power, Vol. 33, No. 5, 2017, pp. 1118-1129,
        doi:10.2514/1.B36321.
    """


    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            N/A

            Source:
            N/A

            Inputs:
            None

            Outputs:
            None

            Properties Used:
            N/A
        """

        self.propulsor        = None
        self.battery          = None
        self.motor_efficiency = .95
        self.throttle         =  1.0
        self.tag              = 'Network'

        # Propulsor areas
        self.mech_fan_dia = 1.0 * Units.m
        self.elec_fan_dia = 1.0 * Units.m

        self.area_noz_fan = 0.6  # FIXME - Find reasonable values
        self.area_jet_noz = 0.95  # FIXME - Find reasonable values

        # areas needed for drag; not in there yet
        self.areas             = Data()
        self.areas.wetted      = 0.0

        # max power tracker
        self.max_power = 0.01
        self.max_bat_power = 0.01


    def evaluate_power(self, state):
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

        # Constants
        hfuel = 43.0 * Units['MJ/kg']
        eta_th = 0.5

        # Unpack inputs
        nr_fans_mech = self.number_of_engines_mech
        nr_fans_elec = self.number_of_engines_elec
        fL = self.fL
        fS = self.fS
        fBLIm = self.fBLIm
        dia_fan_mech = self.fan_diameter_mech
        dia_fan_elec = self.fan_diameter_elec

        # Calculate wing BLI from electrical propulsors
        fBLIe = (nr_fans_elec * dia_fan_elec) / (self.wingspan_projected -
            self.fuselage_effective_diameter)

        # Calculate fan areas
        area_fan_mech = np.pi / 4.0 * dia_fan_mech**2.0
        area_fan_elec = np.pi / 4.0 * dia_fan_elec**2.0

        # Calculate jet area
        area_jet_mech = area_fan_mech * self.area_noz_fan * self.area_jet_noz
        area_jet_elec = area_fan_elec * self.area_noz_fan * self.area_jet_noz

        # Efficiencies
        eta_pe  = 0.98
        eta_mot = 0.95
        eta_fan = 0.9

        CD_tot = conditions.aerodynamics.drag_breakdown.total

        CD_par = conditions.aerodynamics.drag_breakdown.parasite.total

        Vinf = conditions.freestream.velocity

        fsurf = 0.9

        # Calculate total drag
        qinf = 0.5 * conditions.freestream.density * Vinf**2.0
        Dp = CD_tot * qinf * self.reference_area
        Dpp_DP = CD_par / CD_tot

        # Set up system of equations to solve power balance
        nr_elements = np.shape(CD_tot)[0]

        # Initialize solution arrays
        PKm_tot = np.zeros(nr_elements)
        PKe_tot = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        PKe = np.zeros(nr_elements)
        PKm = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat = np.zeros(nr_elements)

        for i in range(nr_elements):

            def power_balance(params):
                """Function to calculate resisuals of power balance equations"""
                PKm, PKe, mdotm, mdote, Vjetm, Vjete = params

                res1 = fL - PKe / (PKe + PKm)

                res2 = mdotm * (Vjetm - Vinf[i]) + mdote * (Vjete - Vinf[i]) - \
                    Dp[i] * (1.0 - fBLIm * Dpp_DP[i] - fBLIe * Dpp_DP[i])

                res3 = PKm - 0.5 * mdotm * (Vjetm**2.0 - Vinf[i]**2.0) - fBLIm * fsurf * Dp[i] * Vinf[i]

                res4 = PKe - 0.5 * mdote * (Vjete**2.0 - Vinf[i]**2.0) - fBLIe * fsurf * Dp[i] * Vinf[i]

                res5 = mdotm - nr_fans_mech * conditions.freestream.density[i] * area_jet_mech * Vjetm

                res6 = mdote - nr_fans_elec * conditions.freestream.density[i] * area_jet_elec * Vjete

                residuals = [
                             abs(res1),
                             abs(res2),
                             abs(res3),
                             abs(res4),
                             abs(res5),
                             abs(res6),
                            ]

                return residuals

            args_init = [300000.0, 300000.0, 100.0, 100.0, 50.0, 50.0]  # FIXME - should be more clever guesses
            [PKm, PKe, mdotm, mdote, Vjetm, Vjete] = remove_negatives(fsolve(power_balance, args_init))

            PKm_tot[i] = PKm
            PKe_tot[i] = PKe
            mdotm_tot[i] = mdotm
            mdote_tot[i] = mdote

            PK_tot = PKm + PKe

            [PKe_i, PKm_i, PfanE_i, PfanM_i, Pmot_i, Pinv_i, Pbat_i, Pturb_i, Pmot_link_i, Pconv_i, Plink_i] = \
            calculate_powers(PK_tot, fS, fL, eta_pe, eta_mot, eta_fan)

            PKm = PKm_i
            PKe = PKe_i

            # Ragone relation for battery efficiency
            psi = Pbat_i / self.Pbat_max
            eta_bat = 0.5 + (1.0 - psi) / 2.0

            # Adjust battery power to account for efficiency drop
            Pbat[i] = Pbat_i / eta_bat

            # Calculate vehicle mass rate of change
            mdot_fuel[i] = Pturb_i / (hfuel * eta_th)

        results = Data()
        results.PK_tot = (PKm_tot + PKe_tot).reshape(nr_elements, 1)
        results.power_required = results.PK_tot
        results.Pbat_max = self.max_bat_power
        results.mdot_tot = mdot_fuel.reshape(nr_elements, 1)
        results.vehicle_mass_rate = results.mdot_tot
        results.Pbat = Pbat.reshape(nr_elements, 1)
        results.throttle = results.PK_tot / self.max_power

        # Keep track of max total and battery power

        if np.amax(Pbat) > self.max_bat_power:
            self.max_bat_power = np.amax(Pbat)

        if PK_tot > self.max_power:
            self.max_power = PK_tot

        # store data
        results_conditions = Data
        conditions.propulsion = results_conditions(
                                                   PK_tot = results.PK_tot,
                                                   Pbat = results.Pbat,
                                                   Pbat_max = results.Pbat_max,
                                                   throttle = results.throttle
                                                   )

        return results

    __call__ = evaluate_power
