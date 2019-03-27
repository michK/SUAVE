## @ingroup Components-Energy-Processes
# Unified_Thrust.py
#
# Created:  Feb 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np
from scipy.optimize import root
# SUAVE imports
from SUAVE.Core import Units
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers, remove_negatives

# ----------------------------------------------------------------------
#  Thrust Process
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Processes
class Unified_Thrust_tmp(Energy_Component):
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
          power                              [W]

        Properties Used:
        self.
          TODO
        """
        # Unpack the values

        # Unpack from inputs
        nr_elements   = self.inputs.nr_elements
        fS            = self.inputs.fS
        fL            = self.inputs.fL        
        eta_th        = self.inputs.eta_th
        eta_pe        = self.inputs.eta_pe
        eta_mot       = self.inputs.eta_mot
        eta_fan       = self.inputs.eta_fan
        Vinf          = self.inputs.Vinf
        Dp            = self.inputs.Dp
        Dpp_DP        = self.inputs.Dpp_DP
        fBLIe         = self.inputs.fBLIe
        fBLIm         = self.inputs.fBLIm
        fsurf         = self.inputs.fsurf
        nr_fans_mech  = self.inputs.nr_fans_mech
        nr_fans_elec  = self.inputs.nr_fans_elec
        area_jet_mech = self.inputs.area_jet_mech
        area_jet_elec = self.inputs.area_jet_elec
        hfuel         = self.inputs.hfuel

        # Unpack from conditions
        u_inf = conditions.freestream.velocity
        throttle = conditions.propulsion.throttle

        # Set up system of equations to solve power balance
        # Initialize solution arrays
        PKm_tot = np.zeros(nr_elements)
        PKe_tot = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        PKe = np.zeros(nr_elements)
        PKm = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat = np.zeros(nr_elements)
        Pturb = np.zeros(nr_elements)

        for i in range(nr_elements):

            def power_balance(params):
                """Function to calculate residuals of power balance equations"""
                PKm, PKe, mdotm, mdote, Vjetm, Vjete = params

                res1 = fL - PKe / (PKe + PKm)

                res2 = mdotm * (Vjetm - Vinf[i]) + mdote * (Vjete - Vinf[i]) - \
                    Dp[i] * (1.0 - fBLIm * Dpp_DP[i] - fBLIe * Dpp_DP[i])

                res3 = PKm - 0.5 * mdotm * (Vjetm**2.0 - Vinf[i]**2.0) - fBLIm * fsurf * Dp[i] * Vinf[i]

                res4 = PKe - 0.5 * mdote * (Vjete**2.0 - Vinf[i]**2.0) - fBLIe * fsurf * Dp[i] * Vinf[i]

                res5 = mdotm - nr_fans_mech * conditions.freestream.density[i] * area_jet_mech * Vjetm

                res6 = mdote - nr_fans_elec * conditions.freestream.density[i] * area_jet_elec * Vjete

                # print(res1,res2,res3,res4,res5,res6)
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
            # args_init = 1 * np.ones(6)  # FIXME - should be more clever guesses
            # [PKm, PKe, mdotm, mdote, Vjetm, Vjete] = remove_negatives(fsolve(power_balance, args_init))
            # sol = root(power_balance, args_init, options={'maxfev':int(1e6), 'xtol': 1e-10})
            sol = root(power_balance, args_init)

            if sol['success'] is not True:
                print()
                raise Exception("Power balance equations not converging: {}".format(sol['message']))
            else:
                [PKm, PKe, mdotm, mdote, Vjetm, Vjete] = sol['x']
                # print(PKm, PKe, mdotm, mdote, Vjetm, Vjete)

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
            # psi = Pbat_i / self.Pbat_max
            # eta_bat = 0.5 + (1.0 - psi) / 2.0
            eta_bat = 0.9  # FIXME Should be calculated

            # Adjust battery power to account for battery efficiency
            # Pbat[i] = Pbat_i / eta_bat
            Pbat[i] = Pbat_i / 1

            # Add turbine power
            Pturb[i] = Pturb_i

            # Calculate vehicle mass rate of change
            mdot_fuel[i] = Pturb_i / (hfuel * eta_th)  # NOTE Could incorporate TSFC here

        print(PKm_tot[0], PKe_tot[0], mdotm_tot[0], mdote_tot[0], Pbat[0], Pturb[0])

        thrust = (PKm_tot + PKe_tot).reshape(nr_elements, 1) / conditions.freestream.velocity * throttle

        # compute power
        power = thrust * u_inf

        # pack outputs
        self.outputs.thrust = thrust
        self.outputs.power = PK_tot
        self.outputs.mdot = mdot_fuel.reshape(nr_elements, 1)
        self.outputs.Pbat = Pbat
        self.outputs.PKm = PKm
        self.outputs.PKe = PKe

    __call__ = compute
