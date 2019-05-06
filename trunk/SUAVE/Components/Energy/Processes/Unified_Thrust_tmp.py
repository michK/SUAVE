# @ingroup Components-Energy-Processes
# Unified_Thrust.py
#
# Created:  Feb 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np
# import pyoptsparse
from scipy.optimize import root
# SUAVE imports
from SUAVE.Core import Units
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers, remove_negatives

# ----------------------------------------------------------------------
#  Thrust Process
# ----------------------------------------------------------------------
# @ingroup Components-Energy-Processes


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
        self.nexus = None

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
        hdot = self.inputs.vertical_velocity
        nr_elements = self.inputs.nr_elements
        fS = self.inputs.fS
        fL = self.inputs.fL
        eta_propm = self.inputs.eta_propm
        eta_prope = self.inputs.eta_prope
        eta_th = self.inputs.eta_th
        eta_pe = self.inputs.eta_pe
        eta_mot = self.inputs.eta_mot
        eta_fan = self.inputs.eta_fan
        Vinf = self.inputs.Vinf
        rho_inf = self.inputs.rho_inf
        Dp = self.inputs.Dp
        Dpar = self.inputs.Dpar
        Dpp_DP = self.inputs.Dpp_DP
        fBLIe = self.inputs.fBLIe
        fBLIm = self.inputs.fBLIm
        fsurf = self.inputs.fsurf
        nr_fans_mech = self.inputs.nr_fans_mech
        nr_fans_elec = self.inputs.nr_fans_elec
        area_jet_mech = self.inputs.area_jet_mech
        area_jet_elec = self.inputs.area_jet_elec
        hfuel = self.inputs.hfuel

        # Unpack from conditions
        throttle = conditions.propulsion.throttle
        W = conditions.weights.total_mass * 9.81

        # Set up system of equations to solve power balance
        # Initialize solution arrays
        PKm_tot = np.zeros(nr_elements)
        PKe_tot = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        Vjetm_tot = np.zeros(nr_elements)
        Vjete_tot = np.zeros(nr_elements)
        # PKe = np.zeros(nr_elements)
        # PKm = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat = np.zeros(nr_elements)
        Pturb = np.zeros(nr_elements)

        # Tentatively assume phi_surf is unaffected
        deltaPhiSurf = 0

        for i in range(nr_elements):

            T = Dp[i]

            def power_balance_m(params_m):
                """
                Function to calculate residuals of mechanical side power balance equations
                """
                global Tm
                mdotm, Vjetm = params_m

                Tm = (1 - fL) * T

                # Residuals
                res1 = Tm - mdotm * (Vjetm - Vinf[i])
                res2 = Tm * Vinf[i] - 0.5 * eta_propm * mdotm * (Vjetm**2 - Vinf[i]**2)

                res_m = np.array([res1, res2])

                return res_m.reshape(2,)

            def power_balance_e(params_e):
                """
                Function to calculate residuals of electrical side power balance equations
                """
                global Te
                mdote, Vjete = params_e

                Te = fL * T

                # Residuals
                res3 = Te - mdote * (Vjete - Vinf[i])
                res4 = Te * Vinf[i] - 0.5 * eta_prope * mdote * (Vjete**2 - Vinf[i]**2)

                res_e = np.array([res3, res4])

                return res_e.reshape(2,)

            args_init_m = [60.0, 30.0]  # FIXME - should be more clever guesses
            args_init_e = [60.0, 30.0]
            scale_m = args_init_m
            scale_e = args_init_e
            sol_m = root(power_balance_m, args_init_m, method='hybr', options={'diag': scale_m, 'eps': 1})
            sol_e = root(power_balance_e, args_init_e, method='hybr', options={'diag': scale_e, 'eps': 1})
            # sol_m = root(power_balance_m, args_init_m, method='hybr')
            # sol_e = root(power_balance_e, args_init_e, method='hybr')

            if sol_m['success'] == True:
                [mdotm, Vjetm] = sol_m['x']
            else:
                raise Exception("Power balance system not converging (mech side)")

            if sol_e['success'] == True:
                [mdote, Vjete] = sol_e['x']
            else:
                raise Exception("Power balance system not converging (elec side)")

            PKm = Tm * Vinf[i]
            PKe = Te * Vinf[i]

            PKm_tot[i] = PKm
            PKe_tot[i] = PKe
            mdotm_tot[i] = mdotm
            mdote_tot[i] = mdote
            Vjetm_tot[i] = Vjetm
            Vjete_tot[i] = Vjete

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
            Pbat[i] = Pbat_i / eta_bat

            # Add turbine power
            Pturb[i] = Pturb_i

            # Calculate vehicle mass rate of change
            mdot_fuel[i] = Pturb_i / (hfuel * eta_th)  # NOTE Could incorporate TSFC here

        # thrust = (PKm_tot + PKe_tot).reshape(nr_elements, 1) / Vinf * throttle
        thrust = T

        # print(PKm_tot[i-2] ,PKe_tot[i-2], mdotm_tot[i-2], mdote_tot[i-2], Vjetm_tot[i-2], Vjete_tot[i-2])
        # compute power
        power = thrust * Vinf

        # pack outputs
        self.outputs.thrust = thrust
        self.outputs.PKm = PKm
        self.outputs.PKe = PKe
        self.outputs.power = PK_tot
        self.outputs.mdot = mdot_fuel.reshape(nr_elements, 1)
        self.outputs.Pbat = Pbat
        self.outputs.PKm_tot = PKm_tot
        self.outputs.PKe_tot = PKe_tot
        self.outputs.mdotm_tot = mdotm_tot
        self.outputs.mdote_tot = mdote_tot
        self.outputs.Vjetm_tot = Vjetm_tot
        self.outputs.Vjete_tot = Vjete_tot

    __call__ = compute


if __name__ == '__main__':
    main()
