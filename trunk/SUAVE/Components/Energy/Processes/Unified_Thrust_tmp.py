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
import scipy
# import pyoptsparse
from scipy.optimize import root
from scipy.optimize import minimize
from scipy.optimize import basinhopping
# SUAVE imports
from SUAVE.Core import Units
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers, remove_negatives

nan = float('nan')

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
        hdot          = self.inputs.vertical_velocity
        nr_elements   = self.inputs.nr_elements
        fS            = self.inputs.fS
        fL            = self.inputs.fL
        eta_propm     = self.inputs.eta_propm
        eta_prope     = self.inputs.eta_prope
        eta_th        = self.inputs.eta_th
        eta_pe        = self.inputs.eta_pe
        eta_mot       = self.inputs.eta_mot
        eta_fan       = self.inputs.eta_fan
        Vinf          = self.inputs.Vinf
        rho_inf       = self.inputs.rho_inf
        Dp            = self.inputs.Dp
        Dpar          = self.inputs.Dpar
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
        throttle = conditions.propulsion.throttle
        W        = conditions.weights.total_mass * 9.81

        # Set up system of equations to solve power balance
        # Initialize solution arrays
        PKm_tot = np.zeros(nr_elements)
        PKe_tot = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        Vjetm_tot = np.zeros(nr_elements)
        Vjete_tot = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat = np.zeros(nr_elements)
        Pturb = np.zeros(nr_elements)

        # Tentatively assume phi_surf is unaffected
        deltaPhiSurf = 0

        for i in range(nr_elements):

            def power_balance(params):
                """Function to calculate residuals of power balance equations"""
                PKm, PKe, mdotm, mdote, Vjetm, Vjete = params

                # Derived quantities
                phi_jet_m = 0.5 * (Vjetm - Vinf[i])**2 * mdotm
                phi_jet_e = 0.5 * (Vjete - Vinf[i])**2 * mdote

                # Residuals
                res1 = PKm - 0.5 * mdotm * (Vjetm**2.0 - Vinf[i]**2.0) - fBLIm * fsurf * Dpar[i] * Vinf[i]
                res2 = PKe - 0.5 * mdote * (Vjete**2.0 - Vinf[i]**2.0) - fBLIe * fsurf * Dpar[i] * Vinf[i]                
                res3 = phi_jet_m - PKm * (1 - eta_propm)
                res4 = phi_jet_e - PKe * (1 - eta_prope)
                res5 = fL - PKe / (PKe + PKm)
                res6 = Dp[i] - (Vjetm - Vinf[i]) * (1 - fL) * mdotm - (Vjete - Vinf[i]) * fL * mdote + \
                    hdot[i] * W[i] / Vinf[i] - fBLIm * Dpar[i] - fBLIe * Dpar[i] - deltaPhiSurf / Vinf[i]

                residuals = [res1, res2, res3, res4, res5, res6]

                return residuals
            
            args_init = [150000.0, 150000.0, 100.0, 100.0, 100.0, 100.0]
            scale = args_init            
            sol = root(power_balance, args_init, method='hybr')

            if sol['success'] == True:
                [PKm, PKe, mdotm, mdote, Vjetm, Vjete] = sol['x']
            else:
                print("Power balance system not convered")
                [PKm, PKe, mdotm, mdote, Vjetm, Vjete] = np.ones(6) * nan

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

        thrust = (PKm_tot + PKe_tot).reshape(nr_elements, 1) / Vinf * throttle

        # print(PKm_tot[i-2] ,PKe_tot[i-2], mdotm_tot[i-2], mdote_tot[i-2], Vjetm_tot[i-2], Vjete_tot[i-2])
        # compute power
        power = thrust * Vinf

        # pack outputs
        self.outputs.thrust = thrust
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
