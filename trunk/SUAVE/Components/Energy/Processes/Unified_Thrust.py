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
from scipy.optimize import root

# SUAVE imports
from SUAVE.Core import Units
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers

# ----------------------------------------------------------------------
#  Thrust Process
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Processes
class Unified_Thrust(Energy_Component):
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

    def compute(self, conditions, numerics):
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
        """
        # Unpack the values

        # Unpack from inputs
        nr_prop_m      = self.inputs.nr_prop_m
        nr_prop_e      = self.inputs.nr_prop_e
        Dfan_m         = self.inputs.Dfan_m
        Dfan_e         = self.inputs.Dfan_e
        Acapm          = self.inputs.Acapm
        Acape          = self.inputs.Acape
        hdot           = self.inputs.vertical_velocity
        nr_elements    = self.inputs.nr_elements
        fS             = self.inputs.fS
        fL             = self.inputs.fL
        eta_propm      = self.inputs.eta_propm
        eta_prope      = self.inputs.eta_prope
        eta_th         = self.inputs.eta_th
        eta_pe         = self.inputs.eta_pe
        eta_mot        = self.inputs.eta_mot
        eta_fan        = self.inputs.eta_fan
        Vinf           = self.inputs.Vinf
        rho_inf        = self.inputs.rho_inf
        Dp             = self.inputs.Dp
        Dpar           = self.inputs.Dpar
        Dpp_DP         = self.inputs.Dpp_DP
        fBLIe          = self.inputs.fBLIe
        fBLIm          = self.inputs.fBLIm
        fsurf          = self.inputs.fsurf
        max_bat_power  = self.inputs.max_bat_power
        Cp             = self.inputs.Cp
        Cp_factor      = self.inputs.Cp_factor
        power_bal_init = self.inputs.power_bal_init

        # Unpack from conditions
        throttle = conditions.propulsion.throttle
        W        = conditions.weights.total_mass * 9.81

        # Set up system of equations to solve power balance
        # Initialize solution arrays
        PKm_tot   = np.zeros(nr_elements)
        PKe_tot   = np.zeros(nr_elements)
        mdottot   = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        Vjetm_tot = np.zeros(nr_elements)
        Vjete_tot = np.zeros(nr_elements)
        eta_p_tot = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat      = np.zeros(nr_elements)
        Pturb     = np.zeros(nr_elements)

        # Tentatively assume phi_surf is unaffected
        deltaPhiSurf = 0
        # Compute overall fBLI
        fBLI = fBLIm + fBLIe
        # Compute total jet area
        Ajet = 0.6 * (Acapm + Acape)

        for i in range(nr_elements):
            def power_balance(params):
                """Function to calculate residuals of power balance equations"""
                PK_tot, mdot_tot, Vjet, eta_p, phi_jet = params

                # Residuals
                res1 = PK_tot - 0.5 * mdot_tot * (Vjet**2.0 - Vinf[i]**2.0) - fBLI * fsurf * Dpar[i] * Vinf[i]
                res2 = Dp[i] - (Vjet - Vinf[i]) * mdot_tot + hdot[i] * W[i] / Vinf[i] - fBLI * Dpar[i] - deltaPhiSurf / Vinf[i]
                res3 = eta_p - (PK_tot - phi_jet) / PK_tot
                res4 = phi_jet - 0.5 * (Vjet - Vinf[i])**2 * mdot_tot
                res5 = mdot_tot - (rho_inf[i] * Vjet * Ajet)

                power_balance.PK  = PK_tot
                power_balance.mdot = mdot_tot
                power_balance.Vjet = Vjet
                power_balance.eta_p = eta_p

                residuals = [res1, res2, res3, res4, res5]
                power_balance.residuals = residuals

                return np.array(residuals, dtype=float).reshape(5,)

            args_init = power_bal_init
            sol = root(power_balance, args_init, method='hybr')

            if sol['success'] == True:
                [PK, mdot, Vjet, eta_p, phi_jet] = sol['x']
            else:
                print("Power balance system not converged")
                numerics.converged = False
                [PK, mdot, Vjet, eta_p, phi_jet] = sol['x']

            # Catch non-physical parameters and set manually
            if power_balance.mdot <= 0:
                power_balance.mdotm = 0

            if power_balance.PK <= 0:
                power_balance.PK = 0

            if Vjet <= Vinf[i]:
                Vjet = Vinf[i]

            PKm_tot[i] = (1 - fL) * power_balance.PK
            PKe_tot[i] = fL * power_balance.PK
            mdottot[i] = mdot
            mdotm_tot[i] = (1 - fL) * power_balance.mdot
            mdote_tot[i] = fL * power_balance.mdot
            Vjetm_tot[i] = power_balance.Vjet
            Vjete_tot[i] = power_balance.Vjet
            eta_p_tot[i] = power_balance.eta_p

            # PK_tot = power_balance.PKm + power_balance.PKe
            PK_tot = power_balance.PK
            [PKm_i, PKe_i, Pturb_i, Pbat_i, PfanM_i, PfanE_i, Pmot_i, Pinv_i, Plink] = \
                calculate_powers(PK_tot, fS, fL, eta_pe, eta_mot, eta_fan)

            PKm = PKm_i
            PKe = PKe_i

            # Ragone relation for battery efficiency
            psi = Pbat_i / max_bat_power
            eta_bat = 0.5 + (1.0 - psi) / 2.0

            # Adjust battery power to account for battery efficiency
            Pbat[i] = Pbat_i / eta_bat

            # Add turbine power
            Pturb[i] = Pturb_i

            # Calculate fuel flow through turbine
            mdot_fuel[i] = Cp_factor * Cp * Pturb_i

        thrust = (PKm_tot + PKe_tot).reshape(nr_elements, 1) / Vinf * throttle

        # compute power
        power = thrust * Vinf

        # pack outputs
        self.outputs.thrust    = thrust
        self.outputs.power     = PK_tot
        self.outputs.mdot      = mdot_fuel.reshape(nr_elements, 1)
        self.outputs.Pbat      = Pbat
        self.outputs.PKm_tot   = PKm_tot
        self.outputs.PKe_tot   = PKe_tot
        self.outputs.mdottot   = mdottot
        self.outputs.mdotm_tot = mdotm_tot
        self.outputs.mdote_tot = mdote_tot
        self.outputs.Vjetm_tot = Vjetm_tot
        self.outputs.Vjete_tot = Vjete_tot
        self.outputs.eta_p_tot = eta_p_tot

    __call__ = compute


if __name__ == '__main__':
    main()
