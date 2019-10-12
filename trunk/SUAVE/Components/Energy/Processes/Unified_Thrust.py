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
        max_bat_power = self.inputs.max_bat_power
        Cp            = self.inputs.Cp

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
                PK, mdot, Vjetm, Vjete = params

                # Derived quantities
                PKm = (1 - fL) * PK
                PKe = fL * PK
                mdotm = (1 - fL) * mdot
                mdote = fL * mdot
                phi_jet_m = 0.5 * (Vjetm - Vinf[i])**2 * mdotm
                phi_jet_e = 0.5 * (Vjete - Vinf[i])**2 * mdote

                # Residuals
                res1 = PK - 0.5 * mdotm * (Vjetm**2.0 - Vinf[i]**2.0) - fBLIm * fsurf * Dpar[i] * Vinf[i] - \
                    0.5 * mdote * (Vjete**2.0 - Vinf[i]**2.0) - fBLIe * fsurf * Dpar[i] * Vinf[i]
                res2 = phi_jet_m - PKm * (1 - eta_propm)
                res3 = phi_jet_e - PKe * (1 - eta_prope)
                res4 = Dp[i] - (Vjetm - Vinf[i]) * mdotm - (Vjete - Vinf[i]) * mdote + \
                    hdot[i] * W[i] / Vinf[i] - fBLIm * Dpar[i] - fBLIe * Dpar[i] - deltaPhiSurf / Vinf[i]

                power_balance.PKm = PKm
                power_balance.PKe = PKe
                power_balance.mdotm = mdotm
                power_balance.mdote = mdote

                residuals = [res1, res2, res3, res4]
                power_balance.residuals = residuals

                return np.array(residuals, dtype=float).reshape(4,)
            
            args_init = [500e3, 250, 100, 100]
            sol = root(power_balance, args_init, method='hybr')

            if sol['success'] == True:
                [PK, mdot, Vjetm, Vjete] = sol['x']
            else:
                print("Power balance system not converged")
                numerics.converged = False
                [PK, mdot, Vjetm, Vjete] = sol['x']

            # Catch non-physical parameters and set manually
            if power_balance.mdotm <= 0:
                power_balance.mdotm = 0
            
            if power_balance.mdote <= 0:
                power_balance.mdote = 0
            
            if power_balance.PKm <= 0:
                power_balance.PKm = 0
            
            if power_balance.PKe <= 0:
                power_balance.PKe = 0

            if Vjetm <= Vinf[i]:
                Vjetm = Vinf[i]

            if Vjete <= Vinf[i]:
                Vjete = Vinf[i]

            PKm_tot[i] = power_balance.PKm
            PKe_tot[i] = power_balance.PKe
            mdotm_tot[i] = power_balance.mdotm
            mdote_tot[i] = power_balance.mdote
            Vjetm_tot[i] = Vjetm
            Vjete_tot[i] = Vjete

            PK_tot = power_balance.PKm + power_balance.PKe

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
            # mdot_fuel[i] = 2.75 * Cp * Pturb_i
            mdot_fuel[i] = 0.986 * 2.17 * Cp * Pturb_i

        thrust = (PKm_tot + PKe_tot).reshape(nr_elements, 1) / Vinf * throttle

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
