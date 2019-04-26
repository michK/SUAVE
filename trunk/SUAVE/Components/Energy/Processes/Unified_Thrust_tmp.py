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
import pyoptsparse
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
        # PKe = np.zeros(nr_elements)
        # PKm = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)
        Pbat = np.zeros(nr_elements)
        Pturb = np.zeros(nr_elements)

        for i in range(nr_elements):

            def power_balance(xdict):
                """Function to calculate residuals of power balance equations"""
                global mdotm, mdote

                x = xdict['xvars']
                funcs = {}

                PKm   = x[0]
                PKe   = x[1]
                Vjetm = x[2]
                Vjete = x[3]
                # mdotm = x[4]
                # mdote = x[5]
                mdotm = 100
                mdote = 100

                # Derived quantities
                deltaPhiSurf = 0  # NOTE This should somehow be calculated/estimated

                # Equality constraints
                funcs['con1'] = PKm - 0.5 * mdotm * (Vjetm**2.0 - Vinf[i]**2.0) - fBLIm * fsurf * Dpar[i] * Vinf[i]
                funcs['con2'] = PKe - 0.5 * mdote * (Vjete**2.0 - Vinf[i]**2.0) - fBLIe * fsurf * Dpar[i] * Vinf[i]
                funcs['con3'] = hdot[i] * W[i] / Vinf[i] - fBLIm * Dpar[i] - fBLIe * Dpar[i] - deltaPhiSurf / Vinf[i] - \
                    (Vjetm - Vinf[i]) * mdotm - (Vjete - Vinf[i]) * mdote + Dp[i]
                funcs['con4'] = fL - PKe / (PKe + PKm)
                # funcs['con5'] = fL - mdote / (mdotm + mdote)

                # Inequality constraints - NOTE These still use rho_inf
                # funcs['con6'] = mdotm - nr_fans_mech * rho_inf[i] * area_jet_mech * Vjetm
                # funcs['con7'] = mdote - nr_fans_elec * rho_inf[i] * area_jet_elec * Vjete

                # Cost
                funcs['obj'] = (0.5 * (Vjetm - Vinf[i])**2 * mdotm) + (0.5 * (Vjete - Vinf[i])**2 * mdote)
                # funcs['obj'] = 0

                fail = False

                return funcs, fail

            # Define problem
            opt_prob = pyoptsparse.Optimization('Power Balance', power_balance)

            # Define objective
            opt_prob.addObj('obj')

            # Define inputs
            low = np.zeros(4)
            x0 = [100e3, 100e3, 100, 100]
            opt_prob.addVarGroup('xvars', 4, lower=low, value=x0)

            # Define constraints
            # Equality
            opt_prob.addCon('con1', upper=0, lower=0)
            opt_prob.addCon('con2', upper=0, lower=0)
            opt_prob.addCon('con3', upper=0, lower=0)
            opt_prob.addCon('con4', upper=0, lower=0)
            # opt_prob.addCon('con5', upper=0, lower=0)
            # Inequality
            # opt_prob.addCon('con6', upper=0)
            # opt_prob.addCon('con7', upper=0)

            # snopt = pyoptsparse.SNOPT()
            slsqp = pyoptsparse.SLSQP()

            # sol = snopt(opt_prob, sens='FD')
            sol = slsqp(opt_prob, sens='FD')

            # PKm, PKe, Vjetm, Vjete, mdotm, mdote = sol.xStar['xvars']
            PKm, PKe, Vjetm, Vjete = sol.xStar['xvars']

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